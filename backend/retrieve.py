"""
Retrieval Pipeline Validation Tool
Validates RAG retrieval quality by running test queries against Qdrant
"""

import os
import sys
import argparse
import statistics
from typing import List, Dict, Set, Tuple
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

import requests
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import FieldCondition, Filter, MatchValue
from loguru import logger
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Environment variables (required)
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
DOCUSAURUS_BASE_URL = os.getenv("DOCUSAURUS_BASE_URL", "https://physical-ai-robotics-textbook-xi.vercel.app")

# Validate required environment variables
def validate_environment():
    """Validate that all required environment variables are set."""
    missing = []
    if not COHERE_API_KEY:
        missing.append("COHERE_API_KEY")
    if not QDRANT_URL:
        missing.append("QDRANT_URL")
    if not QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY")

    if missing:
        logger.error(f"Missing required environment variables: {', '.join(missing)}")
        logger.error("Please check your .env file in backend/ directory")
        sys.exit(1)

# Configuration constants
SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.70"))
TOP_K_RESULTS = int(os.getenv("TOP_K_RESULTS", "5"))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

# Configure logger
logger.remove()
logger.add(
    lambda msg: print(msg, end=""),
    format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>",
    level=LOG_LEVEL
)


def init_qdrant_client() -> QdrantClient:
    """
    Initialize and return Qdrant client with error handling.

    Returns:
        QdrantClient: Connected Qdrant client instance

    Raises:
        ConnectionError: If connection to Qdrant fails
    """
    try:
        client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
        # Verify connection by getting collections
        client.get_collections()
        logger.info(f"Connected to Qdrant at {QDRANT_URL}")
        return client
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {e}")
        logger.error("Check QDRANT_URL and QDRANT_API_KEY in .env file")
        raise ConnectionError(f"Qdrant connection failed: {e}")


# ============================================================================
# Phase 2: Foundational Functions
# ============================================================================

def embed_query(query_text: str) -> List[float]:
    """
    Generate Cohere embedding for query text.

    Args:
        query_text: Query string to embed

    Returns:
        1024-dimensional embedding vector

    Raises:
        ValueError: If query_text is empty
        Exception: If Cohere API call fails
    """
    if not query_text or not query_text.strip():
        raise ValueError("Query text cannot be empty")

    try:
        co = cohere.Client(COHERE_API_KEY)
        response = co.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"  # Different from document ingestion
        )
        return response.embeddings[0]
    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        raise


def parse_query_result(qdrant_result) -> Dict:
    """
    Extract metadata from Qdrant search result.

    Args:
        qdrant_result: Qdrant ScoredPoint object

    Returns:
        Dict with chunk_id, chunk_text, similarity_score, and metadata fields
    """
    return {
        "chunk_id": qdrant_result.id,
        "chunk_text": qdrant_result.payload.get("chunk_text", ""),
        "similarity_score": qdrant_result.score,
        "source_url": qdrant_result.payload.get("source_url", ""),
        "chapter_id": qdrant_result.payload.get("chapter_id", ""),
        "module_name": qdrant_result.payload.get("module_name", ""),
        "heading_hierarchy": qdrant_result.payload.get("heading_hierarchy", []),
        "token_count": qdrant_result.payload.get("token_count", 0),
        "chunk_index": qdrant_result.payload.get("chunk_index", 0)
    }


def query_qdrant(
    query_text: str,
    top_k: int = TOP_K_RESULTS,
    threshold: float = SIMILARITY_THRESHOLD,
    qdrant_client: QdrantClient = None
) -> List[Dict]:
    """
    Generate query embedding and search Qdrant for similar chunks.

    Args:
        query_text: The search query string
        top_k: Number of top results to return (default: 5)
        threshold: Minimum similarity score filter (default: 0.70)
        qdrant_client: Optional Qdrant client instance

    Returns:
        List of retrieved chunks with similarity scores, sorted by score descending

    Raises:
        ValueError: If query_text is empty
        ConnectionError: If Qdrant is unreachable
        Exception: If Cohere embedding generation fails
    """
    if not query_text or not query_text.strip():
        raise ValueError("Query text cannot be empty")

    try:
        # Generate query embedding
        query_embedding = embed_query(query_text)

        # Initialize client if not provided
        if qdrant_client is None:
            qdrant_client = init_qdrant_client()

        # Search Qdrant
        results = qdrant_client.search(
            collection_name="rag_embedding",
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=threshold
        )

        # Parse and return results
        return [parse_query_result(result) for result in results]

    except ValueError as e:
        raise
    except ConnectionError as e:
        logger.error(f"Qdrant connection error: {e}")
        raise
    except Exception as e:
        logger.error(f"Query failed: {e}")
        raise


def get_all_urls(base_url: str) -> List[str]:
    """
    Fetch sitemap.xml and extract all /docs/* URLs.

    Args:
        base_url: Docusaurus base URL

    Returns:
        List of documentation URLs

    Raises:
        requests.RequestException: If sitemap fetch fails
    """
    logger.info(f"Fetching sitemap from {base_url}/sitemap.xml")

    try:
        sitemap_url = f"{base_url}/sitemap.xml"
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse XML sitemap
        root = ET.fromstring(response.content)

        # Extract URLs (handle XML namespaces)
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = []

        for url_elem in root.findall('.//ns:url/ns:loc', namespace):
            url = url_elem.text
            # Filter: only /docs/* paths
            if url and '/docs/' in url:
                urls.append(url)

        logger.info(f"Discovered {len(urls)} documentation URLs")
        return urls

    except Exception as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        raise


# ============================================================================
# Phase 3: User Story 1 - Query Validation (MVP)
# ============================================================================

# Test queries covering major modules
TEST_QUERIES = [
    {
        "query_text": "What is ROS 2?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70,
        "description": "ROS 2 basics"
    },
    {
        "query_text": "How do I create a URDF file for a humanoid robot?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70,
        "description": "URDF humanoid creation"
    },
    {
        "query_text": "What is rclpy and how is it used for robot control?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70,
        "description": "rclpy control basics"
    },
    {
        "query_text": "What is Isaac Sim?",
        "expected_module": "Module 3 Isaac",
        "min_similarity": 0.70,
        "description": "Isaac Sim introduction"
    },
    {
        "query_text": "How do I generate synthetic data in Isaac Sim?",
        "expected_module": "Module 3 Isaac",
        "min_similarity": 0.70,
        "description": "Synthetic data generation"
    },
    {
        "query_text": "What is VSLAM and how does it work with Isaac ROS?",
        "expected_module": "Module 3 Isaac",
        "min_similarity": 0.70,
        "description": "VSLAM with Isaac ROS"
    },
    {
        "query_text": "How do I integrate Nav2 with Isaac Sim?",
        "expected_module": "Module 3 Isaac",
        "min_similarity": 0.70,
        "description": "Nav2 integration"
    },
    {
        "query_text": "What are Vision-Language-Action models?",
        "expected_module": "Module 4 Vla",
        "min_similarity": 0.70,
        "description": "VLA fundamentals"
    },
    {
        "query_text": "How do I perform depth perception and mapping?",
        "expected_module": "Module 3 Isaac",
        "min_similarity": 0.70,
        "description": "Depth perception basics"
    },
    {
        "query_text": "What are the key concepts in the Physical AI textbook?",
        "expected_module": "intro",  # Introduction page
        "min_similarity": 0.65,  # Broader query, slightly lower threshold
        "description": "Textbook overview"
    }
]


def validate_query_result(query: Dict, results: List[Dict]) -> Dict:
    """
    Check if query results meet expected criteria.

    Args:
        query: TestQuery dict with expected_module and min_similarity
        results: List of QueryResult dicts from Qdrant

    Returns:
        QueryValidationResult dict with passed, pass_reason, fail_reason, etc.
    """
    if not results:
        return {
            "query": query,
            "top_results": [],
            "passed": False,
            "pass_reason": None,
            "fail_reason": "No results returned above threshold",
            "best_similarity": 0.0,
            "module_match": False
        }

    # Find best similarity and check module match
    best_similarity = max(r["similarity_score"] for r in results)
    module_match = any(
        query["expected_module"].lower() in r["module_name"].lower() or
        query["expected_module"].lower() in r["chapter_id"].lower()
        for r in results
    )

    passed = best_similarity >= query["min_similarity"] and module_match

    if passed:
        pass_reason = f"Found relevant chunk (similarity={best_similarity:.2f}) in expected module '{query['expected_module']}'"
        fail_reason = None
    else:
        pass_reason = None
        if not module_match:
            fail_reason = f"Expected module '{query['expected_module']}' not found in top results"
        else:
            fail_reason = f"Similarity score {best_similarity:.2f} below threshold {query['min_similarity']}"

    return {
        "query": query,
        "top_results": results,
        "passed": passed,
        "pass_reason": pass_reason,
        "fail_reason": fail_reason,
        "best_similarity": best_similarity,
        "module_match": module_match
    }


def run_validation_queries(test_queries: List[Dict], qdrant_client: QdrantClient) -> List[Dict]:
    """
    Execute all test queries and validate results.

    Args:
        test_queries: List of TestQuery dicts
        qdrant_client: Qdrant client instance

    Returns:
        List of QueryValidationResult dicts
    """
    logger.info(f"Running {len(test_queries)} validation queries...")
    results = []

    for idx, query in enumerate(test_queries, 1):
        logger.info(f"[{idx}/{len(test_queries)}] Query: \"{query['query_text']}\"")

        try:
            # Query Qdrant
            query_results = query_qdrant(
                query["query_text"],
                top_k=TOP_K_RESULTS,
                threshold=query["min_similarity"],
                qdrant_client=qdrant_client
            )

            # Validate
            validation = validate_query_result(query, query_results)
            results.append(validation)

            # Log result
            if validation["passed"]:
                logger.info(f"  PASS (similarity={validation['best_similarity']:.2f})")
            else:
                logger.warning(f"  FAIL: {validation['fail_reason']}")

        except Exception as e:
            logger.error(f"  ERROR: {e}")
            results.append({
                "query": query,
                "top_results": [],
                "passed": False,
                "pass_reason": None,
                "fail_reason": f"Query execution failed: {e}",
                "best_similarity": 0.0,
                "module_match": False
            })

    return results


def print_query_validation_report(results: List[Dict]):
    """
    Print summary report for query validation.

    Args:
        results: List of QueryValidationResult dicts
    """
    total = len(results)
    passed = sum(1 for r in results if r["passed"])
    failed = total - passed
    pass_rate = (passed / total * 100) if total > 0 else 0

    logger.info("\n" + "=" * 80)
    logger.info("Query Validation Results")
    logger.info("=" * 80)

    for idx, result in enumerate(results, 1):
        status = "PASS" if result["passed"] else "FAIL"
        query_text = result["query"]["query_text"][:50] + "..." if len(result["query"]["query_text"]) > 50 else result["query"]["query_text"]
        logger.info(f"[{idx}] {status}: \"{query_text}\"")

        if not result["passed"] and result["fail_reason"]:
            logger.info(f"    Reason: {result['fail_reason']}")

    logger.info("=" * 80)
    logger.info(f"Total Queries:  {total}")
    logger.info(f"Passed:         {passed}")
    logger.info(f"Failed:         {failed}")
    logger.info(f"Pass Rate:      {pass_rate:.1f}%")
    logger.info("=" * 80)

    if pass_rate >= 90:
        logger.success(f"[SUCCESS] Query validation passed (>= 90%)")
    else:
        logger.error(f"[FAIL] Query validation failed (< 90%)")


# ============================================================================
# Phase 4: User Story 2 - Coverage and Completeness Validation (Priority: P2)
# ============================================================================

def get_indexed_urls(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> Set[str]:
    """
    Query Qdrant for all distinct source_url values.

    Args:
        qdrant_client: Qdrant client instance
        collection_name: Collection to query (default: "rag_embedding")

    Returns:
        Set of unique source URLs found in Qdrant
    """
    try:
        urls = set()
        offset = None

        while True:
            # Scroll through all points in collection
            scroll_result = qdrant_client.scroll(
                collection_name=collection_name,
                limit=100,
                offset=offset,
                with_payload=True,
                with_vectors=False  # Don't need vectors, only metadata
            )

            points, next_offset = scroll_result

            # Extract source URLs
            for point in points:
                source_url = point.payload.get("source_url")
                if source_url:
                    urls.add(source_url)

            # Check if we've reached the end
            if next_offset is None:
                break
            offset = next_offset

        logger.info(f"Found {len(urls)} unique URLs in Qdrant")
        return urls

    except Exception as e:
        logger.error(f"Failed to get indexed URLs from Qdrant: {e}")
        raise


def analyze_coverage(sitemap_url: str, qdrant_client: QdrantClient) -> Dict:
    """
    Compare sitemap URLs to Qdrant indexed URLs.

    Args:
        sitemap_url: Base URL for sitemap (e.g., "https://example.com")
        qdrant_client: Qdrant client instance

    Returns:
        CoverageReport dict with total_sitemap_urls, indexed_urls, missing_urls, extra_urls, coverage_percentage
    """
    try:
        # Get URLs from sitemap
        sitemap_urls = set(get_all_urls(sitemap_url))

        # Get URLs from Qdrant
        indexed_urls = get_indexed_urls(qdrant_client)

        # Calculate differences
        missing_urls = sitemap_urls - indexed_urls  # In sitemap but not in Qdrant
        extra_urls = indexed_urls - sitemap_urls    # In Qdrant but not in sitemap

        # Calculate coverage percentage
        coverage_percentage = (len(indexed_urls) / len(sitemap_urls) * 100) if sitemap_urls else 0

        return {
            "total_sitemap_urls": len(sitemap_urls),
            "indexed_urls": len(indexed_urls),
            "missing_urls": list(missing_urls),
            "extra_urls": list(extra_urls),
            "coverage_percentage": coverage_percentage
        }

    except Exception as e:
        logger.error(f"Coverage analysis failed: {e}")
        raise


def print_coverage_report(coverage: Dict):
    """
    Print coverage analysis report.

    Args:
        coverage: CoverageReport dict from analyze_coverage()
    """
    logger.info("\n" + "=" * 80)
    logger.info("Coverage Analysis")
    logger.info("=" * 80)

    logger.info(f"Sitemap URLs:       {coverage['total_sitemap_urls']}")
    logger.info(f"Indexed URLs:       {coverage['indexed_urls']}")
    logger.info(f"Coverage:           {coverage['coverage_percentage']:.1f}%")

    if coverage['missing_urls']:
        logger.warning(f"Missing URLs ({len(coverage['missing_urls'])}): Not in Qdrant but in sitemap")
        for url in coverage['missing_urls'][:10]:  # Show first 10
            logger.warning(f"  - {url}")
        if len(coverage['missing_urls']) > 10:
            logger.warning(f"  ... and {len(coverage['missing_urls']) - 10} more")

    if coverage['extra_urls']:
        logger.info(f"Extra URLs ({len(coverage['extra_urls'])}): In Qdrant but not in sitemap")
        for url in coverage['extra_urls'][:10]:
            logger.info(f"  - {url}")
        if len(coverage['extra_urls']) > 10:
            logger.info(f"  ... and {len(coverage['extra_urls']) - 10} more")

    logger.info("=" * 80)

    if coverage['coverage_percentage'] >= 100:
        logger.success(f"[SUCCESS] Full coverage achieved (100%)")
    elif coverage['coverage_percentage'] >= 90:
        logger.warning(f"[WARNING] Coverage acceptable but not complete (>= 90%)")
    else:
        logger.error(f"[FAIL] Coverage below threshold (< 90%)")


# ============================================================================
# Phase 5: User Story 3 - Embedding Quality Metrics (Priority: P3)
# ============================================================================

# Required metadata fields
REQUIRED_METADATA_FIELDS = [
    "source_url",
    "chapter_id",
    "module_name",
    "heading_hierarchy",
    "token_count",
    "chunk_index"
]


def check_metadata_completeness(payload: Dict) -> bool:
    """
    Verify all required metadata fields are present.

    Args:
        payload: Qdrant point payload

    Returns:
        True if all required fields present, False otherwise
    """
    return all(field in payload for field in REQUIRED_METADATA_FIELDS)


def calculate_token_distribution(token_counts: List[int]) -> Dict:
    """
    Calculate token distribution statistics.

    Args:
        token_counts: List of token counts from all chunks

    Returns:
        Dict with min, max, avg, median token counts
    """
    if not token_counts:
        return {"min": 0, "max": 0, "avg": 0, "median": 0}

    return {
        "min": min(token_counts),
        "max": max(token_counts),
        "avg": statistics.mean(token_counts),
        "median": statistics.median(token_counts)
    }


def analyze_quality_metrics(qdrant_client: QdrantClient, query_results: List[Dict]) -> Dict:
    """
    Analyze embedding quality metrics.

    Args:
        qdrant_client: Qdrant client instance
        query_results: List of QueryValidationResult dicts from query validation

    Returns:
        QualityMetrics dict with metadata_completeness, token_distribution, avg_similarity
    """
    try:
        # Collect metadata completeness stats
        total_chunks = 0
        complete_chunks = 0
        token_counts = []

        offset = None

        while True:
            # Scroll through all points
            scroll_result = qdrant_client.scroll(
                collection_name="rag_embedding",
                limit=100,
                offset=offset,
                with_payload=True,
                with_vectors=False
            )

            points, next_offset = scroll_result

            for point in points:
                total_chunks += 1

                # Check metadata completeness
                if check_metadata_completeness(point.payload):
                    complete_chunks += 1

                # Collect token counts
                token_count = point.payload.get("token_count", 0)
                if token_count > 0:
                    token_counts.append(token_count)

            if next_offset is None:
                break
            offset = next_offset

        # Calculate metadata completeness rate
        metadata_completeness_rate = (complete_chunks / total_chunks * 100) if total_chunks > 0 else 0

        # Calculate token distribution
        token_distribution = calculate_token_distribution(token_counts)

        # Calculate average similarity from query results
        all_similarities = []
        for result in query_results:
            if result["passed"] and result["best_similarity"] > 0:
                all_similarities.append(result["best_similarity"])

        avg_similarity = statistics.mean(all_similarities) if all_similarities else 0.0

        return {
            "total_chunks": total_chunks,
            "complete_chunks": complete_chunks,
            "metadata_completeness_rate": metadata_completeness_rate,
            "token_distribution": token_distribution,
            "avg_similarity": avg_similarity,
            "similarity_distribution": {
                "min": min(all_similarities) if all_similarities else 0.0,
                "max": max(all_similarities) if all_similarities else 0.0,
                "median": statistics.median(all_similarities) if all_similarities else 0.0
            }
        }

    except Exception as e:
        logger.error(f"Quality metrics analysis failed: {e}")
        raise


def print_quality_metrics_report(metrics: Dict):
    """
    Print quality metrics report.

    Args:
        metrics: QualityMetrics dict from analyze_quality_metrics()
    """
    logger.info("\n" + "=" * 80)
    logger.info("Quality Metrics")
    logger.info("=" * 80)

    logger.info(f"Total Chunks:           {metrics['total_chunks']}")
    logger.info(f"Complete Metadata:      {metrics['complete_chunks']} ({metrics['metadata_completeness_rate']:.1f}%)")

    token_dist = metrics['token_distribution']
    logger.info(f"Token Distribution:")
    logger.info(f"  Min:                  {token_dist['min']}")
    logger.info(f"  Max:                  {token_dist['max']}")
    logger.info(f"  Average:              {token_dist['avg']:.1f}")
    logger.info(f"  Median:               {token_dist['median']:.1f}")

    logger.info(f"Average Similarity:     {metrics['avg_similarity']:.2f}")

    sim_dist = metrics['similarity_distribution']
    logger.info(f"Similarity Distribution:")
    logger.info(f"  Min:                  {sim_dist['min']:.2f}")
    logger.info(f"  Max:                  {sim_dist['max']:.2f}")
    logger.info(f"  Median:               {sim_dist['median']:.2f}")

    logger.info("=" * 80)

    if metrics['metadata_completeness_rate'] >= 100:
        logger.success(f"[SUCCESS] All chunks have complete metadata (100%)")
    else:
        logger.error(f"[FAIL] Some chunks missing metadata ({metrics['metadata_completeness_rate']:.1f}%)")


# ============================================================================
# Phase 6: Main Orchestration & Reporting
# ============================================================================

def generate_validation_report(
    query_results: List[Dict],
    coverage: Dict,
    quality: Dict,
    duration: float
) -> Dict:
    """
    Aggregate all validation results into final report.

    Args:
        query_results: List of QueryValidationResult dicts
        coverage: CoverageReport dict
        quality: QualityMetrics dict
        duration: Total runtime in seconds

    Returns:
        ValidationReport dict
    """
    # Calculate query validation stats
    total_queries = len(query_results)
    passed_queries = sum(1 for r in query_results if r["passed"])
    pass_rate = (passed_queries / total_queries * 100) if total_queries > 0 else 0

    # Overall success criteria
    success = (
        pass_rate >= 90 and
        coverage['coverage_percentage'] >= 100 and
        quality['metadata_completeness_rate'] >= 100
    )

    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "duration_seconds": duration,
        "query_validation": {
            "total": total_queries,
            "passed": passed_queries,
            "failed": total_queries - passed_queries,
            "pass_rate": pass_rate
        },
        "coverage": coverage,
        "quality": quality,
        "overall_success": success
    }


def print_validation_report(report: Dict):
    """
    Print final validation report.

    Args:
        report: ValidationReport dict from generate_validation_report()
    """
    logger.info("\n" + "=" * 80)
    logger.info("VALIDATION REPORT")
    logger.info("=" * 80)
    logger.info(f"Timestamp:          {report['timestamp']}")
    logger.info(f"Duration:           {report['duration_seconds']:.1f} seconds")
    logger.info("=" * 80)

    # Query validation summary
    qv = report['query_validation']
    logger.info(f"Query Validation:   {qv['passed']}/{qv['total']} passed ({qv['pass_rate']:.1f}%)")

    # Coverage summary
    cov = report['coverage']
    logger.info(f"Coverage:           {cov['indexed_urls']}/{cov['total_sitemap_urls']} URLs ({cov['coverage_percentage']:.1f}%)")

    # Quality summary
    qual = report['quality']
    logger.info(f"Metadata Complete:  {qual['metadata_completeness_rate']:.1f}%")
    logger.info(f"Avg Similarity:     {qual['avg_similarity']:.2f}")
    logger.info(f"Token Avg/Median:   {qual['token_distribution']['avg']:.0f}/{qual['token_distribution']['median']:.0f}")

    logger.info("=" * 80)

    if report['overall_success']:
        logger.success("[SUCCESS] All validation criteria passed!")
        logger.success("  ✓ Query pass rate >= 90%")
        logger.success("  ✓ Coverage = 100%")
        logger.success("  ✓ Metadata completeness = 100%")
    else:
        logger.error("[FAIL] Validation failed - see details above")
        if qv['pass_rate'] < 90:
            logger.error(f"  ✗ Query pass rate {qv['pass_rate']:.1f}% < 90%")
        if cov['coverage_percentage'] < 100:
            logger.error(f"  ✗ Coverage {cov['coverage_percentage']:.1f}% < 100%")
        if qual['metadata_completeness_rate'] < 100:
            logger.error(f"  ✗ Metadata completeness {qual['metadata_completeness_rate']:.1f}% < 100%")

    logger.info("=" * 80)


def main():
    """
    Main orchestration function - run all validation phases.
    """
    import time
    start_time = time.time()

    try:
        # Validate environment
        validate_environment()

        logger.info("=" * 80)
        logger.info("Retrieval Pipeline Validation")
        logger.info("=" * 80)

        # Initialize Qdrant client
        qdrant_client = init_qdrant_client()

        # Phase 1: Query Validation (User Story 1 - MVP)
        logger.info("\n[Phase 1] Running Query Validation...")
        query_results = run_validation_queries(TEST_QUERIES, qdrant_client)
        print_query_validation_report(query_results)

        # Phase 2: Coverage Analysis (User Story 2)
        logger.info("\n[Phase 2] Running Coverage Analysis...")
        coverage = analyze_coverage(DOCUSAURUS_BASE_URL, qdrant_client)
        print_coverage_report(coverage)

        # Phase 3: Quality Metrics (User Story 3)
        logger.info("\n[Phase 3] Running Quality Metrics Analysis...")
        quality = analyze_quality_metrics(qdrant_client, query_results)
        print_quality_metrics_report(quality)

        # Generate and print final report
        duration = time.time() - start_time
        report = generate_validation_report(query_results, coverage, quality, duration)
        print_validation_report(report)

        # Exit with appropriate code
        sys.exit(0 if report['overall_success'] else 1)

    except KeyboardInterrupt:
        logger.warning("\nValidation interrupted by user")
        sys.exit(130)
    except Exception as e:
        logger.error(f"Validation failed with error: {e}")
        import traceback
        logger.debug(traceback.format_exc())
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Validate RAG retrieval pipeline quality",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python retrieve.py                    # Run all validation phases
  python retrieve.py --log-level DEBUG  # Enable debug logging
        """
    )

    parser.add_argument(
        "--log-level",
        default=LOG_LEVEL,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Set logging level (default: INFO)"
    )

    args = parser.parse_args()

    # Update logger level if provided
    if args.log_level != LOG_LEVEL:
        logger.remove()
        logger.add(
            lambda msg: print(msg, end=""),
            format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>",
            level=args.log_level
        )

    main()
