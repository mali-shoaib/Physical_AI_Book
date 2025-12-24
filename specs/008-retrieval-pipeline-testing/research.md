# Research: Retrieval Pipeline Testing

**Feature**: 008-retrieval-pipeline-testing
**Date**: 2025-12-23
**Phase**: 0 (Research & Design Decisions)

## Overview

This document captures research findings and design decisions for the retrieval validation tool. Since this feature builds on the existing embedding pipeline (feature 007), most technical decisions are already made. Research focuses on validation strategies and query design patterns.

## Research Questions & Decisions

### 1. Query Embedding Parameters

**Question**: Should query embeddings use different parameters than document embeddings?

**Decision**: Use `input_type="search_query"` for queries vs. `input_type="search_document"` for documents

**Rationale**:
- Cohere's embed-english-v3.0 model is optimized for asymmetric search (query vs. document)
- `search_query` optimizes for short query text, `search_document` for longer passages
- This is Cohere's recommended best practice for RAG systems
- Reference: Cohere Embed API documentation (https://docs.cohere.com/docs/embeddings)

**Alternatives Considered**:
- Using same `input_type="search_document"` for both → Rejected: Not optimized for short queries
- Using separate models for query/document → Rejected: Unnecessary complexity, same model works

**Implementation**:
```python
# Query embedding
co.embed(
    texts=[query_text],
    model="embed-english-v3.0",
    input_type="search_query"  # Different from ingestion
)
```

---

### 2. Similarity Threshold Selection

**Question**: What similarity score threshold should be used to determine "relevant" results?

**Decision**: Default threshold of 0.70, configurable via environment variable

**Rationale**:
- 0.70 is industry standard for semantic search systems
- Cohere cosine similarity scores typically range 0.0-1.0
- Threshold of 0.70 balances precision (avoiding false positives) and recall (not missing relevant chunks)
- Empirical testing from similar RAG systems shows 0.70 works well
- Making it configurable allows tuning based on actual retrieval quality

**Alternatives Considered**:
- Fixed threshold 0.80 → Rejected: Too strict, may miss relevant results
- Fixed threshold 0.60 → Rejected: Too loose, may return irrelevant results
- Adaptive threshold → Rejected: Over-engineering for validation tool

**Implementation**:
```python
SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.70"))
```

---

### 3. Test Query Design

**Question**: How should validation test queries be structured and selected?

**Decision**: Manually curated list of 10-15 queries covering major modules, stored as Python list in code

**Rationale**:
- Manual curation ensures coverage of all major topics (ROS 2, URDF, Isaac Sim, etc.)
- Embedding in code (not external file) keeps tool self-contained
- 10-15 queries sufficient for validation without being exhaustive
- Each query includes expected_module for automated validation

**Alternatives Considered**:
- Load queries from JSON/YAML file → Rejected: Adds file dependency, over-engineering
- Generate queries automatically → Rejected: No way to ensure quality coverage
- Use all chapter titles as queries → Rejected: Not realistic user queries

**Implementation**:
```python
TEST_QUERIES = [
    {
        "query": "What is ROS 2?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70
    },
    {
        "query": "How do I create a URDF file?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70
    },
    # ... more queries
]
```

---

### 4. Qdrant Query Strategy

**Question**: What Qdrant search parameters should be used for retrieval?

**Decision**: Standard similarity search with limit=5, no filters

**Rationale**:
- limit=5 returns top 5 most similar chunks (typical RAG configuration)
- No metadata filters needed for validation (want to see what system returns naturally)
- Cosine distance metric already configured in collection (from feature 007)
- Simple search is sufficient for validation purposes

**Alternatives Considered**:
- Using filters (e.g., filter by module) → Rejected: Want to test unfiltered retrieval
- limit=1 → Rejected: Need multiple results to verify ranking
- limit=10 → Rejected: Unnecessary, top 5 sufficient for validation

**Implementation**:
```python
results = qdrant_client.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=5
)
```

---

### 5. Coverage Analysis Methodology

**Question**: How to compare Qdrant contents against sitemap to identify gaps?

**Decision**:
1. Fetch sitemap.xml from Docusaurus site
2. Query Qdrant for all distinct source_url values
3. Set difference to find missing URLs
4. Report coverage percentage and missing list

**Rationale**:
- Sitemap is source of truth for published content
- Qdrant metadata includes source_url for every chunk
- Set operations provide simple, accurate comparison
- Coverage percentage gives clear quality metric

**Alternatives Considered**:
- Query each sitemap URL individually → Rejected: Inefficient, O(n) queries
- Use Qdrant scroll API to iterate all points → Rejected: Slower than metadata query
- Manual comparison → Rejected: Not automated

**Implementation**:
```python
# Get all URLs from sitemap
sitemap_urls = set(get_all_urls(DOCUSAURUS_BASE_URL))

# Get all URLs in Qdrant (distinct source_url values)
# Requires scrolling through collection and extracting unique source_url from metadata
qdrant_urls = set()
# ... scroll through collection ...

# Coverage analysis
missing_urls = sitemap_urls - qdrant_urls
coverage_pct = (len(qdrant_urls) / len(sitemap_urls)) * 100
```

---

### 6. Reporting Format

**Question**: How should validation results be presented to developers?

**Decision**: Command-line output with structured logging (loguru) and final summary report

**Rationale**:
- Developers run from terminal, need immediate readable output
- Structured logging matches existing backend/main.py style
- Summary report provides at-a-glance validation status
- Color-coded pass/fail makes results scannable

**Alternatives Considered**:
- JSON output → Rejected: Less human-readable for manual validation
- HTML report → Rejected: Over-engineering for command-line tool
- Database storage → Rejected: No need for historical tracking in MVP

**Implementation**:
```python
logger.info("=" * 80)
logger.info("Validation Results")
logger.info("=" * 80)
logger.info(f"Total Queries: {total}")
logger.info(f"Passed: {passed} ({pass_rate:.1f}%)")
logger.info(f"Failed: {failed}")
logger.info(f"Coverage: {coverage_pct:.1f}% ({len(qdrant_urls)}/{len(sitemap_urls)} URLs)")
```

---

### 7. Error Handling Strategy

**Question**: How to handle failures (Qdrant down, Cohere API errors, etc.)?

**Decision**: Graceful degradation with clear error messages, no retries

**Rationale**:
- This is a validation tool, not production system - failures should be visible
- Clear error messages help developers diagnose issues
- No retry logic needed - developer can re-run manually
- Connection errors should fail fast with actionable message

**Alternatives Considered**:
- Retry with exponential backoff → Rejected: Validation tool doesn't need resilience
- Silent failure → Rejected: Developer needs to know what failed
- Fallback to partial results → Rejected: Partial validation is misleading

**Implementation**:
```python
try:
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
except Exception as e:
    logger.error(f"Failed to connect to Qdrant: {e}")
    logger.error("Check QDRANT_URL and QDRANT_API_KEY in .env file")
    sys.exit(1)
```

---

## Technology Stack Confirmation

| Component | Choice | Justification |
|-----------|--------|---------------|
| Language | Python 3.10+ | Matches existing backend, all dependencies available |
| Embedding | Cohere embed-english-v3.0 | Same model as ingestion pipeline |
| Vector DB | Qdrant | Existing collection "rag_embedding" |
| Logging | loguru | Matches backend/main.py style |
| Config | python-dotenv | Existing .env setup |

**No new dependencies required** - All packages already in backend/pyproject.toml from feature 007.

---

## Best Practices Applied

1. **Single Responsibility**: retrieve.py focuses only on validation, not ingestion
2. **Configuration Reuse**: Shares .env with main.py, no duplicate config
3. **Logging Consistency**: Uses same loguru style as ingestion pipeline
4. **Error Messages**: Clear, actionable error messages for developers
5. **Testability**: Each user story independently testable (query validation, coverage, quality metrics)

---

## Open Questions & Future Enhancements

**Resolved - No open questions remain for MVP implementation**

**Future Enhancements** (Out of scope for this feature):
- Historical tracking of validation results over time
- Automated regression testing (CI/CD integration)
- Query performance benchmarking (latency distribution)
- Visual dashboard for quality metrics (web UI)

---

## Research Summary

All design decisions finalized. No technical unknowns remaining. Ready for Phase 1 (data model and contracts definition).
