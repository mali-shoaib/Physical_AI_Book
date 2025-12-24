# Research: Embedding Pipeline Setup

**Feature**: 007-embedding-pipeline
**Date**: 2025-12-23
**Purpose**: Resolve technical unknowns and establish best practices for embedding pipeline implementation

---

## Decision 1: Python Environment & Package Management

**Decision**: Use **UV** for Python package management with Python 3.10+

**Rationale**:
- UV is a fast, modern Python package manager (written in Rust)
- Provides deterministic dependency resolution
- Compatible with pip and requirements.txt
- Significantly faster than pip for large dependency trees
- User explicitly requested UV in requirements

**Alternatives Considered**:
- **Poetry**: More mature, but slower and more complex for simple CLI tools
- **pip + venv**: Standard but lacks modern features and speed
- **Conda**: Overkill for non-data-science Python projects

**Implementation**: Initialize project with `uv init` and manage dependencies via `pyproject.toml`

---

## Decision 2: Cohere Embedding Model Selection

**Decision**: Use `embed-english-v3.0` with `embed-types=["search_document"]`

**Rationale**:
- Latest stable Cohere embedding model (1024 dimensions)
- Optimized for search/retrieval use cases
- Supports input type specification (search_document vs search_query)
- Constitutional requirement: Cohere-only embeddings with version pinning

**Alternatives Considered**:
- **embed-multilingual-v3.0**: Not needed (English-only per spec)
- **embed-english-light-v3.0**: Lower quality, not worth the cost savings

**Model Parameters**:
```python
cohere.embed(
    texts=[...],
    model="embed-english-v3.0",
    input_type="search_document",  # For ingestion
    embedding_types=["float"]
)
```

**Version Pinning**: Pin model version in code, log model version in Qdrant metadata

---

## Decision 3: Qdrant Client Configuration

**Decision**: Use Qdrant Cloud with `qdrant-client` Python SDK

**Rationale**:
- Constitution mandates Qdrant as vector store
- Cloud option provides managed infrastructure (no self-hosting overhead)
- Python client supports async operations and batch upserts
- Easy local development with Docker (qdrant/qdrant image)

**Collection Configuration**:
```python
{
    "collection_name": "rag_embedding",
    "vectors_config": {
        "size": 1024,  # Cohere embed-english-v3.0 dimension
        "distance": "Cosine"  # Standard for semantic similarity
    },
    "optimizers_config": {
        "indexing_threshold": 10000  # Balance indexing vs insert speed
    }
}
```

**Alternatives Considered**:
- **Self-hosted Qdrant**: More operational overhead, not needed for MVP
- **Other vector DBs**: Violates constitution (Qdrant-only)

---

## Decision 4: Web Scraping Strategy

**Decision**: Use `requests` + `BeautifulSoup4` with targeted CSS selectors

**Rationale**:
- Docusaurus has predictable HTML structure (`.markdown` class for content)
- No JavaScript rendering needed (static site)
- Lightweight and fast compared to Selenium/Playwright
- Sufficient for structured content extraction

**Extraction Strategy**:
1. Fetch sitemap.xml from Docusaurus deployment
2. Filter URLs (only `/docs/*` paths, exclude `/blog`, `/api`)
3. For each URL: extract `<article class="markdown">` content
4. Remove navigation (`<nav>`), footer, sidebar elements
5. Preserve code blocks (`<pre><code>`), headers (`<h1>-<h6>`), lists

**CSS Selectors** (Docusaurus-specific):
- Main content: `article.markdown` or `.theme-doc-markdown`
- Remove: `.navbar`, `.footer`, `.table-of-contents`, `.pagination-nav`

**Alternatives Considered**:
- **Playwright/Selenium**: Overkill for static content, slower
- **Scrapy**: Too heavy for single-site scraping
- **Trafilatura**: Generic article extraction, less control over structure

---

## Decision 5: Text Chunking Strategy

**Decision**: Recursive character splitting with semantic boundary detection

**Rationale**:
- Spec requires 512-1024 tokens with 10-20% overlap
- Need to preserve semantic structure (headers, paragraphs)
- Recursive splitting handles nested structure (sections → paragraphs → sentences)

**Algorithm**:
1. Split on markdown headers (`##`, `###`) first
2. For chunks > 1024 tokens, split on paragraph breaks (`\n\n`)
3. For chunks still > 1024, split on sentences
4. Add 10-20% overlap (last 100-200 tokens from previous chunk)

**Token Counting**: Use `tiktoken` with `cl100k_base` encoding (OpenAI standard, widely compatible)

**Implementation Library**: Custom implementation (no LangChain dependency)

**Alternatives Considered**:
- **LangChain RecursiveCharacterTextSplitter**: Too heavy, unnecessary dependency
- **Fixed-size chunking**: Ignores semantic boundaries (violates FR-005)
- **Sentence-based only**: May create too-small chunks

---

## Decision 6: Error Handling & Retry Logic

**Decision**: Exponential backoff with `tenacity` library

**Rationale**:
- Spec requires graceful handling of API rate limits (FR-011)
- Cohere and Qdrant APIs may have transient failures
- Exponential backoff prevents thundering herd

**Retry Configuration**:
```python
@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((RateLimitError, ConnectionError))
)
def embed_chunk(text):
    return cohere.embed(...)
```

**Logging**: Structured logging with `loguru` (simpler than stdlib logging)

**Alternatives Considered**:
- **Manual retry loops**: Error-prone, harder to maintain
- **backoff library**: Less features than tenacity

---

## Decision 7: Metadata Schema

**Decision**: Store metadata as Qdrant payload

**Schema**:
```python
{
    "chunk_id": str,  # UUID
    "source_url": str,  # Full Docusaurus URL
    "chapter_id": str,  # e.g., "module-1-ros2/ch1-basics"
    "module_name": str,  # e.g., "Module 1: ROS 2"
    "heading_hierarchy": List[str],  # ["Chapter 1", "Section 1.2", "Subsection 1.2.3"]
    "chunk_index": int,  # Position within chapter (0-based)
    "chunk_text": str,  # Original text (for debugging/display)
    "timestamp": str,  # ISO 8601 ingestion timestamp
    "model_version": str  # "embed-english-v3.0"
}
```

**Rationale**:
- Supports filtering by module/chapter during retrieval
- Enables incremental updates (match by source_url)
- Heading hierarchy preserves context for answer citation
- Chunk text stored for debugging and display to users

---

## Decision 8: Single-File Architecture (main.py)

**Decision**: Implement entire pipeline in `main.py` with functional decomposition

**Rationale**:
- User explicitly requested single-file design
- Simplifies deployment and debugging for MVP
- Functions are self-contained and testable

**Function Signature**s:
```python
def get_all_urls(base_url: str) -> List[str]:
    """Crawl sitemap and return all doc URLs"""

def extract_text_from_url(url: str) -> Tuple[str, Dict]:
    """Fetch and clean HTML, return text + metadata"""

def chunk_text(text: str, metadata: Dict) -> List[Tuple[str, Dict]]:
    """Split text into semantic chunks with overlap"""

def embed(texts: List[str]) -> List[List[float]]:
    """Generate Cohere embeddings for batch of texts"""

def create_collection(client: QdrantClient, name: str):
    """Initialize Qdrant collection with config"""

def save_chunk_to_qdrant(client: QdrantClient, chunk_text: str,
                         embedding: List[float], metadata: Dict):
    """Upsert single chunk with metadata"""

def main():
    """Orchestrate pipeline: crawl → extract → chunk → embed → store"""
```

**Alternatives Considered**:
- **Multi-file modules**: More scalable but violates user requirement
- **Class-based architecture**: Unnecessary state management

---

## Decision 9: Environment Configuration

**Decision**: Use `.env` file with `python-dotenv`

**Required Environment Variables**:
```bash
COHERE_API_KEY=...
QDRANT_URL=https://xyz.qdrant.io
QDRANT_API_KEY=...
DOCUSAURUS_BASE_URL=https://physical-ai-robotics-textbook-xi.vercel.app
CHUNK_SIZE=768  # tokens (default, configurable)
CHUNK_OVERLAP=150  # tokens (20% of 768)
SIMILARITY_THRESHOLD=0.70  # for validation queries
LOG_LEVEL=INFO
```

**Rationale**:
- Separates secrets from code
- Easy to override for different environments
- Standard Python practice

---

## Decision 10: Validation Strategy

**Decision**: Test queries with expected chunks (FR-014)

**Implementation**:
1. Define 10-20 test queries covering different modules
2. After ingestion, query Qdrant with each test query
3. Check: similarity score > 0.70, expected module/chapter present
4. Generate summary report: pass/fail per query, overall coverage

**Example Test Queries**:
```python
test_queries = [
    {"query": "What is ROS 2?", "expected_module": "module-1-ros2"},
    {"query": "How to launch Isaac Sim?", "expected_module": "module-3-isaac"},
    ...
]
```

**Alternatives Considered**:
- **Manual testing**: Not reproducible
- **Full integration test suite**: Out of scope for MVP (P3 priority)

---

## Dependencies

**Core Libraries** (pyproject.toml):
```toml
[project]
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "tiktoken>=0.5.0",
    "tenacity>=8.2.0",
    "loguru>=0.7.0",
    "python-dotenv>=1.0.0"
]
```

**Development Dependencies**:
```toml
[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "ruff>=0.1.0"  # Linting/formatting
]
```

---

## Deployment URL

**Production Docusaurus Site**: https://physical-ai-robotics-textbook-xi.vercel.app/

**Base URL for Ingestion**: `https://physical-ai-robotics-textbook-xi.vercel.app`

**Sitemap Location**: `https://physical-ai-robotics-textbook-xi.vercel.app/sitemap.xml` (standard Docusaurus)

---

## Performance Estimates

Based on spec success criteria:

- **Chunk Processing**: ~5 seconds/chunk (p95) → ~10 chapters/minute
- **Full Ingestion**: Assuming 100 chapters × 5 chunks/chapter = 500 chunks
  - Crawl: ~5 minutes (100 pages @ 3s/page)
  - Embed: ~40 minutes (500 chunks @ 5s/chunk)
  - Store: ~2 minutes (500 upserts @ 0.25s/upsert)
  - **Total**: ~50 minutes (within 30min target with batching optimizations)

**Optimization**: Batch embeddings (Cohere supports up to 96 texts/request) → reduce to ~15 minutes total

---

## Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|------------|
| Cohere rate limits | Slow ingestion | Exponential backoff, batch requests |
| Qdrant connection failures | Data loss | Checkpoint progress, retry with tenacity |
| Docusaurus structure changes | Extraction errors | CSS selector fallbacks, error logging |
| Large chapters exceed memory | OOM crash | Stream processing, chunk-by-chunk |
| Duplicate content in sidebars | Noise in embeddings | CSS selector filtering |

---

## Open Questions Resolved

All technical unknowns from spec have been researched and decisions made. No remaining [NEEDS CLARIFICATION] items.
