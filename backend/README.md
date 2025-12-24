# Embedding Pipeline - Physical AI Textbook RAG System

Complete embedding pipeline that crawls the Docusaurus textbook, generates Cohere embeddings, and stores them in Qdrant for RAG-based retrieval.

## Quick Start

### 1. Install UV (if not already installed)

```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# Windows (PowerShell)
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### 2. Install Dependencies

```bash
cd backend

# Install all dependencies with UV
uv pip install -e .

# Or with development dependencies
uv pip install -e ".[dev]"
```

### 3. Configure Environment

```bash
# Copy example env file
cp .env.example .env

# Edit .env and add your API keys
# COHERE_API_KEY=your_key_here
# QDRANT_API_KEY=your_key_here
# QDRANT_URL=https://your-cluster.qdrant.io
```

### 4. Run the Pipeline

```bash
python main.py
```

Expected output:
```
2025-12-23 14:30:00 | INFO     | Starting Embedding Pipeline
2025-12-23 14:30:05 | INFO     | Discovered 120 documentation URLs
2025-12-23 14:35:00 | INFO     | [1/120] Processing: https://...
...
2025-12-23 15:00:00 | INFO     | ✓ Pipeline completed successfully!
```

---

## Architecture

### Pipeline Flow

```
1. get_all_urls()           → Crawl sitemap.xml
2. extract_text_from_url()  → Fetch HTML, clean, extract text
3. chunk_text()             → Semantic chunking (512-1024 tokens)
4. embed()                  → Generate Cohere embeddings
5. create_collection()      → Initialize Qdrant collection
6. save_chunk_to_qdrant()   → Store vectors with metadata
7. main()                   → Orchestrate pipeline
```

### Functions

#### `get_all_urls(base_url: str) -> List[str]`
- Fetches sitemap.xml
- Filters `/docs/*` URLs
- Returns list of documentation pages

#### `extract_text_from_url(url: str) -> Tuple[str, Dict]`
- Fetches HTML content
- Removes navigation, footer, sidebar
- Extracts main content + metadata
- Returns (text, metadata)

#### `chunk_text(text: str, metadata: Dict) -> List[Tuple[str, Dict]]`
- Splits text on paragraph breaks
- Target: 512-1024 tokens per chunk
- Adds 10-20% overlap
- Returns list of (chunk_text, chunk_metadata)

#### `embed(texts: List[str]) -> List[List[float]]`
- Calls Cohere API in batches (max 96)
- Uses `embed-english-v3.0` model
- Input type: `search_document`
- Returns 1024-dimensional vectors

#### `create_collection(client, name="rag_embedding")`
- Initializes Qdrant collection
- Vector size: 1024
- Distance metric: Cosine
- Idempotent (skips if exists)

#### `save_chunk_to_qdrant(client, chunk_text, embedding, metadata)`
- Upserts point to Qdrant
- Stores embedding + full metadata
- Retry logic with exponential backoff

#### `main()`
- Validates environment variables
- Connects to Qdrant
- Processes all URLs sequentially
- Logs progress and statistics

---

## Configuration

### Environment Variables (.env)

| Variable | Required | Description |
|----------|----------|-------------|
| `COHERE_API_KEY` | ✅ Yes | Cohere API key for embeddings |
| `QDRANT_URL` | ✅ Yes | Qdrant instance URL |
| `QDRANT_API_KEY` | ✅ Yes | Qdrant API key |
| `DOCUSAURUS_BASE_URL` | No | Textbook URL (default: prod site) |
| `CHUNK_SIZE` | No | Target tokens per chunk (default: 768) |
| `CHUNK_OVERLAP` | No | Overlap tokens (default: 150) |
| `LOG_LEVEL` | No | Logging level (default: INFO) |

### Metadata Schema

Each chunk stores:
```python
{
    "chunk_id": str,              # UUID
    "source_url": str,            # Full URL
    "chapter_id": str,            # "module-1-ros2/ch1-basics"
    "module_name": str,           # "Module 1 Ros2"
    "heading_hierarchy": List[str], # ["Chapter 1", "Section 1.2"]
    "chunk_index": int,           # Position in chapter (0-based)
    "total_chunks": int,          # Total for this page
    "chunk_text": str,            # Full chunk text
    "token_count": int,           # Exact token count
    "timestamp": str,             # ISO 8601
    "model_version": str          # "embed-english-v3.0"
}
```

---

## Error Handling

### Retry Logic
- **HTTP requests**: 3 retries with exponential backoff (2s, 4s, 8s)
- **Embedding API**: 3 retries for rate limits/failures
- **Qdrant upserts**: 3 retries for connection issues

### Failure Modes
- **Empty content**: Skipped, logged as warning
- **API rate limits**: Automatic retry with backoff
- **Network errors**: Retry up to 3 times, then skip URL
- **Invalid chunks**: Logged and skipped

---

## Performance

### Expected Metrics
- **Full ingestion**: ~20-30 minutes for 120 pages
- **Embedding latency**: ~3-5 seconds per chunk
- **Throughput**: ~10-15 pages per minute
- **Success rate**: Target 95%+

### Optimization Tips

1. **Batch Embeddings**: Automatically batches up to 96 texts per API call
2. **Parallel Processing**: For faster ingestion, modify `main()` to use `ThreadPoolExecutor`
3. **Incremental Updates**: Track processed URLs to avoid re-ingesting unchanged content

---

## Troubleshooting

### Issue: "COHERE_API_KEY not set"
**Solution**: Create `.env` file and add your API key
```bash
cp .env.example .env
# Edit .env and add COHERE_API_KEY=your_key
```

### Issue: Qdrant connection timeout
**Solution**: Check Qdrant URL and API key
```bash
# Test connection
curl -H "api-key: YOUR_KEY" https://your-cluster.qdrant.io/collections
```

### Issue: Rate limit errors from Cohere
**Solution**: Reduce batch size in `embed()` function or add delays

### Issue: Empty text extraction
**Solution**: Verify Docusaurus site structure hasn't changed
```python
# Inspect page structure
soup = BeautifulSoup(html, 'lxml')
print(soup.find('article'))  # Should find main content
```

---

## Retrieval Validation

After running the embedding pipeline, validate RAG retrieval quality with the automated testing tool:

### Quick Start

```bash
cd backend
python retrieve.py
```

**Expected Output**:
```
2025-12-23 18:30:00 | INFO     | ================================================================================
2025-12-23 18:30:00 | INFO     | Retrieval Pipeline Validation
2025-12-23 18:30:00 | INFO     | ================================================================================

[Phase 1] Running Query Validation...
  [1/10] Query: "What is ROS 2?" - PASS (similarity=0.85)
  [2/10] Query: "How do I create a URDF file?" - PASS (similarity=0.78)
  ...
  Query Validation: 9/10 passed (90.0%)

[Phase 2] Running Coverage Analysis...
  Coverage: 13/13 URLs indexed (100.0%)

[Phase 3] Running Quality Metrics Analysis...
  Metadata Completeness: 100.0% (13/13 chunks)
  Token Distribution: min=450, max=1024, avg=768

================================================================================
VALIDATION REPORT
================================================================================
Query Validation:   9/10 passed (90.0%)
Coverage:           13/13 URLs (100.0%)
Metadata Complete:  100.0%
Avg Similarity:     0.78
Token Avg/Median:   768/780
================================================================================
[SUCCESS] All validation criteria passed!
  ✓ Query pass rate >= 90%
  ✓ Coverage = 100%
  ✓ Metadata completeness = 100%
```

### Configuration

Edit `.env` to customize validation:

```bash
# Retrieval Validation Settings
SIMILARITY_THRESHOLD=0.70      # Minimum similarity score
TOP_K_RESULTS=5                # Results per query
LOG_LEVEL=INFO                 # DEBUG for verbose output
```

### Test Queries

Test queries are hardcoded in `retrieve.py` under `TEST_QUERIES`. Customize by editing:

```python
TEST_QUERIES = [
    {
        "query_text": "What is ROS 2?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70,
        "description": "ROS 2 basics"
    },
    # Add more queries...
]
```

### Use Cases

**1. After Re-Ingestion**: Validate retrieval works correctly
```bash
python main.py      # Re-run ingestion
python retrieve.py  # Validate retrieval
```

**2. Debug Low Scores**: Enable detailed logging
```bash
# In .env: LOG_LEVEL=DEBUG
python retrieve.py
```

**3. CI/CD Integration**: Automated validation
```yaml
# .github/workflows/validate-rag.yml
- name: Run validation
  run: python backend/retrieve.py
```

### Success Criteria

- ✅ Query pass rate >= 90% (9/10 queries)
- ✅ Coverage = 100% (all sitemap URLs indexed)
- ✅ Metadata completeness = 100%
- ✅ Average similarity >= 0.70
- ✅ Runtime < 5 minutes

For detailed usage guide, see `specs/008-retrieval-pipeline-testing/quickstart.md`

---

## Manual Validation (Legacy)

### Test Qdrant Connection

```python
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('rag_embedding')
print(f'Collection has {info.points_count} chunks')
```

### Test Query

```python
import cohere
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))

# Embed test query
query = "What is ROS 2?"
query_embedding = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"
).embeddings[0]

# Search
results = qdrant.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=3
)

for result in results:
    print(f"Score: {result.score:.3f}")
    print(f"Chapter: {result.payload['chapter_id']}")
    print(f"Text: {result.payload['chunk_text'][:200]}...")
    print("---")
```

---

## Project Structure

```
backend/
├── main.py              # Embedding pipeline (ingestion)
├── retrieve.py          # Retrieval validation tool
├── pyproject.toml       # UV dependencies
├── .env                 # API keys (git-ignored)
├── .env.example         # Example configuration
├── .gitignore           # Git ignore rules
└── README.md            # This file
```

---

## Next Steps

1. **Run Initial Ingestion**: `python main.py`
2. **Verify Success**: Check Qdrant collection has chunks
3. **Test Retrieval**: Run validation queries
4. **Integrate with RAG Agent**: Connect to retrieval system
5. **Setup Incremental Updates**: Schedule periodic re-runs

---

## License

MIT License - See parent project LICENSE file

---

## Support

For issues or questions:
- Check `specs/007-embedding-pipeline/` documentation
- Review `quickstart.md` for detailed setup
- Check logs for error details

**Estimated Time**: 10 minutes setup + 30 minutes ingestion
