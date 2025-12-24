# Quickstart: Embedding Pipeline Setup

**Feature**: 007-embedding-pipeline
**Purpose**: Get the embedding pipeline running in under 10 minutes

---

## Prerequisites

- **Python**: 3.10 or higher
- **UV**: Fast Python package manager ([install guide](https://github.com/astral-sh/uv))
- **API Keys**:
  - Cohere API key ([get key](https://dashboard.cohere.com/api-keys))
  - Qdrant Cloud credentials ([get started](https://cloud.qdrant.io/))
- **Deployed Docusaurus Site**: https://physical-ai-robotics-textbook-xi.vercel.app/

---

## Quick Setup (5 minutes)

### 1. Create Project Directory

```bash
mkdir embedding-pipeline
cd embedding-pipeline
```

### 2. Initialize with UV

```bash
# Initialize UV project
uv init

# Create main.py placeholder
touch main.py

# Create .env file
touch .env
```

### 3. Configure Dependencies

Create `pyproject.toml`:

```toml
[project]
name = "embedding-pipeline"
version = "0.1.0"
description = "RAG embedding pipeline for Docusaurus textbook"
requires-python = ">=3.10"
dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.7.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "tiktoken>=0.5.0",
    "tenacity>=8.2.0",
    "loguru>=0.7.0",
    "python-dotenv>=1.0.0",
    "lxml>=4.9.0"  # BeautifulSoup HTML parser
]

[project.optional-dependencies]
dev = [
    "pytest>=7.4.0",
    "ruff>=0.1.0"
]
```

### 4. Install Dependencies

```bash
# Install all dependencies
uv pip install -e ".[dev]"

# Or without dev dependencies
uv pip install -e .
```

### 5. Configure Environment Variables

Edit `.env`:

```bash
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Docusaurus Configuration
DOCUSAURUS_BASE_URL=https://physical-ai-robotics-textbook-xi.vercel.app

# Pipeline Configuration
CHUNK_SIZE=768
CHUNK_OVERLAP=150
SIMILARITY_THRESHOLD=0.70
LOG_LEVEL=INFO
```

---

## Implementation (main.py)

Copy the complete implementation (see [Implementation Plan](plan.md) for full code).

**Key Functions**:
- `get_all_urls(base_url)` - Crawl sitemap
- `extract_text_from_url(url)` - Fetch and clean HTML
- `chunk_text(text, metadata)` - Semantic chunking
- `embed(texts)` - Generate Cohere embeddings
- `create_collection(client, name)` - Initialize Qdrant
- `save_chunk_to_qdrant(client, chunk_text, embedding, metadata)` - Store chunks
- `main()` - Orchestrate pipeline

---

## Running the Pipeline

### Full Ingestion (First Run)

```bash
# Run complete pipeline
python main.py

# Expected output:
# 2025-12-23 14:30:00 | INFO | Starting embedding pipeline...
# 2025-12-23 14:30:05 | INFO | Discovered 120 URLs from sitemap
# 2025-12-23 14:35:00 | INFO | Fetched 118/120 pages successfully
# 2025-12-23 14:40:00 | INFO | Created 850 chunks
# 2025-12-23 14:55:00 | INFO | Generated 845/850 embeddings
# 2025-12-23 14:57:00 | INFO | Stored 845 chunks in Qdrant
# 2025-12-23 14:57:10 | INFO | Validation: 48/50 test queries passed (96%)
# 2025-12-23 14:57:10 | INFO | Pipeline completed in 27 minutes
```

### Incremental Update (Subsequent Runs)

```bash
# Re-run to update only changed content
python main.py --incremental

# Or set environment variable
INCREMENTAL_UPDATE=true python main.py
```

---

## Validation

### Quick Test

```bash
# Test Qdrant connection and query
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

# Check collection
info = client.get_collection('rag_embedding')
print(f'Collection has {info.points_count} chunks')
"
```

### Test Query

```bash
# Query for specific content
python -c "
import cohere
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

# Embed test query
query = 'What is ROS 2?'
query_embedding = co.embed(
    texts=[query],
    model='embed-english-v3.0',
    input_type='search_query'
).embeddings[0]

# Search Qdrant
results = qdrant.search(
    collection_name='rag_embedding',
    query_vector=query_embedding,
    limit=3
)

for result in results:
    print(f'Score: {result.score:.3f}')
    print(f'Chapter: {result.payload[\"chapter_id\"]}')
    print(f'Text: {result.payload[\"chunk_text\"][:200]}...')
    print('---')
"
```

---

## Troubleshooting

### Issue: Cohere Rate Limit Errors

**Solution**: Reduce batch size or add delays

```python
# In embed() function, reduce batch size
MAX_BATCH_SIZE = 50  # Instead of 96

# Or add delay between batches
import time
time.sleep(1)  # 1 second between batches
```

### Issue: Qdrant Connection Timeout

**Solution**: Check network and API key

```bash
# Test connection
curl -H "api-key: YOUR_API_KEY" https://your-cluster.qdrant.io/collections
```

### Issue: Empty Text Extraction

**Solution**: Verify CSS selectors match Docusaurus structure

```python
# Inspect page structure
import requests
from bs4 import BeautifulSoup

url = "https://physical-ai-robotics-textbook-xi.vercel.app/docs/intro"
html = requests.get(url).text
soup = BeautifulSoup(html, 'lxml')

# Check available selectors
print(soup.find('article'))  # Should find main content
print(soup.find_all('h1', 'h2', 'h3'))  # Should find headers
```

### Issue: Token Count Violations

**Solution**: Adjust chunk size parameters

```bash
# In .env, tune chunk sizes
CHUNK_SIZE=512  # Smaller chunks
CHUNK_OVERLAP=100  # 20% of 512
```

---

## Performance Optimization

### Parallel Processing

```python
# Use concurrent.futures for URL fetching
from concurrent.futures import ThreadPoolExecutor

def fetch_all_pages(urls):
    with ThreadPoolExecutor(max_workers=10) as executor:
        results = executor.map(extract_text_from_url, urls)
    return list(results)
```

### Batch Embedding

```python
# Maximize batch size (up to 96 texts per API call)
def embed_chunks_batch(chunks, batch_size=96):
    embeddings = []
    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        batch_embeddings = embed([c[0] for c in batch])
        embeddings.extend(batch_embeddings)
    return embeddings
```

### Batch Qdrant Upsert

```python
# Use batch upsert instead of individual saves
def batch_upsert_to_qdrant(client, chunks, embeddings, metadata_list):
    points = [
        {
            "id": str(uuid.uuid4()),
            "vector": emb,
            "payload": meta
        }
        for emb, meta in zip(embeddings, metadata_list)
    ]

    client.upsert(
        collection_name="rag_embedding",
        points=points
    )
```

---

## Project Structure

```
embedding-pipeline/
├── main.py                  # Complete pipeline implementation
├── pyproject.toml           # UV dependencies
├── .env                     # Environment variables (git-ignored)
├── .gitignore               # Ignore .env, __pycache__, etc.
├── README.md                # Project documentation
└── logs/                    # Pipeline execution logs (optional)
```

---

## Next Steps

1. **Run Initial Ingestion**: Execute `python main.py` and verify 95%+ success rate
2. **Test Retrieval**: Run validation queries to check embedding quality
3. **Monitor Performance**: Log execution times and optimize bottlenecks
4. **Setup Incremental Updates**: Schedule periodic re-ingestion for content changes
5. **Integrate with RAG Agent**: Connect Qdrant to retrieval-augmented generation system

---

## Expected Results

**Successful Run**:
- ✅ 100+ URLs discovered
- ✅ 95%+ pages fetched successfully
- ✅ 800-1000 chunks created (for full textbook)
- ✅ 95%+ embeddings generated
- ✅ All chunks stored in Qdrant
- ✅ Validation queries pass with >0.70 similarity
- ✅ Total runtime < 30 minutes

**Summary Report**:
```json
{
  "job_id": "uuid-here",
  "status": "completed",
  "duration_seconds": 1620,
  "stats": {
    "pages_discovered": 120,
    "pages_fetched_success": 118,
    "chunks_created": 850,
    "embeddings_generated": 845,
    "qdrant_upserts_success": 845
  },
  "success_rate": 99.4
}
```

---

## Support

- **Issues**: Check [troubleshooting](#troubleshooting) section
- **Logs**: Review `logs/pipeline-YYYY-MM-DD.log` for detailed errors
- **Validation**: Run test queries to verify embedding quality

**Estimated Time to First Run**: 10 minutes (setup) + 30 minutes (pipeline execution)
