# Quickstart: Retrieval Pipeline Testing

**Feature**: 008-retrieval-pipeline-testing
**Script**: `backend/retrieve.py`
**Purpose**: Validate RAG retrieval quality by running test queries against Qdrant

---

## Prerequisites

1. **Completed Embedding Pipeline**: Feature 007 must be complete with Qdrant collection "rag_embedding" populated
2. **Python Environment**: Backend virtual environment with dependencies installed
3. **API Access**: Valid Cohere API key and Qdrant credentials in `.env`

**Verify Prerequisites**:
```bash
# Check Qdrant has data
# (Script will validate this automatically)

# Check .env file exists
ls backend/.env

# Check virtual environment exists
ls backend/.venv/
```

---

## Installation

**No additional installation needed** - all dependencies are already in `backend/pyproject.toml` from feature 007.

If you need to reinstall:
```bash
cd backend
uv sync  # or: pip install -r requirements.txt if not using UV
```

---

## Quick Start (5 Minutes)

### 1. Run Full Validation

```bash
cd backend
.venv/Scripts/python.exe retrieve.py
```

**Expected Output**:
```
2025-12-23 18:30:00 | INFO     | ================================================================================
2025-12-23 18:30:00 | INFO     | Starting Retrieval Pipeline Validation
2025-12-23 18:30:00 | INFO     | ================================================================================

2025-12-23 18:30:01 | INFO     | Phase 1: Query Validation
2025-12-23 18:30:01 | INFO     | Running 15 test queries...
2025-12-23 18:30:02 | INFO     | [1/15] Query: "What is ROS 2?" - PASS (similarity=0.85)
2025-12-23 18:30:03 | INFO     | [2/15] Query: "How do I create a URDF?" - PASS (similarity=0.78)
...
2025-12-23 18:30:20 | INFO     | Query Validation: 14/15 passed (93.3%)

2025-12-23 18:30:20 | INFO     | Phase 2: Coverage Analysis
2025-12-23 18:30:21 | INFO     | Fetching sitemap from https://physical-ai-robotics-textbook-xi.vercel.app/sitemap.xml
2025-12-23 18:30:22 | INFO     | Coverage: 13/13 URLs indexed (100.0%)

2025-12-23 18:30:22 | INFO     | Phase 3: Quality Metrics
2025-12-23 18:30:23 | INFO     | Analyzing 13 chunks...
2025-12-23 18:30:23 | INFO     | Metadata Completeness: 100.0% (13/13 chunks)
2025-12-23 18:30:23 | INFO     | Token Distribution: min=450, max=1024, avg=768

2025-12-23 18:30:23 | INFO     | ================================================================================
2025-12-23 18:30:23 | INFO     | Validation Report
2025-12-23 18:30:23 | INFO     | ================================================================================
2025-12-23 18:30:23 | INFO     | Total Queries:       15
2025-12-23 18:30:23 | INFO     | Passed:              14 (93.3%)
2025-12-23 18:30:23 | INFO     | Failed:              1
2025-12-23 18:30:23 | INFO     | Coverage:            100.0% (13/13 URLs)
2025-12-23 18:30:23 | INFO     | Metadata Complete:   100.0%
2025-12-23 18:30:23 | INFO     | Avg Similarity:      0.78
2025-12-23 18:30:23 | INFO     | Duration:            23.5 seconds
2025-12-23 18:30:23 | INFO     | ================================================================================
2025-12-23 18:30:23 | INFO     | [SUCCESS] Validation passed (>= 90% queries successful)
```

### 2. Interpret Results

**✅ SUCCESS Criteria**:
- Pass Rate >= 90% (14/15 = 93.3% ✅)
- Coverage = 100% (13/13 URLs ✅)
- Metadata Completeness = 100% ✅
- Average Similarity >= 0.70 (0.78 ✅)

**❌ FAILURE Example**:
```
2025-12-23 18:30:23 | ERROR    | [FAIL] Validation failed: only 85% queries passed (target: 90%)
2025-12-23 18:30:23 | ERROR    | Failed queries:
2025-12-23 18:30:23 | ERROR    |   - "What is Isaac Sim?" (similarity=0.62, below threshold 0.70)
2025-12-23 18:30:23 | ERROR    |   - "How do I install ROS 2?" (no results above threshold)
```

---

## Configuration

### Environment Variables

Edit `backend/.env`:

```bash
# Retrieval-Specific Settings
SIMILARITY_THRESHOLD=0.70      # Minimum score for "relevant" results
TOP_K_RESULTS=5                # Number of results per query
LOG_LEVEL=INFO                 # DEBUG for verbose output

# Shared with Ingestion Pipeline
COHERE_API_KEY=your-key-here
QDRANT_URL=https://your-qdrant-instance.com
QDRANT_API_KEY=your-qdrant-key
DOCUSAURUS_BASE_URL=https://physical-ai-robotics-textbook-xi.vercel.app
```

### Test Queries

Test queries are **hardcoded** in `backend/retrieve.py`:

```python
TEST_QUERIES = [
    {
        "query_text": "What is ROS 2?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70
    },
    {
        "query_text": "How do I create a URDF file?",
        "expected_module": "Module 1 Ros2",
        "min_similarity": 0.70
    },
    # Add more queries here to increase coverage
]
```

**To customize**:
1. Edit `TEST_QUERIES` list in `backend/retrieve.py`
2. Add queries covering topics you want to validate
3. Set `expected_module` based on Qdrant metadata (e.g., "Module 1 Ros2", "Module 3 Isaac")
4. Adjust `min_similarity` per query (default 0.70)

---

## Common Use Cases

### 1. Validate After Re-Ingestion

After running `backend/main.py` to re-ingest documentation:

```bash
# Re-ingest (feature 007)
cd backend
.venv/Scripts/python.exe main.py

# Validate retrieval (feature 008)
.venv/Scripts/python.exe retrieve.py
```

### 2. Test Specific Module

Edit `TEST_QUERIES` to focus on one module:

```python
TEST_QUERIES = [
    {"query_text": "What is ROS 2?", "expected_module": "Module 1 Ros2", "min_similarity": 0.70},
    {"query_text": "What is rclpy?", "expected_module": "Module 1 Ros2", "min_similarity": 0.70},
    {"query_text": "How to create URDF?", "expected_module": "Module 1 Ros2", "min_similarity": 0.70},
]
```

### 3. Debug Low Similarity Scores

Enable debug logging:

```bash
# In .env
LOG_LEVEL=DEBUG

# Run validation
.venv/Scripts/python.exe retrieve.py
```

**Debug output shows**:
- Exact similarity scores for each result
- Retrieved chunk text snippets
- Metadata for each chunk

### 4. Check Coverage Only

Modify `main()` to skip validation queries:

```python
# In retrieve.py main() function
# Comment out query validation:
# query_results = run_validation_queries(TEST_QUERIES)

# Run only coverage:
coverage_report = analyze_coverage(f"{DOCUSAURUS_BASE_URL}/sitemap.xml")
print(f"Coverage: {coverage_report['coverage_percentage']:.1f}%")
```

---

## Troubleshooting

### Issue: "Failed to connect to Qdrant"

**Cause**: Qdrant URL or API key incorrect

**Fix**:
```bash
# Verify .env file
cat backend/.env | grep QDRANT

# Test connection manually
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='YOUR_URL', api_key='YOUR_KEY'); print(client.get_collections())"
```

### Issue: "Collection 'rag_embedding' not found"

**Cause**: Ingestion pipeline not run yet

**Fix**:
```bash
# Run ingestion first
cd backend
.venv/Scripts/python.exe main.py
```

### Issue: "COHERE_API_KEY not set"

**Cause**: Missing or invalid .env file

**Fix**:
```bash
# Copy template
cp backend/.env.example backend/.env

# Edit with your keys
nano backend/.env  # or use any text editor
```

### Issue: "No results above threshold"

**Cause**: Similarity threshold too high or query mismatch

**Fix**:
1. Lower threshold in .env: `SIMILARITY_THRESHOLD=0.60`
2. Rephrase query to match documentation wording
3. Check if content actually exists in ingested chunks

### Issue: "Coverage < 100%"

**Cause**: Some URLs failed during ingestion

**Fix**:
```bash
# Re-run ingestion and check logs
cd backend
.venv/Scripts/python.exe main.py 2>&1 | grep "FAIL"

# Manually verify missing URLs
curl https://physical-ai-robotics-textbook-xi.vercel.app/docs/missing-chapter
```

---

## Advanced Usage

### Custom Similarity Threshold Per Query

Edit `TEST_QUERIES`:

```python
TEST_QUERIES = [
    {"query_text": "What is ROS 2?", "expected_module": "Module 1 Ros2", "min_similarity": 0.70},
    {"query_text": "Broad robotics question", "expected_module": "Module 1 Ros2", "min_similarity": 0.60},  # Lower threshold for broad query
]
```

### Batch Testing

Create multiple query sets:

```bash
# test-ros2.py
TEST_QUERIES = [ ... ROS 2 queries ... ]

# test-isaac.py
TEST_QUERIES = [ ... Isaac Sim queries ... ]
```

Run separately:
```bash
.venv/Scripts/python.exe test-ros2.py
.venv/Scripts/python.exe test-isaac.py
```

### CI/CD Integration

Add to GitHub Actions:

```yaml
# .github/workflows/validate-rag.yml
name: RAG Validation
on: [push]
jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup Python
        uses: actions/setup-python@v2
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: |
          cd backend
          pip install -r requirements.txt
      - name: Run validation
        env:
          COHERE_API_KEY: ${{ secrets.COHERE_API_KEY }}
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
        run: |
          cd backend
          python retrieve.py
```

---

## Next Steps

1. **Review Failed Queries**: If pass rate < 90%, investigate why specific queries failed
2. **Expand Test Suite**: Add more queries to `TEST_QUERIES` for broader coverage
3. **Tune Thresholds**: Adjust `SIMILARITY_THRESHOLD` based on quality/recall tradeoff
4. **Monitor Quality**: Run validation periodically after content updates
5. **Future Features**: This validation tool enables future RAG features (agent-based retrieval, chatbot)

---

## Success Checklist

- [ ] Validation script runs without errors
- [ ] Pass rate >= 90% (14/15 queries)
- [ ] Coverage = 100% (all sitemap URLs indexed)
- [ ] Metadata completeness = 100%
- [ ] Average similarity >= 0.70
- [ ] Runtime < 5 minutes

**If all checks pass**: ✅ RAG retrieval pipeline is validated and ready for production use!
