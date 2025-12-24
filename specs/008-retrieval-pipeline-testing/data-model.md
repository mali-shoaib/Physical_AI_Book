# Data Model: Retrieval Pipeline Testing

**Feature**: 008-retrieval-pipeline-testing
**Date**: 2025-12-23
**Phase**: 1 (Design)

## Overview

This document defines the data structures used by the retrieval validation tool. Since this is a read-only validation tool (not a persistent system), the "data model" consists of:
1. **Input structures**: Test queries and configuration
2. **Retrieved structures**: Data read from Qdrant
3. **Output structures**: Validation reports and metrics

No database schema needed - all data is ephemeral (in-memory during validation run).

---

## Core Entities

### 1. TestQuery

**Purpose**: Represents a single validation query with expected results

**Attributes**:
| Field | Type | Required | Description | Example |
|-------|------|----------|-------------|---------|
| query_text | str | Yes | The query string to search for | "What is ROS 2?" |
| expected_module | str | Yes | Expected module name in top results | "Module 1 Ros2" |
| expected_chapter | str | No | Optional specific chapter ID | "ch1-ros2-basics" |
| min_similarity | float | Yes | Minimum similarity score for pass | 0.70 |
| description | str | No | Human-readable test description | "Validates ROS 2 basics retrieval" |

**Validation Rules**:
- query_text must not be empty
- min_similarity must be between 0.0 and 1.0
- expected_module should match format "Module N Name" from Qdrant metadata

**Python Representation**:
```python
{
    "query_text": "What is ROS 2?",
    "expected_module": "Module 1 Ros2",
    "expected_chapter": "ch1-ros2-basics",  # Optional
    "min_similarity": 0.70,
    "description": "Validates ROS 2 basics retrieval"
}
```

---

### 2. QueryResult

**Purpose**: Represents a single retrieved chunk from Qdrant

**Attributes**:
| Field | Type | Required | Description | Source |
|-------|------|----------|-------------|--------|
| chunk_id | str | Yes | Unique chunk identifier (UUID) | Qdrant point ID |
| chunk_text | str | Yes | The actual text content | Qdrant payload.chunk_text |
| similarity_score | float | Yes | Cosine similarity (0.0-1.0) | Qdrant search score |
| source_url | str | Yes | Origin URL of content | Qdrant payload.source_url |
| chapter_id | str | Yes | Chapter identifier | Qdrant payload.chapter_id |
| module_name | str | Yes | Module name | Qdrant payload.module_name |
| heading_hierarchy | list[str] | Yes | List of headings | Qdrant payload.heading_hierarchy |
| token_count | int | Yes | Number of tokens in chunk | Qdrant payload.token_count |
| chunk_index | int | Yes | Position in original document | Qdrant payload.chunk_index |

**Validation Rules**:
- similarity_score must be between 0.0 and 1.0
- source_url must be valid URL format
- token_count must be positive integer

**Python Representation**:
```python
{
    "chunk_id": "uuid-string",
    "chunk_text": "ROS 2 is a robotics middleware...",
    "similarity_score": 0.85,
    "source_url": "https://example.com/docs/module-1-ros2/ch1-ros2-basics",
    "chapter_id": "module-1-ros2/ch1-ros2-basics",
    "module_name": "Module 1 Ros2",
    "heading_hierarchy": ["ROS 2 Basics", "Introduction"],
    "token_count": 512,
    "chunk_index": 0
}
```

---

### 3. QueryValidationResult

**Purpose**: Result of validating a single test query

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | TestQuery | Yes | The original test query |
| top_results | list[QueryResult] | Yes | Top N retrieved chunks (default 5) |
| passed | bool | Yes | True if validation passed |
| pass_reason | str | No | Why it passed (if passed=True) |
| fail_reason | str | No | Why it failed (if passed=False) |
| best_similarity | float | Yes | Highest similarity score in results |
| module_match | bool | Yes | True if expected_module found in top results |

**Validation Logic**:
```python
passed = (
    best_similarity >= query.min_similarity and
    module_match == True
)
```

**Python Representation**:
```python
{
    "query": { ... TestQuery object ... },
    "top_results": [ ... list of QueryResult ... ],
    "passed": True,
    "pass_reason": "Found relevant chunk (similarity=0.85) in expected module 'Module 1 Ros2'",
    "fail_reason": None,
    "best_similarity": 0.85,
    "module_match": True
}
```

---

### 4. CoverageReport

**Purpose**: Analysis of indexed content coverage

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| total_sitemap_urls | int | Yes | Count of URLs in sitemap.xml |
| indexed_urls | list[str] | Yes | URLs found in Qdrant |
| missing_urls | list[str] | Yes | URLs in sitemap but not Qdrant |
| extra_urls | list[str] | Yes | URLs in Qdrant but not sitemap (orphans) |
| coverage_percentage | float | Yes | (indexed / total) * 100 |
| timestamp | str | Yes | When analysis was run (ISO 8601) |

**Calculation**:
```python
coverage_percentage = (len(indexed_urls) / total_sitemap_urls) * 100
```

**Python Representation**:
```python
{
    "total_sitemap_urls": 13,
    "indexed_urls": [
        "https://example.com/docs/intro",
        "https://example.com/docs/module-1-ros2/ch1-ros2-basics",
        # ... 11 more
    ],
    "missing_urls": [],  # Empty if 100% coverage
    "extra_urls": [],  # Empty if no orphans
    "coverage_percentage": 100.0,
    "timestamp": "2025-12-23T18:30:00Z"
}
```

---

### 5. QualityMetrics

**Purpose**: Statistical analysis of embedding quality

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| total_chunks | int | Yes | Total chunks in Qdrant |
| chunks_with_complete_metadata | int | Yes | Chunks with all required fields |
| metadata_completeness_rate | float | Yes | (complete / total) * 100 |
| token_distribution | dict | Yes | Min/max/avg/median token counts |
| avg_similarity_score | float | Yes | Average score across all test queries |
| similarity_distribution | dict | Yes | Min/max/avg/median similarity scores |
| timestamp | str | Yes | When analysis was run (ISO 8601) |

**Required Metadata Fields**:
- source_url
- chapter_id
- module_name
- heading_hierarchy
- token_count
- chunk_index

**Python Representation**:
```python
{
    "total_chunks": 13,
    "chunks_with_complete_metadata": 13,
    "metadata_completeness_rate": 100.0,
    "token_distribution": {
        "min": 450,
        "max": 1024,
        "avg": 768,
        "median": 750
    },
    "avg_similarity_score": 0.78,
    "similarity_distribution": {
        "min": 0.65,
        "max": 0.92,
        "avg": 0.78,
        "median": 0.77
    },
    "timestamp": "2025-12-23T18:30:00Z"
}
```

---

### 6. ValidationReport

**Purpose**: Complete validation summary across all tests

**Attributes**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| total_queries | int | Yes | Number of test queries run |
| passed_queries | int | Yes | Number that passed |
| failed_queries | int | Yes | Number that failed |
| pass_rate | float | Yes | (passed / total) * 100 |
| query_results | list[QueryValidationResult] | Yes | Individual query results |
| coverage_report | CoverageReport | Yes | Coverage analysis |
| quality_metrics | QualityMetrics | Yes | Quality statistics |
| duration_seconds | float | Yes | Total validation runtime |
| timestamp | str | Yes | When validation ran (ISO 8601) |

**Success Criteria**:
- pass_rate >= 90.0 (from SC-002 in spec)
- coverage_report.coverage_percentage == 100.0 (from SC-003)
- quality_metrics.metadata_completeness_rate == 100.0 (from SC-005)

**Python Representation**:
```python
{
    "total_queries": 15,
    "passed_queries": 14,
    "failed_queries": 1,
    "pass_rate": 93.3,
    "query_results": [ ... list of QueryValidationResult ... ],
    "coverage_report": { ... CoverageReport object ... },
    "quality_metrics": { ... QualityMetrics object ... },
    "duration_seconds": 42.5,
    "timestamp": "2025-12-23T18:30:00Z"
}
```

---

## Data Flow

### 1. Query Validation Flow

```
TestQuery
    ↓
[Embed query with Cohere]
    ↓
[Search Qdrant with embedding]
    ↓
List[QueryResult]
    ↓
[Validate against expected_module and min_similarity]
    ↓
QueryValidationResult
```

### 2. Coverage Analysis Flow

```
[Fetch sitemap.xml]
    ↓
List[sitemap_urls]
    ↓
[Query Qdrant for distinct source_url values]
    ↓
List[indexed_urls]
    ↓
[Set operations: missing, extra, coverage %]
    ↓
CoverageReport
```

### 3. Quality Metrics Flow

```
[Scroll through all Qdrant points]
    ↓
List[all_chunks_metadata]
    ↓
[Analyze token_count distribution]
[Analyze metadata completeness]
[Aggregate similarity scores from query results]
    ↓
QualityMetrics
```

### 4. Overall Validation Flow

```
List[TestQuery] (hardcoded in retrieve.py)
    ↓
[Run query validation for each]
    ↓
List[QueryValidationResult]
    ↓
[Run coverage analysis]
    ↓
CoverageReport
    ↓
[Run quality analysis]
    ↓
QualityMetrics
    ↓
[Aggregate into final report]
    ↓
ValidationReport
    ↓
[Print to console with loguru]
```

---

## Environment Configuration

**Source**: backend/.env (shared with ingestion pipeline)

| Variable | Type | Required | Default | Description |
|----------|------|----------|---------|-------------|
| COHERE_API_KEY | str | Yes | - | Cohere API key for query embeddings |
| QDRANT_URL | str | Yes | - | Qdrant instance URL |
| QDRANT_API_KEY | str | Yes | - | Qdrant API key |
| DOCUSAURUS_BASE_URL | str | Yes | - | Docusaurus site URL for sitemap |
| SIMILARITY_THRESHOLD | float | No | 0.70 | Minimum similarity score for "relevant" |
| TOP_K_RESULTS | int | No | 5 | Number of results to retrieve per query |
| LOG_LEVEL | str | No | INFO | Logging level (DEBUG/INFO/WARNING/ERROR) |

---

## Relationships

```
TestQuery (1) ----runs----> (1) QueryValidationResult
                               ↓
                          contains
                               ↓
                            (0..N) QueryResult

ValidationReport (1) ----contains----> (N) QueryValidationResult
                 (1) ----contains----> (1) CoverageReport
                 (1) ----contains----> (1) QualityMetrics
```

---

## Implementation Notes

1. **No Persistence**: All entities exist only in memory during script execution
2. **No ORM**: Use dictionaries/dataclasses for simplicity (no SQLAlchemy needed)
3. **JSON Serializable**: All entities can be easily converted to JSON for future extensions
4. **Immutability**: Entities are created once and not modified (functional style)

---

## Future Extensions (Out of Scope for MVP)

- **HistoricalValidation**: Store ValidationReport history in database for trend analysis
- **QueryTemplate**: Support parameterized queries for broader testing
- **ThresholdConfig**: Per-query custom thresholds instead of global default
- **ExportFormats**: JSON/CSV export options for ValidationReport
