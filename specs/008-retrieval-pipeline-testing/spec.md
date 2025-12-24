# Feature Specification: Retrieval Pipeline Testing

**Feature Branch**: `008-retrieval-pipeline-testing`
**Created**: 2025-12-23
**Status**: Draft
**Input**: User description: "Retrieval Pipeline Testing - Goal: Retrieve stored embeddings from **Qdrant**, run similarity queries, and confirm the end-to-end extraction + embedding + vector storage pipeline works correctly. Target: Developers validating backend RAG retrieval flow."

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Basic Similarity Search Validation (Priority: P1)

As a developer validating the RAG pipeline, I need to run test queries against the Qdrant vector database and verify that relevant document chunks are retrieved with appropriate similarity scores, so that I can confirm the end-to-end ingestion pipeline (crawl → extract → chunk → embed → store) worked correctly.

**Why this priority**: This is the core validation test that proves the entire embedding pipeline functions correctly. Without this, we cannot trust that our RAG system will return relevant results to user queries.

**Independent Test**: Can be fully tested by running a predefined set of test queries (e.g., "What is ROS 2?", "How do I create a URDF?") against Qdrant and verifying that returned chunks match expected chapters/modules with similarity scores above threshold (0.70). Delivers immediate validation that embeddings were stored correctly and retrieval works.

**Acceptance Scenarios**:

1. **Given** the embedding pipeline has ingested documentation into Qdrant, **When** a developer runs a test query "What is ROS 2?", **Then** the system returns chunks from the ROS 2 module with similarity scores above 0.70
2. **Given** the Qdrant collection "rag_embedding" contains embeddings, **When** a developer runs a query on a specific technical topic, **Then** the returned chunks contain relevant content matching that topic
3. **Given** multiple chapters exist in different modules, **When** a developer queries a topic covered in multiple places, **Then** results are ranked by similarity score with the most relevant chunks appearing first
4. **Given** test queries covering all major modules, **When** developer runs the full test suite, **Then** at least 90% of queries return relevant results above the similarity threshold

---

### User Story 2 - Coverage and Completeness Validation (Priority: P2)

As a developer validating the RAG pipeline, I need to verify that all documentation chapters were successfully ingested and are searchable in Qdrant, so that I can identify any gaps or missing content in the vector database.

**Why this priority**: After confirming basic retrieval works (P1), we need to ensure comprehensive coverage. Missing chapters would create knowledge gaps in the RAG system. This validates data completeness but doesn't affect core retrieval functionality.

**Independent Test**: Can be tested independently by querying Qdrant for all unique source URLs/chapter IDs and comparing against the sitemap.xml from the Docusaurus site. Delivers a coverage report showing which chapters are indexed and which are missing.

**Acceptance Scenarios**:

1. **Given** the Docusaurus sitemap contains 13 documentation URLs, **When** developer queries Qdrant for distinct source URLs, **Then** all 13 URLs are represented in the vector database
2. **Given** the ingestion pipeline processed multiple modules, **When** developer runs coverage analysis, **Then** a report shows which modules/chapters have embeddings and which are missing
3. **Given** some chapters may have failed during ingestion, **When** developer compares Qdrant contents to sitemap, **Then** any missing chapters are clearly identified with their URLs

---

### User Story 3 - Embedding Quality Metrics (Priority: P3)

As a developer validating the RAG pipeline, I need to measure embedding quality metrics (average similarity scores, chunk distribution, metadata completeness), so that I can assess the overall health and quality of the vector database.

**Why this priority**: While P1 and P2 validate that retrieval works and content is complete, this story provides deeper insights into embedding quality and can help optimize chunking strategies. It's valuable but not critical for basic validation.

**Independent Test**: Can be tested by running statistical analysis on stored embeddings - checking distribution of chunk sizes, verifying all required metadata fields are populated, and measuring average similarity scores across test queries. Delivers a quality dashboard/report.

**Acceptance Scenarios**:

1. **Given** chunks are stored in Qdrant with metadata, **When** developer runs quality analysis, **Then** 100% of chunks have required metadata fields (source_url, chapter_id, module_name, heading_hierarchy)
2. **Given** embeddings were created with specific chunking parameters (512-1024 tokens), **When** developer analyzes chunk distribution, **Then** token counts fall within the configured range with minimal outliers
3. **Given** test queries with known expected results, **When** developer measures average similarity scores, **Then** relevant results consistently score above 0.70 and irrelevant results score below 0.60

---

### Edge Cases

- What happens when querying for content that doesn't exist in the documentation (e.g., "quantum computing")? System should return low similarity scores or no results above threshold
- How does the system handle very broad queries (e.g., "robotics") that could match many chapters? Should return multiple results ranked by relevance
- What happens if Qdrant collection is empty or doesn't exist? System should return clear error message indicating no data available
- How does system perform with very long queries (e.g., full paragraphs)? Should still generate embeddings and return relevant results
- What happens when similarity threshold is set too high (e.g., 0.95)? May return no results; validation should test various thresholds

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a query function that accepts text input and returns similar document chunks from Qdrant
- **FR-002**: System MUST generate query embeddings using the same Cohere model (embed-english-v3.0) and parameters used for document ingestion
- **FR-003**: System MUST return similarity scores with each retrieved chunk to enable quality assessment
- **FR-004**: System MUST support configurable similarity threshold filtering (e.g., only return results above 0.70)
- **FR-005**: System MUST support configurable result limit (e.g., top N results)
- **FR-006**: System MUST provide a validation test suite with predefined queries covering all major documentation modules
- **FR-007**: System MUST generate a validation report showing pass/fail status for each test query
- **FR-008**: System MUST provide coverage analysis showing which chapters/URLs are indexed in Qdrant
- **FR-009**: System MUST compare Qdrant contents against the source sitemap to identify missing content
- **FR-010**: System MUST validate metadata completeness (all required fields present in all chunks)
- **FR-011**: System MUST provide statistical analysis of chunk sizes and token distributions
- **FR-012**: System MUST calculate and report average similarity scores across test queries
- **FR-013**: System MUST support batch querying for running multiple test queries efficiently
- **FR-014**: System MUST log all query operations with timestamps and results for debugging
- **FR-015**: System MUST handle Qdrant connection errors gracefully with clear error messages

### Key Entities

- **Test Query**: Represents a validation query with expected module/chapter and minimum similarity threshold (e.g., query_text: "What is ROS 2?", expected_module: "Module 1 Ros2", min_similarity: 0.70)
- **Query Result**: Retrieved chunk with similarity score and metadata (chunk_text, similarity_score, source_url, chapter_id, module_name)
- **Coverage Report**: Analysis showing indexed vs. missing chapters (total_urls_in_sitemap, urls_in_qdrant, missing_urls, coverage_percentage)
- **Quality Metrics**: Statistical analysis of embeddings (total_chunks, avg_chunk_size, metadata_completeness_rate, avg_similarity_score)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Developers can validate RAG pipeline health in under 5 minutes by running the test suite
- **SC-002**: At least 90% of predefined test queries return relevant results above configured relevance threshold
- **SC-003**: Coverage analysis identifies 100% of indexed chapters and any missing content
- **SC-004**: Validation report clearly shows pass/fail status for each test query with explanations
- **SC-005**: All stored chunks have complete metadata (100% metadata completeness rate)
- **SC-006**: Query response time is under 2 seconds for single queries (user perceives instant results)
- **SC-007**: System successfully handles edge cases (empty queries, non-existent content, very long queries) without crashing

## Assumptions

- Qdrant collection "rag_embedding" already exists and was populated by the embedding pipeline (from feature 007-embedding-pipeline)
- Cohere API key is available for generating query embeddings (same as used for ingestion)
- Test queries will be manually curated to cover major topics across all documentation modules
- Similarity threshold of 0.70 is considered acceptable for "relevant" results (industry standard for semantic search)
- Qdrant is accessible at the URL configured in .env file
- The retrieval validation tool will be a Python script (or function in existing backend/main.py) that can be run from command line

## Dependencies

- Requires completed embedding pipeline (feature 007-embedding-pipeline) with populated Qdrant collection
- Requires access to Docusaurus sitemap.xml for coverage comparison
- Requires Cohere API for query embedding generation
- Requires Qdrant client library for vector search operations

## Constraints

- Must use the same Cohere model (embed-english-v3.0) and input_type="search_query" for query embeddings
- Must query the same Qdrant collection ("rag_embedding") used by the ingestion pipeline
- Cannot modify stored embeddings or chunks during validation (read-only operations)
- Validation should not depend on specific documentation content (queries should be configurable)

## Out of Scope

- Implementing the actual RAG question-answering system (this feature only validates retrieval)
- Re-ingestion or updating of embeddings (covered by embedding pipeline feature)
- Web UI for running queries (command-line/script execution is sufficient)
- Performance optimization of Qdrant queries (focus is on validation, not optimization)
- A/B testing different embedding models or chunking strategies
