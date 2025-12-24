# Tasks: Retrieval Pipeline Testing

**Input**: Design documents from `/specs/008-retrieval-pipeline-testing/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/retrieval-functions.yaml, quickstart.md

**Tests**: No test tasks included (not requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This feature uses single-file architecture as specified:
- Validation script: `backend/retrieve.py` (all functions in one file)
- Configuration: Reuses `backend/.env` from feature 007
- Documentation: Update `backend/README.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create retrieval validation script structure

- [X] T001 Create `backend/retrieve.py` with imports and environment loading (cohere, qdrant-client, requests, loguru, python-dotenv)
- [X] T002 Add environment variable loading in backend/retrieve.py - load .env file, validate required vars (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DOCUSAURUS_BASE_URL)
- [X] T003 Add configuration constants in backend/retrieve.py - SIMILARITY_THRESHOLD (default 0.70), TOP_K_RESULTS (default 5), LOG_LEVEL (from env)
- [X] T004 Configure loguru logger in backend/retrieve.py - set format (timestamp + level + message), log level from env
- [X] T005 Initialize Qdrant client connection function in backend/retrieve.py - create `init_qdrant_client()` that returns QdrantClient with error handling

**Checkpoint**: Project structure ready, retrieve.py initialized with configuration and logging

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core retrieval functions that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Implement `embed_query(query_text: str) -> List[float]` function in backend/retrieve.py - Cohere API call with model="embed-english-v3.0", input_type="search_query", return 1024-dim vector
- [X] T007 Implement `query_qdrant(query_text: str, top_k: int = 5, threshold: float = 0.70) -> List[Dict]` function in backend/retrieve.py - call embed_query(), search Qdrant collection "rag_embedding", filter by similarity threshold, return top_k results with metadata
- [X] T008 Add helper function `parse_query_result(qdrant_result) -> Dict` in backend/retrieve.py - extract chunk_id, chunk_text, similarity_score, source_url, chapter_id, module_name, heading_hierarchy from Qdrant point
- [X] T009 Add `get_all_urls(base_url: str) -> List[str]` function in backend/retrieve.py - fetch sitemap.xml, parse XML, extract /docs/* URLs (reuse logic from main.py)

**Checkpoint**: Foundation ready - all core retrieval and sitemap functions implemented

---

## Phase 3: User Story 1 - Basic Similarity Search Validation (Priority: P1) 🎯 MVP

**Goal**: Run test queries against Qdrant and verify relevant chunks are retrieved with appropriate similarity scores

**Independent Test**: Run `python backend/retrieve.py --validate-queries` and verify 90%+ queries pass with similarity >= 0.70 and module match

**Acceptance Criteria**:
- Test queries execute successfully against Qdrant
- Results include similarity scores and metadata
- Validation logic checks: similarity >= threshold AND expected_module match
- Pass rate calculated and logged
- At least 90% of test queries pass

### Implementation for User Story 1

- [X] T010 [US1] Define TEST_QUERIES constant in backend/retrieve.py - list of 10-15 dicts with query_text, expected_module, expected_chapter (optional), min_similarity (0.70), description
- [X] T011 [US1] Implement `validate_query_result(query: Dict, results: List[Dict]) -> Dict` function in backend/retrieve.py - check if best_similarity >= min_similarity, check if expected_module in top results, return QueryValidationResult dict (passed, pass_reason, fail_reason, best_similarity, module_match)
- [X] T012 [US1] Implement `run_validation_queries(test_queries: List[Dict]) -> List[Dict]` function in backend/retrieve.py - iterate test_queries, call query_qdrant() for each, call validate_query_result(), log progress [N/M], return list of QueryValidationResult
- [X] T013 [US1] Add `print_query_validation_report(results: List[Dict])` function in backend/retrieve.py - calculate total/passed/failed counts, calculate pass_rate, log summary table with pass/fail per query, log final pass rate with color coding (green >= 90%, red < 90%)
- [X] T014 [US1] Add edge case handling in query_qdrant() - handle empty query text (ValueError), handle Qdrant connection errors (ConnectionError with clear message), handle Cohere API errors (log and re-raise)

**Checkpoint**: User Story 1 complete - run validation queries, get pass/fail report with 90%+ success rate

---

## Phase 4: User Story 2 - Coverage and Completeness Validation (Priority: P2)

**Goal**: Verify all documentation chapters were successfully ingested and are searchable in Qdrant

**Independent Test**: Run `python backend/retrieve.py --coverage` and verify 100% coverage (all sitemap URLs found in Qdrant)

**Acceptance Criteria**:
- Sitemap URLs extracted successfully
- Qdrant indexed URLs queried and collected
- Missing URLs identified (sitemap - qdrant)
- Extra URLs identified (qdrant - sitemap)
- Coverage percentage calculated (indexed/total * 100)

### Implementation for User Story 2

- [X] T015 [US2] Implement `get_indexed_urls(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> Set[str]` function in backend/retrieve.py - use scroll API to iterate all points, extract payload.source_url, return deduplicated set
- [X] T016 [US2] Implement `analyze_coverage(sitemap_url: str, qdrant_client: QdrantClient) -> Dict` function in backend/retrieve.py - call get_all_urls(sitemap_url), call get_indexed_urls(), calculate missing_urls (set difference), calculate coverage_percentage, return CoverageReport dict
- [X] T017 [US2] Add `print_coverage_report(coverage: Dict)` function in backend/retrieve.py - log total_sitemap_urls, indexed_urls count, missing_urls (if any with full URLs), coverage_percentage with color coding (green=100%, yellow>=90%, red<90%)
- [X] T018 [US2] Add error handling for sitemap fetching - handle HTTP errors (requests.RequestException), handle XML parsing errors, log clear error messages with troubleshooting hints

**Checkpoint**: Coverage analysis works independently - compare Qdrant to sitemap, identify gaps, report coverage percentage

---

## Phase 5: User Story 3 - Embedding Quality Metrics (Priority: P3)

**Goal**: Measure embedding quality metrics (chunk distribution, metadata completeness, avg similarity)

**Independent Test**: Run `python backend/retrieve.py --quality` and verify 100% metadata completeness, token distribution within range, avg similarity >= 0.70

**Acceptance Criteria**:
- All chunks analyzed for metadata completeness
- Token distribution calculated (min/max/avg/median)
- Average similarity score calculated from validation results
- Quality metrics logged in summary report

### Implementation for User Story 3

- [X] T019 [US3] Define REQUIRED_METADATA_FIELDS constant in backend/retrieve.py - list of ["source_url", "chapter_id", "module_name", "heading_hierarchy", "token_count", "chunk_index"]
- [X] T020 [US3] Implement `check_metadata_completeness(payload: Dict) -> bool` function in backend/retrieve.py - verify all REQUIRED_METADATA_FIELDS present in payload, return True/False
- [X] T021 [US3] Implement `calculate_token_distribution(token_counts: List[int]) -> Dict` function in backend/retrieve.py - calculate min/max/avg/median using statistics module, return dict with distribution stats
- [X] T022 [US3] Implement `analyze_quality_metrics(qdrant_client: QdrantClient, query_results: List[Dict]) -> Dict` function in backend/retrieve.py - scroll all points to collect metadata, check completeness with check_metadata_completeness(), extract token_counts, calculate distribution, aggregate similarity scores from query_results, return QualityMetrics dict
- [X] T023 [US3] Add `print_quality_metrics_report(metrics: Dict)` function in backend/retrieve.py - log total_chunks, metadata_completeness_rate (color: green=100%, red<100%), token_distribution (min/max/avg/median), avg_similarity_score, similarity_distribution

**Checkpoint**: Quality metrics work independently - analyze chunks, validate metadata, report statistics

---

## Phase 6: Main Orchestration & Reporting

**Purpose**: Integrate all user stories into unified validation workflow

- [X] T024 Implement `generate_validation_report(query_results: List[Dict], coverage: Dict, quality: Dict, duration: float) -> Dict` function in backend/retrieve.py - aggregate all results, calculate overall pass_rate, add timestamp, return ValidationReport dict
- [X] T025 Implement `print_validation_report(report: Dict)` function in backend/retrieve.py - print header separator, log all sections (query validation, coverage, quality), log duration, log overall SUCCESS/FAIL status based on criteria (pass_rate >= 90%, coverage >= 100%, metadata_completeness == 100%)
- [X] T026 Implement `main()` orchestration function in backend/retrieve.py - load env vars, init Qdrant client, run Phase 1 (query validation), run Phase 2 (coverage analysis), run Phase 3 (quality metrics), generate final report, print report, exit with code 0 if success else 1
- [X] T027 Add CLI argument parsing in backend/retrieve.py - use argparse to support --validate-queries, --coverage, --quality, --all (default), --log-level flags
- [X] T028 Add `if __name__ == "__main__": main()` entry point to backend/retrieve.py

**Checkpoint**: Complete validation tool ready - run all phases, generate comprehensive report

---

## Phase 7: Polish & Documentation

**Purpose**: Final touches and documentation updates

- [X] T029 [P] Update backend/README.md with retrieve.py usage - add "Retrieval Validation" section with quickstart command, expected output, configuration options (.env variables), troubleshooting tips
- [X] T030 [P] Add comprehensive docstrings to all functions in backend/retrieve.py - include function purpose, parameters with types, return type, raises exceptions, example usage
- [X] T031 Add type hints throughout backend/retrieve.py - ensure all function signatures have proper type annotations (str, int, float, List, Dict, Set, etc.)
- [X] T032 [P] Add example test queries to backend/retrieve.py - ensure TEST_QUERIES covers all major modules (ROS 2, URDF, Isaac Sim, VLA, etc.) with realistic developer questions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if multiple developers)
  - Or sequentially in priority order: US1 → US2 → US3
- **Orchestration (Phase 6)**: Depends on all user stories being complete
- **Polish (Phase 7)**: Can start after Phase 6

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1, but logically follows
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 query_results for similarity stats

### Within Each User Story

- User Story 1: Sequential within story (T010 → T011 → T012 → T013 → T014)
- User Story 2: Sequential within story (T015 → T016 → T017 → T018)
- User Story 3: Sequential within story (T019 → T020 → T021 → T022 → T023)

### Parallel Opportunities

**Phase 1 (Setup)**:
- All tasks are sequential (setting up single file)

**Phase 2 (Foundational)**:
- T006 and T009 can run in parallel (different functions, no dependencies)
- T007 depends on T006 (needs embed_query)
- T008 can run in parallel with T006 and T009

**Phase 7 (Polish)**:
- T029, T030, T032 can run in parallel (documentation vs. code vs. test data)
- T031 should be done after code is complete

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T009) - CRITICAL
3. Complete Phase 3: User Story 1 (T010-T014)
4. **STOP and VALIDATE**: Run `python backend/retrieve.py` and verify 90%+ queries pass
5. Test with actual queries: "What is ROS 2?", "How do I create URDF?"
6. Verify: Queries execute, similarity scores returned, pass/fail logic works
7. **This is the MVP!** ✅

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → **Deploy/Demo (MVP!)**
   - At this point: Query validation working, can verify retrieval quality
3. Add User Story 2 → Test independently → Deploy/Demo
   - At this point: Coverage analysis added, can identify missing chapters
4. Add User Story 3 → Test independently → Deploy/Demo
   - At this point: Quality metrics added, full pipeline validation

### Parallel Team Strategy

With multiple developers after Foundational phase:

1. Team completes Setup + Foundational together (T001-T009)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T010-T014) - Priority 1 MVP
   - **Developer B**: User Story 2 (T015-T018) - Can start in parallel
   - **Developer C**: User Story 3 (T019-T023) - Can start after US1 for similarity stats
3. Merge all functions into backend/retrieve.py (careful with conflicts)
4. Developer A: Orchestration (T024-T028)
5. All developers: Polish (T029-T032) in parallel

---

## Notes

- **Single-file architecture**: All functions go in backend/retrieve.py
- **No new dependencies**: Reuses all packages from backend/pyproject.toml (feature 007)
- **Shared .env**: Uses same configuration as backend/main.py
- Each user story should be independently testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **MVP is User Story 1**: Query validation with 90%+ pass rate

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 4 tasks (BLOCKING - critical path)
- **Phase 3 (US1 - MVP)**: 5 tasks
- **Phase 4 (US2)**: 4 tasks
- **Phase 5 (US3)**: 5 tasks
- **Phase 6 (Orchestration)**: 5 tasks
- **Phase 7 (Polish)**: 4 tasks

**Total**: 32 tasks

**Parallel Opportunities**: 5 tasks marked [P] can run in parallel

**Critical Path**: Phase 1 (5 tasks) → Phase 2 (4 tasks) → Phase 3 (5 tasks) → Phase 6 (5 tasks) = **19 tasks for MVP**
