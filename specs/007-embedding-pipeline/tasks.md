# Tasks: Embedding Pipeline Setup

**Input**: Design documents from `/specs/007-embedding-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/pipeline-functions.yaml

**Tests**: No test tasks included (not requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This feature uses single-file architecture as specified:
- Backend pipeline: `backend/main.py` (all functions in one file)
- Configuration: `backend/pyproject.toml`, `backend/.env`
- Documentation: `backend/README.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure: `backend/` folder at repository root
- [X] T002 Create `backend/pyproject.toml` with UV dependencies (cohere>=5.0.0, qdrant-client>=1.7.0, beautifulsoup4>=4.12.0, tiktoken>=0.5.0, tenacity>=8.2.0, loguru>=0.7.0, python-dotenv>=1.0.0, lxml>=4.9.0)
- [X] T003 [P] Create `backend/.env.example` with environment variable template (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DOCUSAURUS_BASE_URL, CHUNK_SIZE, CHUNK_OVERLAP, LOG_LEVEL)
- [X] T004 [P] Create `backend/.gitignore` to ignore .env, __pycache__, logs/, *.log, .vscode/, .idea/
- [X] T005 [P] Create `backend/README.md` with pipeline documentation (architecture, setup instructions, usage guide from quickstart.md)
- [X] T006 Initialize UV virtual environment and install dependencies in backend/ folder

**Checkpoint**: Project structure ready, dependencies installable

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core pipeline functions that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Implement `get_all_urls(base_url: str) -> List[str]` function in backend/main.py - crawl sitemap.xml, filter /docs/* URLs, handle HTTP errors with retry
- [X] T008 [P] Implement `extract_text_from_url(url: str) -> Tuple[str, Dict]` function in backend/main.py - fetch HTML, remove nav/footer, extract text from article.markdown, create metadata (source_url, chapter_id, module_name, heading_hierarchy)
- [X] T009 [P] Implement `chunk_text(text: str, metadata: Dict) -> List[Tuple[str, Dict]]` function in backend/main.py - recursive splitting on paragraphs, 512-1024 tokens with tiktoken, 10-20% overlap, generate chunk metadata with UUID
- [X] T010 [P] Implement `embed(texts: List[str]) -> List[List[float]]` function in backend/main.py - Cohere API batch calls (up to 96 texts), model="embed-english-v3.0", input_type="search_document", exponential backoff retry
- [X] T011 [P] Implement `create_collection(client: QdrantClient, name: str)` function in backend/main.py - initialize collection "rag_embedding", vector size 1024, distance Cosine, idempotent (skip if exists)
- [X] T012 [P] Implement `save_chunk_to_qdrant(client: QdrantClient, chunk_text: str, embedding: List[float], metadata: Dict)` function in backend/main.py - upsert PointStruct with vector and payload, exponential backoff retry
- [X] T013 Add environment variable loading with python-dotenv in backend/main.py - load .env file, validate required vars (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, DOCUSAURUS_BASE_URL)
- [X] T014 [P] Add structured logging with loguru in backend/main.py - configure log format, log level from env, timestamp + level + function name + message

**Checkpoint**: Foundation ready - all core functions implemented and testable

---

## Phase 3: User Story 1 - Initial Content Ingestion (Priority: P1) 🎯 MVP

**Goal**: Extract all published textbook content from deployed Docusaurus site and store in Qdrant for RAG retrieval

**Independent Test**: Run pipeline against production URL, verify all chapters are crawled, chunked, embedded, and stored in Qdrant with metadata. Query Qdrant for "What is ROS 2?" and verify relevant chunks are returned.

**Acceptance Criteria**:
- All published chapters crawled and extracted
- Content chunked into 512-1024 token segments with 10-20% overlap
- Cohere embeddings generated successfully for all chunks
- All chunks stored in Qdrant with metadata (source_url, chapter_id, module_name, heading_hierarchy)
- Similarity search returns relevant results

### Implementation for User Story 1

- [X] T015 [US1] Implement `main()` orchestration function in backend/main.py - initialize Qdrant client, call create_collection(), execute pipeline flow (crawl → extract → chunk → embed → store)
- [X] T016 [US1] Add ingestion job statistics tracking in main() - initialize stats dict with pages_discovered, pages_fetched_success/failed, chunks_created, embeddings_generated, qdrant_upserts_success/failed
- [X] T017 [US1] Add progress logging throughout main() - log pipeline start, URL discovery count, per-page progress ([N/M] Processing URL), chunk creation count, embedding generation count, storage success count
- [X] T018 [US1] Add error handling in main() pipeline loop - try/except around each URL processing, log errors with URL/chunk_id, track failed operations in stats, continue processing remaining URLs
- [X] T019 [US1] Add final summary report generation in main() - calculate success rate, log final stats (pages discovered/fetched/failed, chunks/embeddings/upserts), log total duration, save to job_report dict
- [X] T020 [US1] Add `if __name__ == "__main__": main()` entry point to backend/main.py

**Checkpoint**: At this point, User Story 1 should be fully functional - run `python backend/main.py` and verify 100% ingestion success

---

## Phase 4: User Story 2 - Incremental Content Updates (Priority: P2)

**Goal**: Update vector database when textbook chapters are modified without full re-ingestion

**Independent Test**: Modify a single chapter in Docusaurus, run incremental update, verify only changed chunks are re-embedded and updated in Qdrant

**Acceptance Criteria**:
- Modified chapters detected and re-processed
- New chapters added to collection
- Deleted chapters removed from Qdrant
- Incremental update completes within 2 minutes for single chapter

### Implementation for User Story 2

- [ ] T021 [US2] Add URL change detection function in backend/main.py - store fetch timestamps in JSON tracking file (backend/tracking.json), compare HTTP Last-Modified headers, return list of changed/new/deleted URLs
- [ ] T022 [US2] Add incremental mode flag to main() - check INCREMENTAL_UPDATE env var or --incremental CLI arg, call change detection if enabled
- [ ] T023 [US2] Implement selective URL processing in main() - if incremental mode, only process changed/new URLs from change detection
- [ ] T024 [US2] Implement Qdrant chunk deletion function in backend/main.py - delete points by source_url filter, handle deleted URLs
- [ ] T025 [US2] Update tracking file after successful ingestion - save current timestamps to backend/tracking.json for next incremental run
- [ ] T026 [US2] Add incremental mode logging - log change detection results (X changed, Y new, Z deleted), log per-URL update status

**Checkpoint**: Incremental updates work independently - modify one chapter, run with INCREMENTAL_UPDATE=true, verify only that chapter re-processed

---

## Phase 5: User Story 3 - Quality Validation and Monitoring (Priority: P3)

**Goal**: Validate ingestion quality and monitor embedding coverage to ensure complete and accurate content

**Independent Test**: Run validation queries against ingested content, verify 95%+ success rate with similarity scores above threshold

**Acceptance Criteria**:
- 95% of test queries retrieve relevant chunks above similarity threshold (0.70)
- Coverage report shows which chapters/modules are fully ingested
- Errors logged with specific URLs and chunk IDs for debugging

### Implementation for User Story 3

- [ ] T027 [US3] Create validation query test suite in backend/main.py - define test_queries list with dicts (query_text, expected_module, expected_chapter_id, min_similarity=0.70)
- [ ] T028 [US3] Implement validation query executor in backend/main.py - for each test query: embed query text with input_type="search_query", query Qdrant, check similarity score and module match
- [ ] T029 [US3] Add validation results logging - log pass/fail per query, calculate pass rate, log failed queries with expected vs actual results
- [ ] T030 [US3] Implement coverage analysis function in backend/main.py - query Qdrant for all unique module_names and chapter_ids, compare with sitemap URL list, generate missing content report
- [ ] T031 [US3] Add validation mode flag to main() - check VALIDATION_MODE env var or --validate CLI arg, run validation queries after ingestion if enabled
- [ ] T032 [US3] Add validation summary to job report - include validation_pass_rate, total_queries, passed_queries, failed_queries, coverage_percent

**Checkpoint**: Validation works independently - run with VALIDATION_MODE=true, verify 95%+ queries pass and coverage report generated

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T033 [P] Add comprehensive error logging for all pipeline stages - structured error format with error_id, error_type, stage, resource_id, message, traceback, timestamp, retry_count
- [ ] T034 [P] Add performance optimization for batch embedding - maximize batch size to 96 texts per Cohere API call, add progress logging for large batches
- [ ] T035 Add CLI argument parsing in backend/main.py - support --incremental, --validate, --base-url, --log-level flags using argparse
- [ ] T036 [P] Update backend/README.md with full usage examples - add quickstart commands, troubleshooting section, example outputs, validation examples
- [ ] T037 [P] Add example .env configuration to backend/README.md - document all environment variables with descriptions and example values
- [ ] T038 Code cleanup and add type hints throughout backend/main.py - ensure all functions have proper type annotations per contracts/pipeline-functions.yaml

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can proceed in parallel (if multiple developers)
  - Or sequentially in priority order: US1 → US2 → US3
- **Polish (Phase 6)**: Depends on desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent but builds on US1's pipeline
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent but validates US1's output

### Within Each User Story

- User Story 1: Sequential within story (T015 → T016 → T017 → T018 → T019 → T020)
- User Story 2: Sequential within story (T021 → T022 → T023 → T024 → T025 → T026)
- User Story 3: Sequential within story (T027 → T028 → T029 → T030 → T031 → T032)

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003, T004, T005 can run in parallel (different files, no dependencies)

**Phase 2 (Foundational)**:
- T008, T009, T010, T011, T012, T014 can run in parallel (different function implementations in same file - requires careful merging or separate branches)
- T007 should complete first (provides URLs for other functions to work with)

**Phase 6 (Polish)**:
- T033, T034, T036, T037 can run in parallel (different aspects of the same file)

---

## Parallel Example: Foundational Phase

```bash
# After T007 completes, these can be developed in parallel:
# Developer A:
Task T008: "Implement extract_text_from_url() function"
Task T009: "Implement chunk_text() function"

# Developer B:
Task T010: "Implement embed() function"
Task T011: "Implement create_collection() function"

# Developer C:
Task T012: "Implement save_chunk_to_qdrant() function"
Task T014: "Add structured logging with loguru"

# Then merge all function implementations into backend/main.py
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T014) - CRITICAL
3. Complete Phase 3: User Story 1 (T015-T020)
4. **STOP and VALIDATE**: Run `python backend/main.py` against production Docusaurus URL
5. Verify: 100+ URLs discovered, 95%+ success rate, all chunks in Qdrant
6. Test query: "What is ROS 2?" returns relevant results
7. Deploy/demo if ready ✅ **This is the MVP!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → **Deploy/Demo (MVP!)**
   - At this point: Full ingestion pipeline working, can answer questions
3. Add User Story 2 → Test independently → Deploy/Demo
   - At this point: Incremental updates working, reduces operational overhead
4. Add User Story 3 → Test independently → Deploy/Demo
   - At this point: Quality validation working, confidence in system accuracy

### Parallel Team Strategy

With multiple developers after Foundational phase:

1. Team completes Setup + Foundational together (T001-T014)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T015-T020) - Priority 1 MVP
   - **Developer B**: User Story 2 (T021-T026) - Can start in parallel
   - **Developer C**: User Story 3 (T027-T032) - Can start in parallel
3. Each story integrates independently into backend/main.py
4. Test each story works on its own before integration

---

## Notes

- **[P] tasks** = different files or independent sections, no blocking dependencies
- **[Story] label** maps task to specific user story for traceability
- **Single-file architecture**: All functions go in backend/main.py per user requirement
- Each user story should be independently testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **MVP is User Story 1**: Full ingestion pipeline with 100% success rate

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 8 tasks (BLOCKING - critical path)
- **Phase 3 (US1 - MVP)**: 6 tasks
- **Phase 4 (US2)**: 6 tasks
- **Phase 5 (US3)**: 6 tasks
- **Phase 6 (Polish)**: 6 tasks

**Total**: 38 tasks

**Parallel Opportunities**: 11 tasks marked [P] can run in parallel with proper coordination

**Critical Path**: Phase 1 (6 tasks) → Phase 2 (8 tasks) → Phase 3 (6 tasks) = **20 tasks for MVP**
