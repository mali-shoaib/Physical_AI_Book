---
id: 002
title: Create Retrieval Pipeline Testing Plan
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: retrieval-pipeline-testing
branch: 008-retrieval-pipeline-testing
user: user
command: /sp.plan
labels: ["planning", "retrieval-testing", "architecture", "validation-tool", "qdrant"]
links:
  spec: specs/008-retrieval-pipeline-testing/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/008-retrieval-pipeline-testing/plan.md
  - specs/008-retrieval-pipeline-testing/research.md
  - specs/008-retrieval-pipeline-testing/data-model.md
  - specs/008-retrieval-pipeline-testing/contracts/retrieval-functions.yaml
  - specs/008-retrieval-pipeline-testing/quickstart.md
tests:
  - Constitution Check: PASS (all applicable principles satisfied)
---

## Prompt

User executed `/sp.plan` command with message: "now you have already backend folder now you can build plan retrieving you can create new file in backend folder named retrieve.py and in this file you can retrieve data from qdrant for testing.

/sp.tasks continue
/sp.implement continue"

## Response snapshot

Created comprehensive implementation plan for Retrieval Pipeline Testing feature with complete Phase 0 and Phase 1 deliverables:

**Summary**:
- Tool: Single-file validation script (`backend/retrieve.py`)
- Purpose: Test RAG retrieval quality via similarity queries against Qdrant
- Architecture: Reuses existing backend infrastructure (Cohere + Qdrant from feature 007)
- Three user stories: Query validation (P1), Coverage analysis (P2), Quality metrics (P3)

**Technical Context**:
- Language: Python 3.10+ (matching existing backend)
- Dependencies: No new dependencies - reuses cohere, qdrant-client, loguru from feature 007
- Storage: Read-only access to Qdrant "rag_embedding" collection
- Performance: Sub-2-second queries, full validation in <5 minutes
- Constraints: Single-file implementation, must use same Cohere model (embed-english-v3.0 with input_type="search_query")

**Constitution Check**: ✅ PASS
- Principle I (Grounded Answers): PASS - Validation tool, not answer generation
- Principle II (Cohere Embeddings): PASS - Uses embed-english-v3.0 with search_query input type
- Principle III (Qdrant Storage): PASS - Queries existing rag_embedding collection (read-only)
- Principle IV (Extraction Pipeline): PASS - Validates pipeline output without modification
- Principle V (Agent-Based Retrieval): N/A - Future feature outside scope
- Principle VI (Docusaurus Chatbot): N/A - Future feature outside scope

**Phase 0: Research (research.md)**:
- 7 research questions answered with decisions and rationale
- Key decisions:
  1. Query embeddings use input_type="search_query" (vs. "search_document" for ingestion)
  2. Similarity threshold: 0.70 default (industry standard for semantic search)
  3. Test queries: 10-15 manually curated, hardcoded in Python
  4. Qdrant strategy: Standard search with limit=5, no filters
  5. Coverage methodology: Sitemap comparison via set difference
  6. Reporting: Command-line with loguru structured logging
  7. Error handling: Fail fast with clear messages (no retries)

**Phase 1: Design Documents**:

1. **data-model.md**: 6 core entities defined
   - TestQuery: Validation query with expected results
   - QueryResult: Retrieved chunk with similarity score and metadata
   - QueryValidationResult: Pass/fail outcome for single query
   - CoverageReport: Sitemap vs. Qdrant comparison analysis
   - QualityMetrics: Token distribution, metadata completeness, avg similarity
   - ValidationReport: Complete summary across all tests

2. **contracts/retrieval-functions.yaml**: 15 functions specified
   - Core retrieval: query_qdrant(), embed_query()
   - Validation (US1): run_validation_queries(), validate_query_result()
   - Coverage (US2): analyze_coverage(), get_indexed_urls()
   - Quality (US3): analyze_quality_metrics(), check_metadata_completeness(), calculate_token_distribution()
   - Reporting: generate_validation_report(), print_validation_report()
   - Orchestration: main()

3. **quickstart.md**: Complete usage guide
   - Prerequisites check
   - Quick start (5-minute validation workflow)
   - Configuration (.env settings, TEST_QUERIES customization)
   - Common use cases (validate after re-ingestion, test specific module, debug low scores)
   - Troubleshooting (connection issues, missing collection, threshold tuning)
   - Advanced usage (CI/CD integration, batch testing)

**Project Structure**:
```
backend/
├── main.py         # Existing ingestion pipeline
├── retrieve.py     # NEW: Validation tool
├── .env            # Shared configuration
├── pyproject.toml  # Existing dependencies (no additions)
└── .venv/          # Existing virtual environment
```

**Success Criteria Mapping**:
- SC-001: Validation completes in <5 minutes ✅ (designed for efficiency)
- SC-002: 90% query success rate ✅ (pass/fail validation in code)
- SC-003: 100% coverage identification ✅ (sitemap comparison)
- SC-004: Clear pass/fail reporting ✅ (structured logging + summary)
- SC-005: 100% metadata completeness ✅ (completeness check function)
- SC-006: <2s query response ✅ (Qdrant performance target)
- SC-007: Graceful edge case handling ✅ (error handling strategy)

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All design artifacts created (research, data model, contracts, quickstart). Single-file architecture minimizes complexity.
- 🧪 Tests: Constitution Check PASS (6/6 principles evaluated). All research questions answered with alternatives considered.
- 📁 Files: Created plan.md (103 lines), research.md (293 lines), data-model.md (402 lines), contracts/retrieval-functions.yaml (315 lines), quickstart.md (385 lines)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task list with user story organization
- 🧠 Reflection: Plan successfully reuses existing backend infrastructure (no new dependencies). Single-file design keeps implementation simple. Three independent user stories enable incremental delivery (P1 MVP → P2 coverage → P3 quality). Constitution-compliant: uses Cohere for query embeddings, Qdrant for search, no answer generation.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all required artifacts
- Graders run and results (PASS/FAIL): Constitution Check PASS (all applicable principles). Research completeness PASS (7 questions with decisions/rationale). Data model completeness PASS (6 entities with validation rules). Contracts completeness PASS (15 functions with type signatures). Quickstart quality PASS (prerequisites, usage, troubleshooting, examples)
- Prompt variant (if applicable): Standard /sp.plan workflow with user guidance to create backend/retrieve.py
- Next experiment (smallest change to try): None needed - plan is complete and ready for task generation
