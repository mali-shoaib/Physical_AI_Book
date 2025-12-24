# Tasks: Retrieval-Enabled Agent

**Input**: Design documents from `/specs/009-retrieval-agent/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/agent-functions.yaml, quickstart.md

**Tests**: No test tasks included (not requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This feature uses single-file architecture as specified:
- Agent implementation: `backend/agent.py` (all functions in one file)
- Configuration: Reuses `backend/.env` from features 007/008
- Documentation: Update `backend/README.md`
- Dependencies: Update `backend/pyproject.toml`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare project structure and dependencies for agent implementation

- [X] T001 Verify backend virtual environment exists (backend/.venv/) and is activated
- [X] T002 Add openai dependency to backend/pyproject.toml (openai>=1.0.0)
- [X] T003 Install openai package using UV or pip in backend/.venv
- [X] T004 Add OPENAI_API_KEY to backend/.env file with placeholder value
- [X] T005 Add optional environment variables to backend/.env (OPENAI_MODEL=gpt-4, AGENT_MAX_HISTORY=20)

**Checkpoint**: Dependencies installed, environment configured for agent development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core retrieval and embedding functions that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T006 Create backend/agent.py with imports (openai, cohere, qdrant_client, loguru, dotenv, argparse, uuid, datetime)
- [X] T007 Add environment variable loading in backend/agent.py - load .env file, validate required vars (OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [X] T008 Configure loguru logger in backend/agent.py - set format (timestamp + level + message), log level from env (default INFO)
- [X] T009 Implement embed_query(query_text: str) -> List[float] function in backend/agent.py - reuse Cohere client from feature 008, model="embed-english-v3.0", input_type="search_query"
- [X] T010 Implement init_qdrant_client() -> QdrantClient function in backend/agent.py - connect to Qdrant with error handling, verify connection
- [X] T011 Implement retrieve_context(query: str, top_k: int = 5, threshold: float = 0.70) -> List[Dict] function in backend/agent.py - call embed_query(), search Qdrant collection "rag_embedding", return results with metadata (chunk_id, chunk_text, similarity_score, source_url, chapter_id, module_name)

**Checkpoint**: Foundation ready - all core retrieval and embedding functions implemented

---

## Phase 3: User Story 1 - Command-Line Q&A Agent (Priority: P1) 🎯 MVP

**Goal**: Single-question command-line agent that retrieves from Qdrant and generates grounded answers

**Independent Test**: Run `python backend/agent.py "What is ROS 2?"` and verify it retrieves relevant chunks, generates a grounded answer with citations, and completes in <10 seconds

**Acceptance Criteria**:
- Agent retrieves relevant chunks from Qdrant (similarity >= 0.70)
- Agent generates answer based only on retrieved context
- Answer includes inline citations (e.g., "[Source: Module 1 - ROS 2 Basics]")
- Out-of-scope questions return "I don't have information about that in the textbook"
- Multi-part questions retrieve content for all concepts

### Implementation for User Story 1

- [X] T012 [US1] Define SYSTEM_PROMPT constant in backend/agent.py - explicit grounding instructions ("Answer ONLY based on provided context", failure mode for no context, citation requirements)
- [X] T013 [US1] Implement format_context_for_agent(results: List[Dict]) -> str function in backend/agent.py - format retrieved chunks with citations (source URLs, module names, chapter IDs) for OpenAI agent
- [X] T014 [US1] Implement create_retrieval_tool_schema() -> Dict function in backend/agent.py - define OpenAI function tool schema with name="retrieve_context", parameters (query, top_k, threshold), descriptions
- [X] T015 [US1] Implement initialize_agent(model: str = "gpt-4") -> Tuple[openai.Client, str] function in backend/agent.py - create OpenAI client, generate session_id (UUID), set system prompt, register retrieval tool
- [X] T016 [US1] Implement execute_function_call(function_name: str, arguments: Dict) -> str function in backend/agent.py - handle "retrieve_context" calls from agent, call retrieve_context(), format results, return to agent
- [X] T017 [US1] Implement ask_question(client: openai.Client, question: str, session_id: str) -> Dict function in backend/agent.py - send question to agent, handle function calls, get response with citations, return AgentResponse dict (answer, citations, retrieval_count, tokens_used, grounded)
- [X] T018 [US1] Implement single_question_mode(question: str, args: argparse.Namespace) function in backend/agent.py - initialize agent, call ask_question(), print answer and citations (if --show-citations), log duration, exit with code 0 or 1
- [X] T019 [US1] Add CLI argument parsing in backend/agent.py - use argparse for positional question argument, --model, --show-citations, --log-level, --threshold, --top-k flags
- [X] T020 [US1] Add if __name__ == "__main__": main() entry point in backend/agent.py - validate environment, parse args, call single_question_mode() or print help
- [X] T021 [US1] Add error handling in backend/agent.py - retry logic for OpenAI API (3 attempts, exponential backoff), handle Qdrant errors, handle Cohere errors, log all errors with clear messages

**Checkpoint**: User Story 1 complete - single-question agent works, retrieves from Qdrant, generates grounded answers

---

## Phase 4: User Story 2 - Interactive Conversation Mode (Priority: P2)

**Goal**: Multi-turn conversation with context maintenance across questions

**Independent Test**: Run `python backend/agent.py --interactive`, ask "What is Isaac Sim?" then "How do I use it for synthetic data?", verify agent understands "it" refers to Isaac Sim and maintains context

**Acceptance Criteria**:
- Agent maintains conversation history across multiple turns
- Follow-up questions use conversation context
- Agent understands pronoun references (e.g., "it", "that")
- Conversation history respects max_history limit (sliding window)
- User can reset conversation with "reset" command

### Implementation for User Story 2

- [X] T022 [US2] Implement ConversationSession class in backend/agent.py - attributes: session_id, model, conversation_history (List[Dict]), max_history (int), started_at (datetime)
- [X] T023 [US2] Implement add_message(session: ConversationSession, role: str, content: str) method in backend/agent.py - append message to history, enforce max_history with sliding window (remove oldest user-assistant pair)
- [X] T024 [US2] Implement reset_conversation(session: ConversationSession) method in backend/agent.py - clear conversation_history except system message, log reset action
- [X] T025 [US2] Implement interactive_mode(args: argparse.Namespace) function in backend/agent.py - create ConversationSession, loop with input("You: "), handle special commands (exit/quit/bye, reset, help), call ask_question() with conversation history, add responses to history, print answers
- [X] T026 [US2] Update CLI argument parsing in backend/agent.py - add --interactive flag, --max-history argument (default 20)
- [X] T027 [US2] Update main() function in backend/agent.py - check if --interactive flag set, call interactive_mode() or single_question_mode() based on args
- [X] T028 [US2] Add conversation context handling in ask_question() - pass full conversation_history to OpenAI API, maintain message format (role + content), handle function calls within conversation context

**Checkpoint**: User Story 2 complete - interactive mode works, maintains context, handles follow-up questions

---

## Phase 5: User Story 3 - Citation and Source Verification (Priority: P3)

**Goal**: Detailed citation metadata for verification and debugging

**Independent Test**: Run `python backend/agent.py "What is VSLAM?" --show-citations`, verify output includes chunk IDs, similarity scores, source URLs, and chapter names for all retrieved chunks

**Acceptance Criteria**:
- Citations include chunk_id, similarity_score, source_url, chapter_id, module_name
- Citations ranked by relevance (similarity score descending)
- Source URLs are valid and link to actual textbook pages
- Citation display can be toggled with --show-citations flag

### Implementation for User Story 3

- [ ] T029 [US3] Implement format_citations_detailed(results: List[Dict]) -> str function in backend/agent.py - format citations with all metadata fields, rank by similarity descending, include chunk IDs and token counts
- [ ] T030 [US3] Update AgentResponse dict structure in backend/agent.py - add citations field (List[RetrievalResult]), include all metadata from Qdrant (chunk_id, similarity_score, source_url, chapter_id, module_name, heading_hierarchy, token_count, chunk_index)
- [ ] T031 [US3] Update single_question_mode() in backend/agent.py - check --show-citations flag, if true call format_citations_detailed() and print after answer
- [ ] T032 [US3] Update interactive_mode() in backend/agent.py - add "citations" command to show detailed citations for last answer, store last AgentResponse in session
- [ ] T033 [US3] Add citation validation in backend/agent.py - verify source_url format (valid HTTP/HTTPS), verify similarity_score range (0.0-1.0), log warnings for invalid citations

**Checkpoint**: User Story 3 complete - detailed citations available, verification works, URLs are valid

---

## Phase 6: Polish & Documentation

**Purpose**: Final touches, documentation, and integration

- [ ] T034 [P] Update backend/README.md with agent.py usage - add "RAG Agent" section with quickstart command, configuration options (.env variables), CLI arguments, example outputs, troubleshooting tips
- [ ] T035 [P] Add comprehensive docstrings to all functions in backend/agent.py - include function purpose, parameters with types, return type, raises exceptions, examples
- [ ] T036 Add type hints throughout backend/agent.py - ensure all function signatures have proper type annotations (str, int, float, List, Dict, Tuple, Optional, etc.)
- [ ] T037 [P] Update backend/pyproject.toml with complete dependency list - verify all dependencies (openai, cohere, qdrant-client, loguru, python-dotenv) with version constraints
- [ ] T038 [P] Add example usage section to backend/README.md - single-question examples, interactive mode examples, citation verification examples, troubleshooting common issues (API key errors, Qdrant connection, rate limits)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories CAN proceed in parallel (if multiple developers)
  - Or sequentially in priority order: US1 → US2 → US3
- **Polish (Phase 6)**: Can start after user stories are functional

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 implementation (extends ask_question, adds session management)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Enhances US1/US2 output (adds citation formatting)

### Within Each User Story

- User Story 1: Sequential within story (T012 → T013 → T014 → T015 → T016 → T017 → T018 → T019 → T020 → T021)
- User Story 2: Sequential within story, depends on US1 functions (T022 → T023 → T024 → T025 → T026 → T027 → T028)
- User Story 3: Sequential within story, depends on US1 functions (T029 → T030 → T031 → T032 → T033)

### Parallel Opportunities

**Phase 1 (Setup)**:
- All tasks are sequential (environment setup)

**Phase 2 (Foundational)**:
- T006-T008 can run together (file creation + config)
- T009-T011 must run sequentially (each depends on previous)

**Phase 6 (Polish)**:
- T034, T035, T037, T038 can run in parallel (different files, documentation)
- T036 should be done after code is complete

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T011) - CRITICAL
3. Complete Phase 3: User Story 1 (T012-T021)
4. **STOP and VALIDATE**: Run `python backend/agent.py "What is ROS 2?"` and verify grounded answer with citations
5. Test with queries from feature 008: "What is Isaac Sim?", "How do I create URDF?"
6. Verify: Retrieval works, answers are grounded, citations included, out-of-scope questions refused
7. **This is the MVP!** ✅

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → **Deploy/Demo (MVP!)**
   - At this point: Single-question agent working, RAG functionality proven
3. Add User Story 2 → Test independently → Deploy/Demo
   - At this point: Interactive conversations added, context maintenance working
4. Add User Story 3 → Test independently → Deploy/Demo
   - At this point: Full citation verification added, complete agent functionality

### Parallel Team Strategy

With multiple developers after Foundational phase:

1. Team completes Setup + Foundational together (T001-T011)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T012-T021) - Priority 1 MVP
   - **Developer B**: User Story 2 (T022-T028) - Can start in parallel but needs US1 functions
   - **Developer C**: User Story 3 (T029-T033) - Can start in parallel but needs US1 functions
3. Merge all functions into backend/agent.py (careful with conflicts in shared functions)
4. All developers: Polish (T034-T038) in parallel

---

## Notes

- **Single-file architecture**: All functions go in backend/agent.py
- **Reuses existing infrastructure**: Cohere client, Qdrant client, .env configuration from features 007/008
- **No new dependencies except OpenAI SDK**: Keeps implementation simple
- Each user story should be independently testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **MVP is User Story 1**: Single-question RAG agent with grounded answers

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 6 tasks (BLOCKING - critical path)
- **Phase 3 (US1 - MVP)**: 10 tasks
- **Phase 4 (US2)**: 7 tasks
- **Phase 5 (US3)**: 5 tasks
- **Phase 6 (Polish)**: 5 tasks

**Total**: 38 tasks

**Parallel Opportunities**: 5 tasks marked [P] can run in parallel

**Critical Path**: Phase 1 (5 tasks) → Phase 2 (6 tasks) → Phase 3 (10 tasks) → Phase 6 (5 tasks) = **26 tasks for MVP**
