# Tasks: Frontend-Backend Integration of RAG Chatbot

**Input**: Design documents from `/specs/010-docusaurus-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-schema.yaml, quickstart.md

**Tests**: No test tasks included (not requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This feature creates a FastAPI backend and React frontend component:
- Backend API: `backend/api.py` (new file wrapping agent.py)
- Frontend widget: `src/theme/ChatbotWidget/` (new directory with React component)
- Configuration: Reuses `backend/.env` from features 007/008/009
- Dependencies: Update `backend/pyproject.toml` and `package.json`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare project structure and dependencies for backend and frontend integration

- [X] T001 Verify backend virtual environment exists (backend/.venv/) and is activated
- [X] T002 Add FastAPI dependencies to backend/pyproject.toml (fastapi>=0.104.0, uvicorn>=0.24.0, python-multipart>=0.0.6)
- [X] T003 Install FastAPI packages using UV or pip in backend/.venv
- [X] T004 Verify frontend dependencies in package.json (axios for HTTP requests)
- [X] T005 Install axios in Docusaurus project (npm install axios)
- [X] T006 Create backend/api.py skeleton with imports (fastapi, uvicorn, dotenv, typing, uuid, datetime)
- [X] T007 Create src/theme/ChatbotWidget/ directory for React component

**Checkpoint**: Dependencies installed, project structure ready for implementation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core backend API structure and health endpoints that ALL user stories depend on

**CRITICAL**: No user story work can begin until this phase is complete

- [X] T008 Implement environment variable loading in backend/api.py - load .env file, reuse OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY from feature 009
- [X] T009 Create FastAPI application instance in backend/api.py - app = FastAPI(title="RAG Chatbot API", version="1.0.0")
- [X] T010 Configure CORS middleware in backend/api.py - allow origins ["http://localhost:3000", "http://127.0.0.1:3000"], methods ["GET", "POST", "OPTIONS"], headers ["Content-Type"]
- [X] T011 Define Pydantic models in backend/api.py - ChatRequest (query: str, conversation_id: str | None, context: str | None), ChatResponse (answer: str, citations: List[Citation], conversation_id: str, metadata: dict), Citation (module_name: str, chapter_id: str, source_url: str, similarity_score: float)
- [X] T012 Implement conversation state management in backend/api.py - create conversations dict (in-memory storage), add get_conversation(), create_conversation(), add_message() helper functions
- [X] T013 [P] Implement GET /api/health endpoint in backend/api.py - return {"status": "healthy", "timestamp": datetime.now().isoformat()}
- [X] T014 Import agent.py functions in backend/api.py - from agent import retrieve_context, embed_query (reuse existing feature 009 functions)

**Checkpoint**: Foundation ready - FastAPI app configured, CORS enabled, health endpoint working, agent.py imported

---

## Phase 3: User Story 1 - Interactive Chatbot Query (Priority: P1) MVP

**Goal**: Basic chatbot that accepts queries, retrieves context from Qdrant, and returns grounded answers with citations

**Independent Test**: Start backend (`uvicorn backend.api:app`), send POST to /api/chat/query with query "What is ROS 2?", verify response contains grounded answer with citations and conversation_id

**Acceptance Criteria**:
- Backend accepts POST /api/chat/query with query field
- Backend calls agent.py retrieve_context() to get relevant chunks
- Backend generates answer based on retrieved context (grounded)
- Response includes citations with module_name, chapter_id, source_url, similarity_score
- Out-of-scope queries return "I don't have information about that in the textbook"
- Frontend displays chatbot widget with toggle button
- Frontend sends query to backend and displays response

### Implementation for User Story 1

#### Backend Tasks (US1)

- [X] T015 [US1] Implement POST /api/chat/query endpoint skeleton in backend/api.py - accept ChatRequest payload, validate query field (non-empty, max 500 chars), return 400 if validation fails
- [X] T016 [US1] Implement conversation_id generation in backend/api.py - generate UUID v4 if conversation_id is null/missing, create ConversationState entry in conversations dict
- [X] T017 [US1] Implement context retrieval logic in backend/api.py - call retrieve_context(query, top_k=5, threshold=0.70), handle empty results (no relevant content found)
- [X] T018 [US1] Implement answer generation wrapper in backend/api.py - format retrieved chunks for OpenAI agent, call OpenAI API with system prompt for grounding, extract answer text
- [X] T019 [US1] Implement citation formatting in backend/api.py - convert retrieve_context() results to Citation objects, include module_name, chapter_id, source_url, similarity_score, sort by similarity descending
- [X] T020 [US1] Implement response construction in backend/api.py - create ChatResponse with answer, citations, conversation_id, metadata (retrieval_count, tokens_used, grounded flag), return JSON with 200 status
- [X] T021 [US1] Add error handling in backend/api.py - catch Qdrant errors (503), OpenAI errors (503), validation errors (400), log all errors with loguru, return user-friendly error messages
- [X] T022 [US1] Add request logging in backend/api.py - log every /api/chat/query request with query text (first 50 chars), conversation_id, timestamp, response status

#### Frontend Tasks (US1)

- [X] T023 [US1] [P] Create TypeScript types in src/theme/ChatbotWidget/types.ts - define ChatMessage, Citation, ChatWidgetState, ChatRequest, ChatResponse interfaces
- [X] T024 [US1] Implement ChatbotWidget component skeleton in src/theme/ChatbotWidget/index.tsx - create functional component with useState hooks for messages, isOpen, isLoading, error, conversationId
- [X] T025 [US1] Implement toggle button in src/theme/ChatbotWidget/index.tsx - add button to open/close widget, update isOpen state on click, position fixed bottom-right corner
- [X] T026 [US1] Implement message list UI in src/theme/ChatbotWidget/index.tsx - render messages array, display user messages right-aligned (blue), assistant messages left-aligned (gray), show timestamp
- [X] T027 [US1] Implement input field and submit in src/theme/ChatbotWidget/index.tsx - add textarea for user input, submit button, handle Enter key, disable input when isLoading=true
- [X] T028 [US1] Implement API call function in src/theme/ChatbotWidget/index.tsx - create handleSubmitMessage() that sends POST to http://localhost:8000/api/chat/query with axios, include query and conversation_id in payload
- [X] T029 [US1] Implement response handling in src/theme/ChatbotWidget/index.tsx - parse ChatResponse JSON, create assistant ChatMessage with answer and citations, append to messages array, update conversationId state
- [X] T030 [US1] Implement error handling in src/theme/ChatbotWidget/index.tsx - catch network errors, 4xx/5xx responses, timeout errors, display user-friendly error message, set error state
- [X] T031 [US1] Implement loading state in src/theme/ChatbotWidget/index.tsx - show loading spinner when isLoading=true, disable input during loading, clear loading after response or error
- [X] T032 [US1] Implement citation display in src/theme/ChatbotWidget/index.tsx - render citations below assistant messages, show module_name and chapter_id, make source_url clickable link, display similarity_score as percentage
- [X] T033 [US1] [P] Create styles in src/theme/ChatbotWidget/styles.module.css - style widget container (fixed bottom-right), toggle button, message bubbles, input field, citations block, loading spinner, use Docusaurus Infima CSS variables
- [X] T034 [US1] Integrate ChatbotWidget into Docusaurus Layout - create or modify src/theme/Layout/index.js to import and render ChatbotWidget component at bottom of every page

**Checkpoint**: User Story 1 complete - basic chatbot working, retrieves from Qdrant via backend, displays answers with citations in frontend

---

## Phase 4: User Story 2 - Context-Aware Follow-up Questions (Priority: P2)

**Goal**: Multi-turn conversations where backend maintains context and agent understands pronoun references

**Independent Test**: Open chatbot, ask "What is URDF?", then ask "How do I create one?", verify second question is understood as "How do I create a URDF?" and returns relevant content

**Acceptance Criteria**:
- Backend maintains conversation history in ConversationState
- Backend passes full message history to agent for context
- Agent understands pronoun references ("it", "that", "one")
- Frontend sends conversation_id with every request after first
- User can reset conversation and start fresh

### Implementation for User Story 2

#### Backend Tasks (US2)

- [ ] T035 [US2] Implement message history storage in backend/api.py - update add_message() to append user and assistant messages to ConversationState.messages with role, content, timestamp
- [ ] T036 [US2] Implement conversation context passing in backend/api.py - extract full message history from ConversationState, format as OpenAI messages array [{"role": "user", "content": "..."}, {"role": "assistant", "content": "..."}]
- [ ] T037 [US2] Update answer generation in backend/api.py - pass conversation history to OpenAI API call, include system prompt with grounding instructions, handle multi-turn context
- [ ] T038 [US2] Implement POST /api/chat/reset endpoint in backend/api.py - accept conversation_id in payload, delete ConversationState entry from conversations dict, return 200 success
- [ ] T039 [US2] Add conversation history size limit in backend/api.py - limit messages list to last 50 messages (or ~8000 tokens), implement sliding window to remove oldest messages when limit exceeded

#### Frontend Tasks (US2)

- [ ] T040 [US2] Implement sessionStorage persistence in src/theme/ChatbotWidget/index.tsx - save conversationId to sessionStorage on every update, load from sessionStorage on component mount
- [ ] T041 [US2] Update API call in src/theme/ChatbotWidget/index.tsx - always include conversationId in request payload (from state or sessionStorage), handle new conversation_id in response
- [ ] T042 [US2] Implement reset button in src/theme/ChatbotWidget/index.tsx - add "Reset Conversation" button, call /api/chat/reset endpoint on click, clear messages array, set conversationId=null, clear sessionStorage
- [ ] T043 [US2] Add confirmation dialog for reset in src/theme/ChatbotWidget/index.tsx - show "Are you sure?" dialog before resetting, prevent accidental conversation loss

**Checkpoint**: User Story 2 complete - multi-turn conversations working, context maintained across questions, reset functionality added

---

## Phase 5: User Story 3 - Contextual Query from Selected Text (Priority: P3)

**Goal**: Users can highlight text on page and pass it as context to chatbot for focused questions

**Independent Test**: Highlight a paragraph about "VSLAM" on a page, trigger chatbot with selected text, ask "Explain this in simpler terms", verify backend receives context and uses it in answer

**Acceptance Criteria**:
- Frontend detects text selection on page
- User can trigger chatbot with selected text as context
- Backend accepts optional context field in ChatRequest
- Backend prioritizes provided context along with retrieved chunks
- Frontend displays selected text in chatbot as context indicator

### Implementation for User Story 3

#### Frontend Tasks (US3)

- [ ] T044 [US3] Implement text selection detection in src/theme/ChatbotWidget/index.tsx - add event listener for text selection (window.getSelection()), detect when user highlights text on page (selection.toString().length > 0)
- [ ] T045 [US3] Implement context menu or button for "Ask about this" in src/theme/ChatbotWidget/index.tsx - show button near selected text or in context menu, onClick opens chatbot with selected text as context
- [ ] T046 [US3] Update API call to include context in src/theme/ChatbotWidget/index.tsx - add context field to ChatRequest payload, pass selected text (max 2000 chars), truncate if longer with warning
- [ ] T047 [US3] Display context indicator in src/theme/ChatbotWidget/index.tsx - show selected text snippet above input field when context is provided, allow user to clear context before submitting

#### Backend Tasks (US3)

- [ ] T048 [US3] Implement context field validation in backend/api.py - validate context field in ChatRequest (max 2000 chars), trim whitespace, truncate with warning if exceeded
- [ ] T049 [US3] Update retrieval logic to incorporate context in backend/api.py - prepend provided context to query for embedding generation, combine context with retrieved chunks, prioritize provided context in answer generation
- [ ] T050 [US3] Update answer generation to use context in backend/api.py - pass both selected context and retrieved chunks to OpenAI agent, include instruction to prioritize provided context, generate focused answer

**Checkpoint**: User Story 3 complete - text selection working, context passed to backend, focused answers based on selected text

---

## Phase 6: Polish & Documentation

**Purpose**: Final touches, documentation, error handling improvements, and integration testing

- [ ] T051 [P] Update backend/README.md with API documentation - add "RAG Chatbot API" section, document POST /api/chat/query endpoint (request/response schemas), document POST /api/chat/reset endpoint, document GET /api/health endpoint
- [ ] T052 [P] Add API usage examples to backend/README.md - curl examples for /api/chat/query, JavaScript fetch examples, error response examples, CORS configuration notes
- [ ] T053 [P] Create quickstart guide in specs/010-docusaurus-chatbot/quickstart.md - step-by-step setup instructions (backend + frontend), environment variable configuration, running backend (`uvicorn backend.api:app --reload`), running frontend (`npm start`), troubleshooting common issues
- [ ] T054 Add comprehensive docstrings to all functions in backend/api.py - include function purpose, parameters with types, return type, raises exceptions, examples
- [ ] T055 Add type hints throughout backend/api.py - ensure all function signatures have proper type annotations (str, int, float, List, Dict, Optional, etc.)
- [ ] T056 [P] Add loading timeout handling in src/theme/ChatbotWidget/index.tsx - set 30-second timeout for API requests, show "Request timed out. Please try again." error message, allow retry
- [ ] T057 [P] Add rate limiting warning in src/theme/ChatbotWidget/index.tsx - detect 429 Too Many Requests response, display "Too many requests. Please wait before trying again." message
- [ ] T058 Improve error messages in backend/api.py - return specific error codes (BAD_REQUEST, QUERY_TOO_LONG, AGENT_ERROR, INTERNAL_ERROR), include user-friendly messages, hide sensitive details from error responses
- [ ] T059 [P] Add responsive design to src/theme/ChatbotWidget/styles.module.css - make widget mobile-friendly (full-width on small screens), adjust font sizes for mobile, ensure touch-friendly button sizes (min 44px)
- [ ] T060 [P] Add accessibility improvements to src/theme/ChatbotWidget/index.tsx - add ARIA labels for buttons and inputs, ensure keyboard navigation works (Tab, Enter, Escape), add focus indicators, ensure screen reader compatibility

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

- **User Story 1 (P1 - MVP)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Depends on US1 backend endpoint and frontend component - Extends conversation functionality
- **User Story 3 (P3)**: Depends on US1 backend endpoint and frontend component - Adds context parameter support

### Within Each User Story

- User Story 1 Backend: Sequential (T015 → T016 → T017 → T018 → T019 → T020 → T021 → T022)
- User Story 1 Frontend: T023 can run first, then T024-T034 mostly sequential (component building)
- User Story 2 Backend: Sequential (T035 → T036 → T037 → T038 → T039)
- User Story 2 Frontend: Sequential (T040 → T041 → T042 → T043)
- User Story 3 Frontend: Sequential (T044 → T045 → T046 → T047)
- User Story 3 Backend: Sequential (T048 → T049 → T050)

### Parallel Opportunities

**Phase 1 (Setup)**:
- All tasks are sequential (environment setup)

**Phase 2 (Foundational)**:
- T008-T012 must run sequentially (API structure)
- T013 can run in parallel with T014 (health endpoint independent)

**Phase 3 (US1)**:
- Backend tasks (T015-T022) can run independently from frontend tasks (T023-T034)
- Within frontend: T023 [P] and T033 [P] can run early in parallel
- T034 (integration) must wait for component completion

**Phase 6 (Polish)**:
- T051, T052, T053, T059, T060 can run in parallel (different files, documentation)
- T054, T055 should be done after code is complete
- T056, T057, T058 can run in parallel (independent features)

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T014) - CRITICAL
3. Complete Phase 3: User Story 1 Backend (T015-T022)
4. Complete Phase 3: User Story 1 Frontend (T023-T034)
5. **STOP and VALIDATE**:
   - Start backend: `uvicorn backend.api:app --reload`
   - Start frontend: `npm start`
   - Open chatbot widget in browser
   - Ask "What is ROS 2?" and verify grounded answer with citations
6. Test with queries from feature 008/009: "What is Isaac Sim?", "How do I create URDF?"
7. Verify: Backend responds, frontend displays answer, citations are clickable, error handling works
8. **This is the MVP!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → **Deploy/Demo (MVP!)**
   - At this point: Single-query chatbot working, RAG functionality proven, frontend-backend integration complete
3. Add User Story 2 → Test independently → Deploy/Demo
   - At this point: Multi-turn conversations added, context maintenance working
4. Add User Story 3 → Test independently → Deploy/Demo
   - At this point: Text selection context added, complete chatbot functionality
5. Complete Polish → Final documentation and error handling improvements

### Parallel Team Strategy

With multiple developers after Foundational phase:

1. Team completes Setup + Foundational together (T001-T014)
2. Once Foundational is done:
   - **Developer A (Backend)**: User Story 1 Backend (T015-T022) - Priority 1 MVP
   - **Developer B (Frontend)**: User Story 1 Frontend (T023-T034) - Priority 1 MVP
   - Coordinate on API contract (ChatRequest/ChatResponse structure)
3. After US1 complete:
   - **Developer A**: User Story 2 Backend (T035-T039) + User Story 3 Backend (T048-T050)
   - **Developer B**: User Story 2 Frontend (T040-T043) + User Story 3 Frontend (T044-T047)
4. All developers: Polish (T051-T060) in parallel

---

## Dependency Graph

```
Phase 1 (Setup)
  T001 → T002 → T003
  T004 → T005
  T006, T007 (after T003, T005)
    ↓
Phase 2 (Foundational) - BLOCKING
  T008 → T009 → T010 → T011 → T012
  T013 [P], T014 [P] (parallel after T009)
    ↓
Phase 3 (US1 - MVP) - Can split into Backend + Frontend tracks
  Backend Track:
    T015 → T016 → T017 → T018 → T019 → T020 → T021 → T022
  Frontend Track (can run in parallel with Backend):
    T023 [P] (can start early)
    T024 → T025 → T026 → T027 → T028 → T029 → T030 → T031 → T032
    T033 [P] (can run early in parallel)
    T034 (waits for T024-T032 complete)
    ↓
Phase 4 (US2)
  Backend: T035 → T036 → T037 → T038 → T039
  Frontend: T040 → T041 → T042 → T043
    ↓
Phase 5 (US3)
  Frontend: T044 → T045 → T046 → T047
  Backend: T048 → T049 → T050
    ↓
Phase 6 (Polish) - Many parallel opportunities
  Documentation [P]: T051, T052, T053
  Code quality: T054 → T055
  Features [P]: T056, T057, T058, T059, T060
```

---

## MVP Scope (Minimum Viable Product)

**MVP = Phase 1 + Phase 2 + Phase 3 (User Story 1)**

**Total MVP Tasks**: 34 tasks
- Phase 1: 7 tasks
- Phase 2: 7 tasks
- Phase 3: 20 tasks (12 backend + 8 frontend)

**Critical Path for MVP**: T001 → ... → T014 (Foundational) → T015 → ... → T034 (US1)

**MVP delivers**:
- Working FastAPI backend wrapping agent.py
- React chatbot widget in Docusaurus
- Basic query → answer with citations flow
- Error handling and loading states
- CORS configured for local development
- Health endpoint for monitoring

**Post-MVP enhancements**:
- Phase 4 (US2): Multi-turn conversations (7 tasks)
- Phase 5 (US3): Text selection context (7 tasks)
- Phase 6: Polish and documentation (10 tasks)

---

## Independent Test Criteria

### User Story 1 (MVP) - Interactive Chatbot Query

**Test Steps**:
1. Start backend: `cd backend && uvicorn api:app --reload`
2. Verify health endpoint: `curl http://localhost:8000/api/health`
3. Start frontend: `npm start` (opens http://localhost:3000)
4. Open chatbot widget (click toggle button)
5. Ask: "What is ROS 2?"
6. Verify:
   - Loading spinner appears while waiting
   - Answer appears within 10 seconds
   - Answer is grounded (references ROS 2 content)
   - Citations displayed below answer
   - Citations are clickable links to textbook pages
   - No console errors
7. Ask out-of-scope question: "What's the weather?"
8. Verify: "I don't have information about that in the textbook" message

**Success Criteria**: All verifications pass, chatbot is functional

### User Story 2 - Context-Aware Follow-up Questions

**Test Steps**:
1. Complete US1 test first (chatbot working)
2. Ask: "What is URDF?"
3. Verify answer about URDF appears
4. Ask follow-up: "How do I create one?"
5. Verify:
   - Answer is about creating URDF (not generic "one")
   - Agent understood "one" refers to URDF from previous context
   - Citations still appear
6. Click "Reset Conversation" button
7. Ask: "How do I create one?"
8. Verify: Answer is confused or asks for clarification (no context)

**Success Criteria**: Multi-turn context maintained, reset clears context

### User Story 3 - Contextual Query from Selected Text

**Test Steps**:
1. Complete US1 test first (chatbot working)
2. Navigate to a chapter about VSLAM
3. Highlight a paragraph about VSLAM
4. Click "Ask about this" button (or similar trigger)
5. Verify:
   - Chatbot opens
   - Selected text shown as context
6. Ask: "Explain this in simpler terms"
7. Verify:
   - Answer is specific to selected text
   - Citations may include the current page
8. Clear context and ask same question
9. Verify: Answer is more general (no selected context)

**Success Criteria**: Selected text passed as context, answer is focused

---

## Notes

- **Backend wraps agent.py**: All retrieval and answer generation delegated to existing feature 009 code
- **Frontend uses Docusaurus theme**: Swizzling approach for integration, Infima CSS for styling
- **No persistence**: Conversation state in-memory only, cleared on backend restart
- **Local development only**: CORS configured for localhost, no production deployment
- **Single API endpoint**: POST /api/chat/query handles all chat interactions (US1, US2, US3)
- Each user story should be independently testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **MVP is User Story 1**: Basic chatbot with grounded answers and citations

---

## Task Count Summary

- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 7 tasks (BLOCKING - critical path)
- **Phase 3 (US1 - MVP)**: 20 tasks (12 backend + 8 frontend)
- **Phase 4 (US2)**: 7 tasks (5 backend + 2 frontend)
- **Phase 5 (US3)**: 7 tasks (4 frontend + 3 backend)
- **Phase 6 (Polish)**: 10 tasks

**Total**: 60 tasks

**Parallel Opportunities**: 10 tasks marked [P] can run in parallel

**Critical Path**: Phase 1 (7 tasks) → Phase 2 (7 tasks) → Phase 3 (20 tasks) → Phase 6 (10 tasks) = **44 tasks for MVP + Polish**

**User Story Distribution**:
- US1 (P1 - MVP): 20 tasks
- US2 (P2): 7 tasks
- US3 (P3): 7 tasks
- Infrastructure (Phase 1+2): 14 tasks
- Polish: 10 tasks
