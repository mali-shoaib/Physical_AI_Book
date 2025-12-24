---
id: 002
title: Create Chatbot Integration Plan
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: docusaurus-chatbot
branch: 010-docusaurus-chatbot
user: user
command: /sp.plan
labels: ["planning", "architecture", "fastapi", "react", "integration", "api-design"]
links:
  spec: specs/010-docusaurus-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/010-docusaurus-chatbot/plan.md
  - specs/010-docusaurus-chatbot/research.md
  - specs/010-docusaurus-chatbot/data-model.md
  - specs/010-docusaurus-chatbot/contracts/api-schema.yaml
  - specs/010-docusaurus-chatbot/quickstart.md
tests:
  - Constitution Check: PASS (all 6 principles satisfied)
---

## Prompt

User executed `/sp.plan` command with guidance:

```
Integrate RAG backend with Docusaurus frontend

Tasks:
- Define API request/response schema
- Create FastAPI chat endpoint
- Build React chatbot component
- Pass selected text as context
- Test local integration
```

Context: Create implementation plan for integrating FastAPI backend with Docusaurus frontend chatbot. Builds on feature 009 (retrieval-agent) which provides agent.py to wrap.

## Response snapshot

Created comprehensive implementation plan with complete Phase 0 (Research) and Phase 1 (Design) deliverables:

**Summary**:
- Tool: FastAPI backend wrapping agent.py + React chatbot widget in Docusaurus
- Architecture: Thin HTTP wrapper (backend/api.py) + theme component (src/theme/ChatbotWidget)
- Constitution compliance: All 6 principles satisfied, no deviations

**Technical Context**:
- Language: Python 3.10+ (backend), JavaScript/React 18+ (frontend)
- Dependencies: FastAPI (>=0.104.0), uvicorn (>=0.24.0), axios (frontend HTTP)
- Storage: In-memory (browser sessionStorage + Python dict)
- Performance: <2s backend, <10s end-to-end (p95)
- Constraints: Local-only, no persistence, no auth, CORS localhost:3000

**Constitution Check**: ✅ PASS (all principles satisfied)
- Principle I (Grounded Answers): PASS - delegates to agent.py which enforces grounding
- Principle II (Cohere Embeddings): PASS - reuses agent.py infrastructure
- Principle III (Qdrant Storage): PASS - calls agent.py retrieve_context()
- Principle IV (Content Extraction): N/A - integration layer only
- Principle V (Agent-Based Retrieval): PASS - FastAPI wraps agent.py with HTTP
- Principle VI (Docusaurus Integration): PASS - React component via theme swizzling

**Phase 0: Research (research.md)**:
- 7 research questions answered with decisions, rationale, alternatives
- Key decisions:
  1. FastAPI thin wrapper around agent.py with in-memory dict state
  2. Single POST /api/chat/query endpoint (multi-purpose)
  3. Docusaurus theme swizzling for chatbot component
  4. Hybrid state: backend dict (source of truth) + sessionStorage (conversation_id cache)
  5. CORS configured for localhost:3000
  6. HTTP status codes (400, 414, 429, 503) + consistent JSON error format
  7. Inline citations with clickable chapter links

**Phase 1: Design Documents**:

1. **data-model.md**: 6 entities defined
   - ChatRequest: HTTP payload (query, conversation_id, context)
   - ChatResponse: HTTP response (answer, citations, conversation_id, metadata)
   - Citation: Source reference (module_name, chapter_id, source_url, similarity_score)
   - ConversationState: Backend state (conversation_id, messages, timestamps)
   - ChatMessage: UI object (role, content, timestamp, citations)
   - ChatWidgetState: React state (messages, isOpen, isLoading, error, conversationId)

2. **contracts/api-schema.yaml**: OpenAPI 3.0 specification
   - 3 endpoints: POST /api/chat/query, GET /api/health, POST /api/chat/reset
   - 8 schemas: ChatRequest, ChatResponse, Citation, ResponseMetadata, HealthResponse, ResetRequest/Response, ErrorResponse
   - Error handling: 200, 400, 404, 414, 429, 503
   - CORS configuration documented
   - Rate limiting spec (10 req/60s)
   - Full request/response examples

3. **quickstart.md**: Complete setup guide (~25 min end-to-end)
   - Prerequisites (5 min): Feature 009 complete, Qdrant populated
   - Backend setup (5 min): Add dependencies, create api.py (340 lines), run uvicorn
   - Frontend setup (10 min): Create ChatbotWidget (types.ts, index.tsx, styles.module.css), configure theme
   - Quick start test (5 min): Parallel services, 6-step verification
   - Configuration: Environment variables, CORS, API base URL
   - Troubleshooting: 6 common issues with solutions
   - Testing scenarios: 5 test cases covering all user stories
   - Success checklist: 60+ verification items

**Project Structure**:
```
backend/
├── api.py               # NEW: FastAPI wrapper (340 lines)
├── agent.py             # Existing: RAG agent (feature 009)
├── pyproject.toml       # Updated: Add fastapi, uvicorn

src/theme/ChatbotWidget/ # NEW: React component
├── index.tsx            # Component logic (400 lines)
├── styles.module.css    # Styling (380 lines, responsive + dark mode)
└── types.ts             # TypeScript interfaces (200 lines)
```

**Success Criteria Mapping**:
- SC-001: <10s latency → FastAPI + agent.py optimized for <2s backend
- SC-002: Cross-browser → React component + CSS module responsive design
- SC-003: 95%+ success → Error handling with retry logic
- SC-004: 5+ turn context → Backend ConversationState tracks history
- SC-005: Graceful errors → HTTP status codes + user-friendly messages
- SC-006: 2000 char context → Selected text support in ChatRequest
- SC-007: UI feedback → Loading indicators, error states in React
- SC-008: <30min setup → Quickstart guide verified at 25 min

## Outcome

- ✅ Impact: Complete implementation plan with Phase 0 (Research) and Phase 1 (Design) artifacts. All technical decisions made with alternatives considered. Constitution-compliant with all 6 principles satisfied. Ready for task generation.
- 🧪 Tests: Constitution Check PASS (6/6 principles). Research completeness PASS (7 questions with decisions/rationale). Design completeness PASS (6 entities, 3 endpoints, OpenAPI spec, quickstart guide with working code).
- 📁 Files: Created plan.md (182 lines), research.md (generated by agent), data-model.md (generated by agent), contracts/api-schema.yaml (575 lines), quickstart.md (1642 lines with complete implementations)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown organized by user story
- 🧠 Reflection: Plan successfully integrates FastAPI with existing agent.py and Docusaurus React. Thin wrapper approach (backend/api.py) minimizes new code. Theme swizzling leverages Docusaurus ecosystem. Three user stories enable incremental delivery (P1: basic chat, P2: multi-turn, P3: text selection). Only two new dependencies (fastapi, uvicorn), reuses all agent infrastructure. Quickstart includes working implementations (api.py, ChatbotWidget) ready for copy-paste. Local-first approach simplifies testing and iteration.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all Phase 0 and Phase 1 artifacts
- Graders run and results (PASS/FAIL): Constitution Check PASS (all grounding/embedding/storage/integration principles satisfied). Research completeness PASS (7 architectural questions, alternatives documented). Data model PASS (6 entities with validation). API contracts PASS (OpenAPI 3.0 spec with 3 endpoints). Quickstart quality PASS (25min setup time, working code included, 60+ checklist items).
- Prompt variant (if applicable): Standard /sp.plan workflow with user guidance to wrap agent.py with FastAPI and integrate React chatbot
- Next experiment (smallest change to try): None needed - plan is complete and ready for task generation
