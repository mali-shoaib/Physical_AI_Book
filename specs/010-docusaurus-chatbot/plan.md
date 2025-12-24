# Implementation Plan: Frontend-Backend Integration of RAG Chatbot

**Branch**: `010-docusaurus-chatbot` | **Date**: 2025-12-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/010-docusaurus-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a FastAPI backend that wraps the existing RAG agent (feature 009) and exposes REST API endpoints, then build a React chatbot widget that embeds into the Docusaurus theme to provide interactive Q&A for textbook readers. The backend acts as a thin HTTP wrapper around agent.py, while the frontend provides a persistent chatbot UI accessible from all pages.

## Technical Context

**Language/Version**: Python 3.10+ (backend), JavaScript/React 18+ (frontend)
**Primary Dependencies**: FastAPI (>=0.104.0), React (18+), uvicorn (>=0.24.0), axios (for frontend HTTP)
**Storage**: In-memory conversation state (browser session storage for frontend, Python dict for backend)
**Testing**: Manual end-to-end testing (quickstart validation), pytest for backend unit tests (optional)
**Target Platform**: Local development environment (localhost:8000 for backend, localhost:3000 for Docusaurus)
**Project Type**: Web application (backend Python + frontend React)
**Performance Goals**: <2s backend processing, <10s end-to-end query response (p95)
**Constraints**: Local-only integration, no persistence, no authentication, CORS must allow localhost:3000
**Scale/Scope**: Single-user development setup, 1 FastAPI file, 1 React component, 2-3 endpoints

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Grounded Answers Only ✅ PASS

- ✅ FastAPI backend delegates to agent.py from feature 009 which enforces grounding
- ✅ Agent returns "Information not found in the book" for low-similarity results
- ✅ No direct LLM calls in FastAPI - all requests go through agent.py
- ✅ Frontend displays citations from backend responses

**Implementation**: FastAPI simply calls agent.py functions which already enforce grounding constraints

### Principle II: Cohere Embeddings Standard ✅ PASS

- ✅ Backend reuses agent.py which uses Cohere embed-english-v3.0
- ✅ No new embedding generation in this feature
- ✅ Frontend has no embedding logic

**Implementation**: No changes to embedding layer - reuses existing agent.py infrastructure

### Principle III: Qdrant Vector Storage ✅ PASS

- ✅ Backend calls agent.py retrieve_context() which queries Qdrant
- ✅ No new Qdrant access patterns introduced
- ✅ Metadata preserved through agent.py response structure

**Implementation**: Backend passes through Qdrant results from agent.py

### Principle IV: Content Extraction Pipeline ✅ N/A

- N/A: This feature consumes existing pipeline outputs
- No content extraction or ingestion in this feature
- Feature 007 handles all content processing

**Rationale**: Integration layer only - no content pipeline changes

### Principle V: Agent-Based Retrieval ✅ PASS

- ✅ FastAPI backend exposes agent.py functionality via HTTP endpoints
- ✅ Implements POST `/api/chat/query` endpoint wrapping agent functions
- ✅ Agent grounding enforced by existing agent.py implementation

**Implementation**: FastAPI provides HTTP wrapper around existing agent.py

### Principle VI: Docusaurus Chatbot Integration ✅ PASS

- ✅ React chatbot component embedded in Docusaurus theme
- ✅ Component makes API calls to FastAPI backend
- ✅ Citations displayed in chat UI
- ✅ Session-based conversation history (browser sessionStorage)

**Implementation**: Custom React component integrated via Docusaurus theme swizzling

### Summary

**Status**: ✅ PASS - All principles satisfied

- **No deviations**: Feature strictly integrates existing components without violating constitution
- **Compliance**: Wraps feature 009 agent.py with HTTP layer, maintains all grounding/embedding/storage principles
- **Integration**: Provides user-facing interface while preserving all backend constraints

## Project Structure

### Documentation (this feature)

```text
specs/010-docusaurus-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── api-schema.yaml  # OpenAPI spec for FastAPI endpoints
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── api.py               # NEW: FastAPI application with chat endpoints
├── agent.py             # Existing: RAG agent from feature 009
├── main.py              # Existing: Embedding ingestion (feature 007)
├── retrieve.py          # Existing: Retrieval validation (feature 008)
├── .env                 # Shared: API keys (no new keys needed)
├── pyproject.toml       # Updated: Add fastapi, uvicorn dependencies
└── README.md            # Updated: Add API usage section

src/
└── theme/              # NEW: Docusaurus theme customization
    └── ChatbotWidget/  # NEW: React chatbot component
        ├── index.tsx   # Component implementation
        ├── styles.module.css  # Component styles
        └── types.ts    # TypeScript interfaces

docs/                   # Existing: Docusaurus content
└── (no changes)        # Chatbot widget auto-appears on all pages
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus React). Backend adds single file (api.py) to existing backend/ directory. Frontend adds custom theme component to Docusaurus. No new directories needed - integrates into existing structure.

## Complexity Tracking

No constitution violations - no justifications needed.

---

## Phase 0: Research Complete ✅

**Artifacts Created**:
- `research.md` - 7 research questions with decisions, rationale, and alternatives

**Key Decisions Made**:
1. FastAPI backend wraps existing agent.py with minimal transformation
2. Single `/api/chat/query` endpoint handles all chat interactions
3. Frontend uses Docusaurus theme swizzling for chatbot component
4. Conversation state managed in browser sessionStorage (frontend) and Python dict (backend)
5. CORS configured for localhost:3000 in FastAPI
6. Error handling returns JSON with appropriate HTTP status codes
7. Citations passed through from agent.py response structure

**Dependencies Added**: `fastapi>=0.104.0`, `uvicorn>=0.24.0`, `python-multipart` (for FastAPI file uploads if needed later)

---

## Phase 1: Design Complete ✅

**Artifacts Created**:
- `data-model.md` - 6 entities with validation rules and relationships
- `contracts/api-schema.yaml` - OpenAPI specification for FastAPI endpoints
- `quickstart.md` - Setup instructions and usage examples

**Entities Defined**:
1. **ChatRequest** - HTTP request payload (query, conversation_id, context)
2. **ChatResponse** - HTTP response payload (answer, citations, conversation_id, metadata)
3. **Citation** - Source reference in response (module_name, chapter_id, source_url, similarity_score)
4. **ConversationState** - Backend in-memory state (conversation_id, messages, last_updated)
5. **ChatMessage** - UI message object (role, content, timestamp, citations)
6. **ChatWidgetState** - Frontend React state (messages, isOpen, isLoading, error)

**Contracts Specified**:
- REST API: POST /api/chat/query, GET /api/health, POST /api/chat/reset
- Request/response schemas with JSON examples
- Error response formats
- CORS configuration
- React component props and events

---

## Next Steps (Implementation - `/sp.tasks`)

1. Run `/sp.tasks` to generate task breakdown
2. Implement `backend/api.py` (FastAPI wrapper around agent.py)
3. Add `fastapi` and `uvicorn` to `backend/pyproject.toml`
4. Create `src/theme/ChatbotWidget/index.tsx` (React component)
5. Test local integration (backend running on :8000, frontend on :3000)
6. Verify Constitution compliance (grounding, citations, error handling)
