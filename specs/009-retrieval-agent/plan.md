# Implementation Plan: Retrieval-Enabled Agent

**Branch**: `009-retrieval-agent` | **Date**: 2025-12-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/009-retrieval-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a command-line RAG (Retrieval-Augmented Generation) agent using OpenAI Agents SDK that retrieves information from Qdrant vector database and generates grounded answers strictly based on the Physical AI textbook content. The agent will integrate with existing embedding infrastructure (features 007/008), use Cohere for query embeddings, and support both single-question and interactive conversation modes.

## Technical Context

**Language/Version**: Python 3.10+ (matching existing backend pipeline)
**Primary Dependencies**: OpenAI Python SDK (>=1.0.0), Cohere Python SDK (>=5.0.0), Qdrant client (>=1.7.0), Loguru (>=0.7.0)
**Storage**: Qdrant vector database (read-only access to "rag_embedding" collection from feature 007)
**Testing**: Manual testing with test queries from feature 008 validation tool
**Target Platform**: Local command-line (Windows/Linux/macOS), Python virtual environment
**Project Type**: Single-file CLI tool (backend/agent.py)
**Performance Goals**: <2s retrieval latency, <10s end-to-end response time (retrieval + generation)
**Constraints**: Must use Cohere embed-english-v3.0 for query embeddings, must query existing Qdrant collection, in-memory conversation context only, no FastAPI/web server
**Scale/Scope**: Single-user local tool, processes one question at a time, maintains conversation history for current session only

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Grounded Answers Only ✅ PASS

- ✅ Agent configured to answer strictly from retrieved context
- ✅ No retrieved chunks (similarity < 0.70) → "Information not available in the textbook"
- ✅ Agent system prompt enforces grounding constraint
- ✅ Responses cite retrieved chunks with source URLs

**Implementation**: Agent system prompt includes "Answer ONLY based on provided context. If information is not in the context, respond: 'I don't have information about that in the textbook.'"

### Principle II: Cohere Embeddings Standard ✅ PASS

- ✅ Uses Cohere embed-english-v3.0 for query embeddings
- ✅ input_type="search_query" (consistent with feature 008 retrieval validation)
- ✅ No mixing of embedding providers
- ✅ Reuses existing Cohere API integration from backend

**Implementation**: Query embedding function uses same Cohere client and model as features 007/008

### Principle III: Qdrant Vector Storage ✅ PASS

- ✅ Queries "rag_embedding" collection from feature 007
- ✅ Read-only access (no writes to collection)
- ✅ Preserves metadata (source URL, chapter ID, module name)
- ✅ Configurable similarity threshold (default 0.70)

**Implementation**: Qdrant client connects to same instance used by ingestion pipeline

### Principle IV: Content Extraction Pipeline ✅ N/A

- N/A: This feature consumes pre-extracted content from feature 007
- No content extraction or ingestion in this feature
- Feature 007 handles Crawl → Clean → Chunk → Embed → Store

**Rationale**: Agent is read-only consumer of existing pipeline

### Principle V: Agent-Based Retrieval ⚠️ PARTIAL DEVIATION

**Constitution Requirement**: FastAPI backend exposing retrieval endpoints
**This Feature**: Command-line tool only (no FastAPI per user request)

**Justification**:
- User explicitly requested "Without FastAPI" in feature description
- Specification states "Out of Scope: Web-based user interface (FastAPI, frontend)"
- MVP focuses on core RAG functionality (retrieval + generation)
- FastAPI integration deferred to future feature (Docusaurus chatbot)

**Compliance**: Agent-based retrieval with tool integration maintained (OpenAI Agents SDK + retrieval function)

### Principle VI: Docusaurus Chatbot Integration ⚠️ FUTURE FEATURE

- N/A: Docusaurus integration out of scope for this feature
- Command-line agent is foundational component for future chatbot
- Future feature will wrap agent logic in FastAPI and integrate with Docusaurus

**Rationale**: This feature establishes core retrieval + generation logic that will be reused by chatbot

### Summary

**Status**: ✅ PASS with documented deviations

- **Deviations**: No FastAPI (Principle V), no Docusaurus integration (Principle VI)
- **Justification**: Explicit user requirement ("Without FastAPI"), MVP-first approach
- **Compliance**: Core grounding, embedding, and storage principles fully satisfied
- **Migration Path**: Future feature will add FastAPI wrapper and Docusaurus integration

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent.py             # NEW: RAG agent with OpenAI SDK + Qdrant retrieval
├── main.py              # Existing: Embedding ingestion pipeline (feature 007)
├── retrieve.py          # Existing: Retrieval validation tool (feature 008)
├── .env                 # Shared: API keys (OPENAI_API_KEY added)
├── pyproject.toml       # Updated: Add openai dependency
├── .venv/               # Existing: Virtual environment
└── README.md            # Updated: Add agent.py usage section
```

**Structure Decision**: Single-file architecture matching features 007/008. Agent implementation in `backend/agent.py` reuses existing infrastructure (.env, Qdrant client, Cohere integration). No new directories or complex structure needed.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No complexity violations - constitution deviations (no FastAPI, no Docusaurus) explicitly requested by user and documented in Constitution Check section.

---

## Phase 0: Research Complete ✅

**Artifacts Created**:
- `research.md` - 7 research questions with decisions, rationale, and alternatives

**Key Decisions Made**:
1. Use OpenAI Python SDK (>=1.0.0) with function calling
2. Retrieval function as OpenAI tool with query/top_k/threshold parameters
3. In-memory conversation history (list of message dicts, max 20 messages)
4. Explicit grounding system prompt with failure mode handling
5. CLI with `--interactive` flag using argparse
6. Retry logic with exponential backoff for API errors
7. Inline citations + structured metadata output

**Dependencies Added**: `openai>=1.0.0`

---

## Phase 1: Design Complete ✅

**Artifacts Created**:
- `data-model.md` - 6 entities with validation rules and relationships
- `contracts/agent-functions.yaml` - Function signatures and OpenAI tool schemas
- `quickstart.md` - Setup instructions and usage examples

**Entities Defined**:
1. **AgentSession** - Session state with conversation history
2. **Message** - Individual message (user/assistant/system/function)
3. **FunctionCall** - Retrieval function request from agent
4. **RetrievalResult** - Retrieved chunk with metadata
5. **ConversationTurn** - One Q&A exchange
6. **AgentResponse** - Generated answer with citations

**Contracts Specified**:
- 11 core functions (retrieve_context, initialize_agent, ask_question, etc.)
- OpenAI tool schema for function calling
- CLI arguments and environment variables
- Error codes and performance targets

---

## Next Steps (Implementation - `/sp.tasks`)

1. Run `/sp.tasks` to generate task breakdown
2. Implement `backend/agent.py` following research decisions and contracts
3. Add `openai` dependency to `backend/pyproject.toml`
4. Update `backend/README.md` with agent usage
5. Test with queries from feature 008 validation tool
6. Verify Constitution compliance (grounding, citations, error handling)
