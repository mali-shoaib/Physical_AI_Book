---
id: 002
title: Create Retrieval Agent Plan
stage: plan
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: retrieval-agent
branch: 009-retrieval-agent
user: user
command: /sp.plan
labels: ["planning", "rag-agent", "openai-sdk", "qdrant-integration", "retrieval-tool", "constitution-compliant"]
links:
  spec: specs/009-retrieval-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/009-retrieval-agent/plan.md
  - specs/009-retrieval-agent/research.md
  - specs/009-retrieval-agent/data-model.md
  - specs/009-retrieval-agent/contracts/agent-functions.yaml
  - specs/009-retrieval-agent/quickstart.md
tests:
  - Constitution Check: PASS with documented deviations (no FastAPI per user request)
---

## Prompt

User executed `/sp.plan` command with guidance:

```
- Initialize an OpenAI Agent with custom retrieval tool
- Connect tool to Qdrant collection for similarity search
- Pass retrieval chunks in to agents's context
- You can create agent.py file in backend folder in this file you can get embedding from qdrant you have
  qdrant url and qdrant api and you have also qdrant collection name than retireve information to agent and answer based on the user's query you can use
  OpenAI Agents SDK for agent with retirieve function .
```

Context: Generate implementation plan for Retrieval-Enabled Agent feature based on spec.md and constitution.md

## Response snapshot

Created comprehensive implementation plan with complete Phase 0 (Research) and Phase 1 (Design) deliverables:

**Summary**:
- Tool: Command-line RAG agent using OpenAI SDK + Qdrant retrieval
- Architecture: Single-file (`backend/agent.py`), reuses existing backend infrastructure
- Constitution compliance: Core principles satisfied, documented deviations for FastAPI/Docusaurus (user requested)

**Technical Context**:
- Language: Python 3.10+ (matching existing backend)
- Dependencies: OpenAI Python SDK (>=1.0.0) - ONLY new dependency, reuses cohere/qdrant-client/loguru
- Storage: Read-only access to Qdrant "rag_embedding" collection (feature 007)
- Performance: <2s retrieval, <10s end-to-end response
- Constraints: In-memory conversation context, no FastAPI/web server per user request

**Constitution Check**: ✅ PASS with documented deviations
- Principle I (Grounded Answers): PASS - system prompt enforces grounding, no-context refusal
- Principle II (Cohere Embeddings): PASS - uses embed-english-v3.0 with input_type="search_query"
- Principle III (Qdrant Storage): PASS - queries "rag_embedding" collection, read-only
- Principle IV (Content Extraction): N/A - consumes existing pipeline (feature 007)
- Principle V (Agent-Based Retrieval): PARTIAL - OpenAI SDK + function calling, no FastAPI (user requested)
- Principle VI (Docusaurus Chatbot): FUTURE - out of scope for CLI tool MVP

**Phase 0: Research (research.md)**:
- 7 research questions answered with decisions, rationale, alternatives
- Key decisions:
  1. OpenAI Python SDK with function calling (vs LangChain, direct API, frameworks)
  2. Retrieval as OpenAI function tool (query/top_k/threshold parameters)
  3. In-memory conversation history (list of dicts, max 20 messages, sliding window)
  4. Explicit grounding system prompt with failure mode ("I don't have information...")
  5. CLI with --interactive flag using argparse (single-shot + conversation modes)
  6. Retry logic with exponential backoff (3 attempts, 2s/4s/8s)
  7. Inline citations + structured metadata (supports US1/US3)

**Phase 1: Design Documents**:

1. **data-model.md**: 6 entities defined
   - AgentSession: Session state with conversation history, max_history, model
   - Message: Individual message (user/assistant/system/function) with role/content
   - FunctionCall: Retrieval function request (name="retrieve_context", arguments)
   - RetrievalResult: Retrieved chunk with metadata (chunk_id, similarity_score, source_url, chapter_id, etc.)
   - ConversationTurn: One Q&A exchange (user_query, retrieval_results, agent_response, duration)
   - AgentResponse: Generated answer (answer text, citations, grounded flag, tokens_used)

2. **contracts/agent-functions.yaml**: 11 functions + configuration
   - Core: retrieve_context (OpenAI tool), initialize_agent, ask_question, reset_conversation
   - Helpers: embed_query, format_citations, validate_environment
   - CLI: single_question_mode, interactive_mode
   - OpenAI tool schema for function calling with parameters
   - Environment variables, CLI arguments, error codes, performance targets

3. **quickstart.md**: Complete usage guide
   - Prerequisites (feature 007/008, OpenAI API key)
   - Installation (add openai>=1.0.0, update .env)
   - Quick start (single-question + interactive modes)
   - Configuration (environment variables, CLI arguments)
   - Common use cases (quick lookup, learning session, debugging, different models, retrieval tuning)
   - Troubleshooting (7 common issues with fixes)
   - Advanced usage (scripting, automation)
   - Testing scenarios (known questions, edge cases)
   - Performance expectations table
   - Success checklist

**Project Structure**:
```
backend/
├── agent.py             # NEW: RAG agent implementation
├── main.py              # Existing: Ingestion (feature 007)
├── retrieve.py          # Existing: Validation (feature 008)
├── .env                 # Updated: Add OPENAI_API_KEY
├── pyproject.toml       # Updated: Add openai dependency
└── README.md            # Updated: Add agent usage section
```

**Success Criteria Mapping**:
- SC-001: 90%+ grounded answers → System prompt + function calling enforcement
- SC-002: <2s retrieval → Reuses feature 008 query_qdrant()
- SC-003: <10s end-to-end → OpenAI API latency + retrieval
- SC-004: Zero hallucination → Explicit grounding rules in system prompt
- SC-005: 10+ turn context → In-memory history with sliding window
- SC-006: 95%+ valid citations → Metadata from Qdrant, inline + structured format
- SC-007: Graceful errors → Retry logic, clear messages, error handling
- SC-008: <30s first run → Environment validation, simple setup

## Outcome

- ✅ Impact: Complete implementation plan with Phase 0 (Research) and Phase 1 (Design) artifacts. All technical decisions made with alternatives considered. Constitution-compliant with documented deviations (no FastAPI per user request). Ready for task generation.
- 🧪 Tests: Constitution Check PASS (6/6 principles evaluated, 2 documented deviations). Research completeness PASS (7 questions with decisions/rationale). Design completeness PASS (6 entities, 11 functions, complete quickstart).
- 📁 Files: Created plan.md (179 lines), research.md (350 lines), data-model.md (420 lines), contracts/agent-functions.yaml (450 lines), quickstart.md (416 lines)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown organized by user story
- 🧠 Reflection: Plan successfully integrates OpenAI SDK with existing Qdrant/Cohere infrastructure. Single-file architecture (backend/agent.py) keeps implementation simple. Three user stories enable incremental delivery (P1: CLI Q&A, P2: interactive mode, P3: citations). Only one new dependency (openai SDK), reuses all backend infrastructure. Constitution deviations (no FastAPI/Docusaurus) explicitly requested and justified. System prompt design critical for grounding enforcement.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all Phase 0 and Phase 1 artifacts
- Graders run and results (PASS/FAIL): Constitution Check PASS (core grounding/embedding/storage principles satisfied). Research completeness PASS (7 questions, alternatives documented). Data model PASS (6 entities with validation rules). Contracts PASS (11 functions with OpenAI tool schemas). Quickstart quality PASS (prerequisites, installation, usage, troubleshooting, examples).
- Prompt variant (if applicable): Standard /sp.plan workflow with user guidance to create backend/agent.py with OpenAI SDK + Qdrant retrieval
- Next experiment (smallest change to try): None needed - plan is complete and ready for task generation
