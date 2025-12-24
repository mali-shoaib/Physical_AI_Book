# Research: Retrieval-Enabled Agent

**Feature**: 009-retrieval-agent
**Date**: 2025-12-23
**Purpose**: Research technical decisions for OpenAI Agents SDK + Qdrant RAG implementation

---

## Research Questions

### 1. Which OpenAI SDK should we use for agent functionality?

**Decision**: Use OpenAI Python SDK (>=1.0.0) with Assistants API (beta) or newer Agents API if available

**Rationale**:
- OpenAI's official Python SDK provides both Chat Completions API and Assistants API
- Assistants API supports function calling which is essential for retrieval tool integration
- SDK handles conversation history management, tool execution, and streaming
- Well-documented, actively maintained, stable API

**Alternatives Considered**:
1. **LangChain with OpenAI**:
   - **Rejected**: Adds unnecessary abstraction layer and complexity
   - Con: Heavy dependency footprint, harder to debug, slower updates
   - Con: Constitution specifies "OpenAI Agents SDK", not third-party frameworks

2. **Direct OpenAI API calls with requests library**:
   - **Rejected**: Too low-level, would need to implement conversation management manually
   - Con: No built-in function calling support, manual message formatting
   - Con: More prone to errors, harder to maintain

3. **Haystack or LlamaIndex**:
   - **Rejected**: Overkill for this use case, opinionated RAG frameworks
   - Con: Lock-in to framework patterns, harder to customize grounding logic

**Implementation Notes**:
- Use `openai.Client` for API interactions
- Leverage function calling feature to define `retrieve_context` tool
- System message enforces grounding constraint
- Conversation history stored in list of message dicts (user-assistant pairs)

---

### 2. How should we structure the retrieval function for OpenAI function calling?

**Decision**: Define retrieval as OpenAI function tool with parameters: `query: str`, `top_k: int`, `threshold: float`

**Rationale**:
- Function calling allows agent to autonomously decide when to retrieve context
- Tool definition specifies exact parameters the agent can control
- Enables agent to request more/fewer chunks based on question complexity
- Maintains grounding by making retrieval explicit in conversation flow

**Function Schema**:
```python
{
    "type": "function",
    "function": {
        "name": "retrieve_context",
        "description": "Retrieve relevant content from the Physical AI textbook based on semantic similarity",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query to find relevant textbook content"
                },
                "top_k": {
                    "type": "integer",
                    "description": "Number of most relevant chunks to retrieve (default: 5)",
                    "default": 5
                },
                "threshold": {
                    "type": "number",
                    "description": "Minimum similarity score (0-1, default: 0.70)",
                    "default": 0.70
                }
            },
            "required": ["query"]
        }
    }
}
```

**Alternatives Considered**:
1. **Always retrieve before each response**:
   - **Rejected**: Inefficient for follow-up questions that don't need new retrieval
   - Con: Wastes API calls and time on clarification questions
   - Con: Agent can't autonomously decide when retrieval is needed

2. **Manual retrieval trigger (user types "search: ...")**:
   - **Rejected**: Poor UX, requires user to understand retrieval mechanics
   - Con: Breaks natural conversation flow
   - Con: User might forget to trigger retrieval

**Implementation Notes**:
- Function implementation reuses `query_qdrant()` from feature 008 (retrieve.py)
- Returns formatted context string with source citations
- If no results above threshold, return empty list (agent handles gracefully)

---

### 3. How should conversation history be managed?

**Decision**: In-memory list of message dicts with max_history parameter (default: 20 messages)

**Rationale**:
- OpenAI SDK requires conversation history as list of `{"role": ..., "content": ...}` dicts
- In-memory storage aligns with constitution constraint (no persistence)
- Max history prevents token limit issues with long conversations
- Simple implementation without external dependencies

**Data Structure**:
```python
conversation_history = [
    {"role": "system", "content": "You are a helpful assistant..."},
    {"role": "user", "content": "What is ROS 2?"},
    {"role": "assistant", "content": "ROS 2 is..."},
    {"role": "user", "content": "How do I install it?"},
    # ... continues
]
```

**Alternatives Considered**:
1. **SQLite database for persistence**:
   - **Rejected**: Violates constitution constraint (in-memory only)
   - Con: Adds complexity without clear benefit for CLI tool

2. **Redis or external session store**:
   - **Rejected**: Overkill for single-user local tool
   - Con: Requires external service, violates single-file architecture

3. **File-based JSON storage**:
   - **Rejected**: Persistent storage not required per spec
   - Con: Adds I/O overhead, file locking issues

**Implementation Notes**:
- Implement `reset_conversation()` function to clear history (keeps system message)
- Sliding window: remove oldest user-assistant pair when max_history reached
- System message always first, never removed

---

### 4. What should the agent system prompt contain to enforce grounding?

**Decision**: Explicit grounding instructions with failure mode handling

**System Prompt**:
```
You are a helpful assistant for the Physical AI Robotics Textbook. Your role is to answer questions STRICTLY based on retrieved content from the textbook.

CRITICAL RULES:
1. Answer ONLY using information from the retrieved context provided in function calls
2. If the context doesn't contain relevant information, respond: "I don't have information about that in the textbook. Please ask about ROS 2, Isaac Sim, URDF, VSLAM, or other topics covered in the Physical AI curriculum."
3. Always cite the source chapter/module when answering (e.g., "According to Module 1 - ROS 2...")
4. Do not use external knowledge, assumptions, or information not in the context
5. If the question is unclear, ask for clarification
6. For multi-part questions, retrieve context for each part separately if needed

When you need information to answer a question, use the retrieve_context function.
```

**Rationale**:
- Explicit grounding rules prevent hallucination
- Failure mode specified ("I don't have information...")
- Citation requirement ensures traceability
- Multi-part question handling improves UX

**Alternatives Considered**:
1. **Minimal prompt ("Answer based on context")**:
   - **Rejected**: Too vague, agent may hallucinate
   - Con: No explicit failure mode, poor grounding enforcement

2. **Chain-of-thought prompting**:
   - **Rejected**: Adds token overhead, slower responses
   - Con: Unnecessary complexity for straightforward Q&A

**Implementation Notes**:
- System message set once at conversation start
- Prompt testing with queries outside textbook scope to verify refusal behavior
- Iterate prompt based on validation results

---

### 5. How should CLI modes (single question vs interactive) be implemented?

**Decision**: Single entry point with `--interactive` flag using argparse

**CLI Interface**:
```bash
# Single question mode
python backend/agent.py "What is ROS 2?"

# Interactive mode
python backend/agent.py --interactive

# With custom settings
python backend/agent.py --interactive --model gpt-4-turbo --max-history 30
```

**Rationale**:
- Argparse is stdlib, no external dependency
- Single entry point simplifies usage
- Interactive mode uses input() loop with exit commands
- Matches pattern from feature 008 (retrieve.py)

**Implementation**:
```python
def main():
    parser = argparse.ArgumentParser(description="RAG Agent for Physical AI Textbook")
    parser.add_argument("question", nargs="?", help="Question to ask (single-shot mode)")
    parser.add_argument("--interactive", action="store_true", help="Start interactive session")
    parser.add_argument("--model", default="gpt-4", help="OpenAI model (default: gpt-4)")
    parser.add_argument("--max-history", type=int, default=20, help="Max conversation turns")
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING"])

    args = parser.parse_args()

    if args.interactive:
        interactive_mode(args)
    elif args.question:
        single_question_mode(args.question, args)
    else:
        parser.print_help()
```

**Alternatives Considered**:
1. **Separate scripts (agent_single.py, agent_interactive.py)**:
   - **Rejected**: Code duplication, harder to maintain
   - Con: Violates single-file architecture goal

2. **Always interactive (no single-shot mode)**:
   - **Rejected**: Poor DX for quick questions, harder to script
   - Con: Can't pipe questions or integrate with other tools

**Implementation Notes**:
- Interactive mode: `while True` loop with `input("You: ")`
- Exit commands: "exit", "quit", "bye", Ctrl+C
- Reset command: "reset" clears conversation history

---

### 6. How should errors be handled (API failures, Qdrant errors, embedding errors)?

**Decision**: Retry logic with exponential backoff + clear error messages

**Error Handling Strategy**:

**OpenAI API Errors**:
```python
@retry(max_attempts=3, backoff_factor=2.0)
def call_openai_api(messages, tools):
    try:
        response = client.chat.completions.create(...)
        return response
    except openai.APIError as e:
        logger.error(f"OpenAI API error: {e}")
        raise
    except openai.RateLimitError as e:
        logger.warning(f"Rate limit hit, retrying...")
        raise
```

**Qdrant Errors**:
```python
def retrieve_context(query, top_k=5, threshold=0.70):
    try:
        # Query Qdrant
        results = qdrant_client.search(...)
    except QdrantException as e:
        logger.error(f"Qdrant error: {e}")
        return {"error": "Unable to retrieve content. Please check Qdrant connection."}
```

**Cohere Embedding Errors**:
```python
def embed_query(text):
    try:
        response = cohere_client.embed(...)
        return response.embeddings[0]
    except CohereError as e:
        logger.error(f"Cohere embedding error: {e}")
        raise
```

**Rationale**:
- Retry logic handles transient failures (rate limits, network blips)
- Clear error messages help debugging
- Graceful degradation where possible
- Aligns with SC-007 (graceful error handling)

**Alternatives Considered**:
1. **No retry logic**:
   - **Rejected**: Poor reliability, frustrating UX
   - Con: Single network error breaks entire session

2. **Unlimited retries**:
   - **Rejected**: Could hang indefinitely on persistent errors
   - Con: No clear failure signal to user

**Implementation Notes**:
- Use `tenacity` library for retry decorator (already in backend dependencies)
- Log all errors with timestamps for debugging
- Return user-friendly messages ("Unable to connect to Qdrant. Check .env configuration.")

---

### 7. Should citations be inline in the answer or separate metadata?

**Decision**: Inline citations in answer text + detailed metadata in structured output

**Format**:
```
Answer: "ROS 2 is a flexible framework for writing robot software [Source: Module 1 - ROS 2 Basics]. It provides..."

Metadata:
{
  "answer": "ROS 2 is...",
  "citations": [
    {
      "chunk_id": "uuid-123",
      "source_url": "https://textbook.com/docs/module-1/ros2-basics",
      "chapter_id": "module-1-ros2/ch1-basics",
      "module_name": "Module 1 Ros2",
      "similarity_score": 0.87,
      "chunk_text": "ROS 2 is a flexible framework..."
    }
  ],
  "retrieval_count": 3,
  "model_used": "gpt-4"
}
```

**Rationale**:
- Inline citations improve answer readability
- Metadata provides full context for debugging and verification
- Supports both US1 (simple Q&A) and US3 (detailed citations)
- Matches constitution requirement for traceability

**Alternatives Considered**:
1. **Citations only in metadata (not inline)**:
   - **Rejected**: Poor UX, user must cross-reference manually
   - Con: Harder to verify which chunk supports which claim

2. **Only inline citations (no metadata)**:
   - **Rejected**: Lacks details for debugging and verification
   - Con: Doesn't satisfy US3 (citation verification)

**Implementation Notes**:
- Agent instructed to include `[Source: ...]` inline
- Post-process response to extract and format metadata
- Option to show/hide metadata via CLI flag `--show-citations`

---

## Summary of Research Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **OpenAI SDK** | OpenAI Python SDK (>=1.0.0) with function calling | Official SDK, built-in conversation management, function tools support |
| **Retrieval Function** | OpenAI function tool with query/top_k/threshold params | Agent autonomy, explicit retrieval, parameter control |
| **Conversation History** | In-memory list, max 20 messages, sliding window | Constitution constraint, simple implementation, token management |
| **System Prompt** | Explicit grounding rules + failure mode | Prevents hallucination, enforces citation, handles edge cases |
| **CLI Modes** | Argparse with `--interactive` flag | Single entry point, flexible usage, matches feature 008 pattern |
| **Error Handling** | Retry with exponential backoff + clear messages | Reliability, UX, graceful degradation |
| **Citations** | Inline + structured metadata | Readability, traceability, supports all user stories |

---

## Dependencies Added

**New Python Packages** (to be added to `backend/pyproject.toml`):
- `openai>=1.0.0` - OpenAI API client with function calling
- No new packages needed for other components (reuse cohere, qdrant-client, loguru)

**Environment Variables** (to be added to `backend/.env`):
```bash
OPENAI_API_KEY=your_key_here          # NEW: Required for agent
OPENAI_MODEL=gpt-4                    # NEW: Optional, defaults to gpt-4
AGENT_MAX_HISTORY=20                  # NEW: Optional conversation history limit
```

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| OpenAI API rate limits | Medium | High | Retry logic, clear error messages, rate limit detection |
| Token limit exceeded in conversation | Medium | Medium | Max history limit, sliding window, truncate old messages |
| Agent hallucinates despite grounding | Low | High | Strong system prompt, function calling enforcement, testing with out-of-scope queries |
| Qdrant connection failures | Low | High | Retry logic, connection validation on startup, clear error messaging |
| Cohere embedding API failures | Low | Medium | Reuse retry logic from feature 008, validate on startup |

---

## Next Steps (Phase 1)

1. Create **data-model.md** with entity definitions (AgentSession, RetrievalResult, AgentResponse, ConversationTurn)
2. Create **contracts/agent-functions.yaml** with function signatures and OpenAI tool schemas
3. Create **quickstart.md** with setup instructions and example usage
4. Implement **backend/agent.py** following research decisions
