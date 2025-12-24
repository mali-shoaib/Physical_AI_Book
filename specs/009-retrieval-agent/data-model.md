# Data Model: Retrieval-Enabled Agent

**Feature**: 009-retrieval-agent
**Date**: 2025-12-23
**Purpose**: Define entities and data structures for RAG agent

---

## Entity Definitions

### 1. AgentSession

**Description**: Represents a single interaction session with the RAG agent, maintaining conversation state and history.

**Attributes**:
- `session_id` (str): Unique identifier for the session (UUID)
- `started_at` (datetime): Session start timestamp (UTC)
- `model` (str): OpenAI model being used (e.g., "gpt-4", "gpt-4-turbo")
- `conversation_history` (List[Message]): Ordered list of conversation messages
- `max_history` (int): Maximum number of messages to retain (default: 20)
- `is_interactive` (bool): Whether session is interactive or single-question

**Relationships**:
- Contains multiple `ConversationTurn` objects
- Has zero or more `RetrievalResult` objects from all turns

**State Transitions**:
```
CREATED → ACTIVE → COMPLETED
         ↓
      RESET (clears history, stays ACTIVE)
```

**Validation Rules**:
- `session_id` must be unique UUID
- `max_history` must be > 0 and <= 100
- `conversation_history` length <= max_history (sliding window)
- First message in history must be system message (role="system")

**Example**:
```python
{
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "started_at": "2025-12-23T18:30:00Z",
    "model": "gpt-4",
    "conversation_history": [
        {"role": "system", "content": "You are a helpful assistant..."},
        {"role": "user", "content": "What is ROS 2?"},
        {"role": "assistant", "content": "ROS 2 is..."}
    ],
    "max_history": 20,
    "is_interactive": true
}
```

---

### 2. Message

**Description**: Single message in the conversation (user, assistant, system, or function result).

**Attributes**:
- `role` (str): Message sender - "system", "user", "assistant", or "function"
- `content` (str): Message text content
- `function_call` (Optional[FunctionCall]): If assistant requests function execution
- `function_response` (Optional[str]): If message is function result
- `timestamp` (datetime): When message was created (UTC)

**Relationships**:
- Belongs to `AgentSession.conversation_history`
- May reference `FunctionCall` if role is "assistant"

**Validation Rules**:
- `role` must be one of: "system", "user", "assistant", "function"
- `content` required for all roles except when `function_call` present
- `function_call` only valid when role is "assistant"
- `function_response` only valid when role is "function"

**Example (User Message)**:
```python
{
    "role": "user",
    "content": "What is ROS 2?",
    "timestamp": "2025-12-23T18:30:15Z"
}
```

**Example (Assistant with Function Call)**:
```python
{
    "role": "assistant",
    "function_call": {
        "name": "retrieve_context",
        "arguments": {"query": "ROS 2 basics", "top_k": 5, "threshold": 0.70}
    },
    "timestamp": "2025-12-23T18:30:16Z"
}
```

---

### 3. FunctionCall

**Description**: Represents a request from the assistant to execute a retrieval function.

**Attributes**:
- `name` (str): Function name ("retrieve_context")
- `arguments` (Dict[str, Any]): Function parameters (query, top_k, threshold)

**Validation Rules**:
- `name` must be "retrieve_context" (only supported function)
- `arguments` must contain "query" (str)
- `arguments.top_k` must be 1-10 if present
- `arguments.threshold` must be 0.0-1.0 if present

**Example**:
```python
{
    "name": "retrieve_context",
    "arguments": {
        "query": "ROS 2 communication patterns",
        "top_k": 5,
        "threshold": 0.70
    }
}
```

---

### 4. RetrievalResult

**Description**: Contains a single retrieved chunk from Qdrant with similarity score and metadata.

**Attributes**:
- `chunk_id` (str): Unique identifier from Qdrant point ID
- `chunk_text` (str): Full text content of the chunk
- `similarity_score` (float): Cosine similarity score (0.0-1.0)
- `source_url` (str): URL to source documentation page
- `chapter_id` (str): Chapter identifier (e.g., "module-1-ros2/ch1-basics")
- `module_name` (str): Module name (e.g., "Module 1 Ros2")
- `heading_hierarchy` (List[str]): Ordered list of section headings
- `token_count` (int): Number of tokens in chunk
- `chunk_index` (int): Position of chunk in source document

**Relationships**:
- Retrieved by `ConversationTurn.retrieve_context()` function
- Multiple results form context for `AgentResponse`

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `chunk_text` cannot be empty
- `source_url` must be valid HTTP/HTTPS URL
- `token_count` must be > 0

**Sorting**:
- Default sort: `similarity_score` descending

**Example**:
```python
{
    "chunk_id": "uuid-abc123",
    "chunk_text": "ROS 2 is a flexible framework for writing robot software. It provides services including hardware abstraction, device drivers, and message-passing between processes.",
    "similarity_score": 0.87,
    "source_url": "https://physical-ai-robotics-textbook-xi.vercel.app/docs/module-1-ros2/ch1-basics",
    "chapter_id": "module-1-ros2/ch1-basics",
    "module_name": "Module 1 Ros2",
    "heading_hierarchy": ["Chapter 1: ROS 2 Basics", "What is ROS 2?"],
    "token_count": 42,
    "chunk_index": 0
}
```

---

### 5. ConversationTurn

**Description**: Represents one question-answer exchange including user query, retrieval, and agent response.

**Attributes**:
- `turn_id` (str): Unique identifier (UUID)
- `user_query` (str): The question asked by the user
- `retrieval_results` (List[RetrievalResult]): Chunks retrieved for this turn
- `agent_response` (AgentResponse): The generated answer
- `timestamp` (datetime): When turn occurred (UTC)
- `duration_seconds` (float): Total time for retrieval + generation

**Relationships**:
- Contains one `AgentResponse`
- Contains zero or more `RetrievalResult` objects
- Belongs to `AgentSession`

**Validation Rules**:
- `user_query` cannot be empty
- `duration_seconds` must be > 0
- If `retrieval_results` is empty, `agent_response` must indicate "no information"

**Example**:
```python
{
    "turn_id": "turn-550e8400",
    "user_query": "What is ROS 2?",
    "retrieval_results": [
        {
            "chunk_id": "uuid-abc123",
            "chunk_text": "ROS 2 is a flexible framework...",
            "similarity_score": 0.87,
            # ... (full RetrievalResult)
        }
    ],
    "agent_response": {
        "answer": "ROS 2 is a flexible framework...",
        "citations": [...],
        # ... (full AgentResponse)
    },
    "timestamp": "2025-12-23T18:30:20Z",
    "duration_seconds": 8.5
}
```

---

### 6. AgentResponse

**Description**: The generated answer with citations and metadata from the OpenAI agent.

**Attributes**:
- `answer` (str): The natural language answer text
- `citations` (List[RetrievalResult]): Sources used to generate answer
- `retrieval_count` (int): Number of chunks retrieved
- `model_used` (str): OpenAI model that generated response
- `tokens_used` (int): Total tokens consumed (prompt + completion)
- `grounded` (bool): Whether answer is based on retrieved context (vs. "no information" response)
- `timestamp` (datetime): When response was generated (UTC)

**Relationships**:
- Belongs to one `ConversationTurn`
- References multiple `RetrievalResult` objects as citations

**Validation Rules**:
- `answer` cannot be empty
- `retrieval_count` must be >= 0
- `tokens_used` must be > 0
- If `grounded` is false, answer must contain "I don't have information" phrase
- If `grounded` is true, `citations` must not be empty

**Example (Grounded Response)**:
```python
{
    "answer": "ROS 2 is a flexible framework for writing robot software [Source: Module 1 - ROS 2 Basics]. It provides services including hardware abstraction, device drivers, and message-passing between processes.",
    "citations": [
        {
            "chunk_id": "uuid-abc123",
            "chunk_text": "ROS 2 is a flexible framework...",
            "similarity_score": 0.87,
            "source_url": "https://...",
            # ... (full RetrievalResult)
        }
    ],
    "retrieval_count": 3,
    "model_used": "gpt-4",
    "tokens_used": 450,
    "grounded": true,
    "timestamp": "2025-12-23T18:30:20Z"
}
```

**Example (Ungrounded Response)**:
```python
{
    "answer": "I don't have information about weather forecasting in the textbook. Please ask about ROS 2, Isaac Sim, URDF, VSLAM, or other topics covered in the Physical AI curriculum.",
    "citations": [],
    "retrieval_count": 0,
    "model_used": "gpt-4",
    "tokens_used": 120,
    "grounded": false,
    "timestamp": "2025-12-23T18:30:25Z"
}
```

---

## Entity Relationships Diagram

```
AgentSession
├── session_id (UUID)
├── conversation_history (List[Message])
└── turns (implicit, derived from history)
    └── ConversationTurn
        ├── turn_id (UUID)
        ├── user_query (str)
        ├── retrieval_results (List[RetrievalResult])
        │   └── RetrievalResult
        │       ├── chunk_id (UUID)
        │       ├── chunk_text (str)
        │       ├── similarity_score (float)
        │       └── metadata (source_url, chapter_id, etc.)
        └── agent_response (AgentResponse)
            ├── answer (str)
            ├── citations (List[RetrievalResult])
            └── metadata (model_used, tokens_used, etc.)
```

---

## Data Flow

### Single Question Mode
```
1. User provides query → Message (role=user)
2. Agent calls retrieve_context() → FunctionCall
3. Retrieval function queries Qdrant → List[RetrievalResult]
4. Agent generates answer with context → AgentResponse
5. ConversationTurn records entire exchange
6. Output AgentResponse to user
```

### Interactive Mode
```
1. Initialize AgentSession
2. Loop:
   a. User input → Message (role=user)
   b. Append to conversation_history
   c. Agent processes message (may call retrieve_context)
   d. Generate AgentResponse
   e. Append assistant message to conversation_history
   f. Output to user
3. Until user exits or session terminates
```

---

## Validation Rules Summary

| Entity | Key Validations |
|--------|-----------------|
| **AgentSession** | UUID unique, max_history > 0, first message is system |
| **Message** | Role in allowed set, content or function_call present |
| **FunctionCall** | name="retrieve_context", query required, top_k 1-10, threshold 0-1 |
| **RetrievalResult** | similarity_score 0-1, chunk_text non-empty, valid URL |
| **ConversationTurn** | user_query non-empty, duration > 0 |
| **AgentResponse** | answer non-empty, grounded matches citation presence |

---

## Performance Considerations

**Memory Usage**:
- Each Message: ~500 bytes (average)
- Max conversation_history (20 messages): ~10 KB
- RetrievalResult with 1024-token chunk: ~4 KB
- Total session memory: <50 KB (negligible)

**Token Management**:
- System message: ~200 tokens
- Average user query: ~50 tokens
- Average assistant response: ~300 tokens
- Average retrieved context (5 chunks): ~3000 tokens
- Max conversation with 20 messages: ~10,000 tokens (well under gpt-4 limits)

**Optimization Strategies**:
- Sliding window removes oldest messages when max_history exceeded
- Truncate very long retrieved chunks if necessary
- Cache Qdrant results for identical queries within session

---

## Error States

| State | Description | Recovery |
|-------|-------------|----------|
| **EMPTY_RETRIEVAL** | No chunks above similarity threshold | Return "no information" response |
| **API_ERROR** | OpenAI or Cohere API failure | Retry with exponential backoff (max 3 attempts) |
| **QDRANT_ERROR** | Qdrant connection or query failure | Log error, return "Unable to retrieve content" message |
| **TOKEN_LIMIT** | Conversation exceeds model context limit | Truncate oldest messages, warn user |
| **INVALID_INPUT** | Empty query or malformed input | Prompt user to rephrase |

---

## Testing Scenarios

1. **Single Question** (User Story 1):
   - Input: "What is ROS 2?"
   - Expected: AgentResponse with grounded=true, citations from Module 1

2. **Out-of-Scope Question** (Edge Case):
   - Input: "What's the weather today?"
   - Expected: AgentResponse with grounded=false, "I don't have information" message

3. **Multi-Turn Conversation** (User Story 2):
   - Turn 1: "What is Isaac Sim?"
   - Turn 2: "How do I use it for synthetic data?"
   - Expected: Second turn understands "it" refers to Isaac Sim, retrieves relevant context

4. **Citation Verification** (User Story 3):
   - Input: Any valid question
   - Expected: AgentResponse.citations contains valid source_url, chunk_id, similarity_score

5. **History Reset**:
   - Initial conversation with 5 turns
   - Reset command
   - Expected: conversation_history clears except system message, session continues

---

## Next Steps

- Implement data classes in `backend/agent.py` using Python dataclasses or Pydantic
- Add validation logic in constructors
- Implement serialization/deserialization for logging and debugging
- Create unit tests for validation rules
