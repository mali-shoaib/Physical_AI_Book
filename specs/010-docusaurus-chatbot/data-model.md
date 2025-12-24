# Data Model: Frontend-Backend Integration of RAG Chatbot

**Feature**: 010-docusaurus-chatbot | **Date**: 2025-12-23 | **Status**: Design Complete

---

## Overview

This document defines the 6 core entities for the chatbot system, including data structures, validation rules, relationships, and data flow. The model spans both the HTTP boundary (request/response) and internal state management (backend and frontend).

### Entity Relationship Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                         Frontend (React)                      │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ ChatWidgetState (Component State)                       │ │
│  │ ├─ messages: ChatMessage[]                              │ │
│  │ ├─ isOpen: boolean                                      │ │
│  │ ├─ isLoading: boolean                                   │ │
│  │ ├─ error: string | null                                 │ │
│  │ └─ conversationId: string | null                        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                            │                                   │
│                            ↓ (user input)                      │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ ChatMessage (UI Object)                                 │ │
│  │ ├─ role: "user" | "assistant"                           │ │
│  │ ├─ content: string                                      │ │
│  │ ├─ timestamp: Date                                      │ │
│  │ └─ citations: Citation[] | null                         │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                               │
└──────────────────────────────────────────────────────────────┘
                              │
                    HTTP POST ↓↑ HTTP Response
                    /api/chat/query
                              │
┌──────────────────────────────────────────────────────────────┐
│                     Backend (FastAPI)                         │
│                                                               │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ ChatRequest (HTTP Request Payload)                      │ │
│  │ ├─ query: string                                        │ │
│  │ ├─ conversation_id: string | null                       │ │
│  │ └─ context: string | null (max 2000 chars)             │ │
│  └─────────────────────────────────────────────────────────┘ │
│                            │                                   │
│                            ↓ (validation & routing)            │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ ConversationState (Backend Memory)                      │ │
│  │ ├─ conversation_id: str (UUID)                          │ │
│  │ ├─ messages: List[dict]                                 │ │
│  │ ├─ created_at: datetime                                 │ │
│  │ └─ last_updated: datetime                               │ │
│  └─────────────────────────────────────────────────────────┘ │
│                            │                                   │
│                            ↓ (call agent.py)                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ agent.py (Feature 009)                                  │ │
│  │ ├─ retrieve_context(query, context)                     │ │
│  │ ├─ generate_answer(query, context, history)            │ │
│  │ └─ [returns: answer + citations]                        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                            │                                   │
│                            ↓ (format response)                 │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │ ChatResponse (HTTP Response Payload)                    │ │
│  │ ├─ answer: string                                       │ │
│  │ ├─ citations: Citation[]                                │ │
│  │ ├─ conversation_id: string                              │ │
│  │ └─ metadata: {                                          │ │
│  │     retrieval_count: int,                               │ │
│  │     tokens_used: int,                                   │ │
│  │     grounded: boolean                                   │ │
│  │   }                                                     │ │
│  └─────────────────────────────────────────────────────────┘ │
│                                                               │
└──────────────────────────────────────────────────────────────┘
```

---

## Entity 1: ChatRequest

**Purpose**: HTTP request payload sent from frontend to backend

**Scope**: REST API boundary (frontend → backend)

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `query` | string | ✅ Yes | Non-empty, max 500 chars (after trim) |
| `conversation_id` | string | ❌ No | UUID v4 format if provided, generates new UUID if null |
| `context` | string | ❌ No | Max 2000 chars (user-selected text from page) |

**Validation Rules**:

1. **query**:
   - ✅ Non-empty after trimming whitespace
   - ✅ Max 500 characters
   - ✅ Reject if only whitespace
   - ❌ Special characters allowed (no validation)

2. **conversation_id**:
   - ✅ If provided: valid UUID v4 format (36 chars with hyphens)
   - ✅ If null/missing: backend generates new UUID4
   - ✅ Treat empty string as null

3. **context**:
   - ✅ Max 2000 characters
   - ✅ Truncate with warning if exceeded
   - ✅ Trim leading/trailing whitespace

**Example JSON Request**:

```json
{
  "query": "What is Isaac Sim?",
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "context": "Isaac Sim is a physics simulation platform..."
}
```

**Example JSON Request (minimal)**:

```json
{
  "query": "How do I create a URDF?"
}
```

**Error Responses**:

```json
{
  "error": "Missing required field: query",
  "status": 400
}
```

```json
{
  "error": "Query exceeds maximum length of 500 characters",
  "status": 400
}
```

```json
{
  "error": "Context exceeds maximum length of 2000 characters. Please reduce context size.",
  "status": 400
}
```

---

## Entity 2: ChatResponse

**Purpose**: HTTP response payload returned from backend to frontend

**Scope**: REST API boundary (backend → frontend)

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `answer` | string | ✅ Yes | Non-empty response text |
| `citations` | List[Citation] | ✅ Yes | Can be empty list [] if no sources |
| `conversation_id` | string | ✅ Yes | UUID v4 format (echoed from request or generated) |
| `metadata` | object | ✅ Yes | Contains retrieval_count, tokens_used, grounded flag |

**metadata object**:

| Sub-Attribute | Type | Required | Constraints |
|---------------|------|----------|-------------|
| `retrieval_count` | integer | ✅ Yes | Number of documents retrieved from Qdrant |
| `tokens_used` | integer | ✅ Yes | Approximate tokens consumed by agent |
| `grounded` | boolean | ✅ Yes | true = answer based on textbook, false = out-of-scope |

**Validation Rules**:

1. **answer**:
   - ✅ Always non-empty string
   - ✅ If no relevant content found: "I don't have information about that in the textbook. Please try a different question."
   - ✅ Always grounded (no hallucinations)

2. **citations**:
   - ✅ Each citation object must have: module_name, chapter_id, source_url, similarity_score
   - ✅ Sorted by similarity_score descending
   - ✅ Only include citations with similarity_score >= 0.5 (configurable threshold)

3. **conversation_id**:
   - ✅ UUID v4 format (36 chars with hyphens)
   - ✅ Must match or echo request conversation_id
   - ✅ Must be new UUID if request had null/missing conversation_id

4. **metadata**:
   - ✅ retrieval_count >= 0 (can be 0 if no documents retrieved)
   - ✅ tokens_used >= 0
   - ✅ grounded: true | false (always present)

**Example JSON Response (successful query)**:

```json
{
  "answer": "Isaac Sim is NVIDIA's physics simulation platform for robotics and autonomous machines. It provides accurate physics simulation, synthetic data generation, and robot learning capabilities. You can integrate it with ROS 2 for real-world robot control and testing.",
  "citations": [
    {
      "module_name": "Module 3: Isaac",
      "chapter_id": "ch1-introduction-to-isaac",
      "source_url": "https://docs.docusaurus.ai/module-3-isaac/ch1-introduction",
      "similarity_score": 0.92
    },
    {
      "module_name": "Module 3: Isaac",
      "chapter_id": "ch2-simulation-and-physics",
      "source_url": "https://docs.docusaurus.ai/module-3-isaac/ch2-simulation",
      "similarity_score": 0.87
    }
  ],
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "metadata": {
    "retrieval_count": 5,
    "tokens_used": 342,
    "grounded": true
  }
}
```

**Example JSON Response (out-of-scope query)**:

```json
{
  "answer": "I don't have information about that in the textbook. Please try a different question.",
  "citations": [],
  "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
  "metadata": {
    "retrieval_count": 0,
    "tokens_used": 87,
    "grounded": false
  }
}
```

**Example Error Response (backend unavailable)**:

```json
{
  "error": "Backend processing error: agent initialization failed",
  "status": 500,
  "conversation_id": null
}
```

---

## Entity 3: Citation

**Purpose**: Source reference in ChatResponse indicating where the answer came from

**Scope**: Embedded in ChatResponse payload (part of response structure)

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `module_name` | string | ✅ Yes | e.g., "Module 3: Isaac", "Module 1: Foundations" |
| `chapter_id` | string | ✅ Yes | e.g., "ch1-introduction-to-isaac", kebab-case, unique per chapter |
| `source_url` | string | ✅ Yes | Full HTTPS URL to Docusaurus page |
| `similarity_score` | float | ✅ Yes | Range: 0.0 to 1.0 (Qdrant cosine similarity) |

**Validation Rules**:

1. **module_name**:
   - ✅ Non-empty string
   - ✅ Should match existing module names in docs
   - ✅ Format: "Module N: Title" pattern (enforced by content pipeline from feature 007)

2. **chapter_id**:
   - ✅ Non-empty string
   - ✅ Kebab-case format (lowercase, hyphens, no spaces)
   - ✅ Must be unique identifier for chapter in module
   - ✅ Used to construct source_url

3. **source_url**:
   - ✅ Valid HTTPS URL
   - ✅ Points to existing Docusaurus page
   - ✅ Format: `https://docs.docusaurus.ai/<module-path>/<chapter-id>`
   - ✅ Must be accessible (no broken links)

4. **similarity_score**:
   - ✅ Float between 0.0 and 1.0 (inclusive)
   - ✅ Returned from Qdrant cosine similarity computation
   - ✅ Only include in response if >= 0.5 (configurable minimum threshold)

**Example Citation Object**:

```json
{
  "module_name": "Module 3: Isaac",
  "chapter_id": "ch1-introduction-to-isaac",
  "source_url": "https://docs.docusaurus.ai/module-3-isaac/ch1-introduction",
  "similarity_score": 0.92
}
```

**Example Citation Object (lower similarity)**:

```json
{
  "module_name": "Module 2: Software Stack",
  "chapter_id": "ch5-robot-control",
  "source_url": "https://docs.docusaurus.ai/module-2-software-stack/ch5-robot-control",
  "similarity_score": 0.67
}
```

---

## Entity 4: ConversationState

**Purpose**: Backend in-memory state managing multi-turn conversation context

**Scope**: Backend-only (Python dict, not exposed directly via API)

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `conversation_id` | str | ✅ Yes | UUID v4 format, unique identifier |
| `messages` | List[dict] | ✅ Yes | List of message objects with role, content, timestamp |
| `created_at` | datetime | ✅ Yes | UTC timestamp when conversation started |
| `last_updated` | datetime | ✅ Yes | UTC timestamp of last interaction |

**Message Object Structure** (element of `messages` list):

```python
{
    "role": "user" | "assistant",
    "content": str,
    "timestamp": datetime,
    "citations": List[Citation] | None  # Only for assistant messages
}
```

**Validation Rules**:

1. **conversation_id**:
   - ✅ UUID v4 format (generated on first ChatRequest if null)
   - ✅ Unique key in in-memory dict (Python dict with UUID keys)
   - ✅ Persist for browser session lifetime only

2. **messages**:
   - ✅ Ordered list (chronological insertion order)
   - ✅ Each message has: role, content, timestamp
   - ✅ Role must be "user" or "assistant" only
   - ✅ Content non-empty string
   - ✅ Assistant messages include citations list (can be empty)

3. **created_at**:
   - ✅ UTC datetime when conversation_id first created
   - ✅ Immutable after creation

4. **last_updated**:
   - ✅ UTC datetime of most recent message appended
   - ✅ Updated with every new request/response cycle

**State Transitions**:

```
┌─────────────────────────────────────────────────────────┐
│  NEW CONVERSATION (conversation_id = null or missing)   │
│  ↓                                                      │
│  Backend generates UUID v4                              │
│  Creates ConversationState {                            │
│    conversation_id: UUID,                               │
│    messages: [user_message],                            │
│    created_at: now(),                                   │
│    last_updated: now()                                  │
│  }                                                      │
└────────────────────────┬────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  EXISTING CONVERSATION (conversation_id in request)    │
│  ↓                                                      │
│  Backend looks up ConversationState[conversation_id]   │
│  Appends user_message to messages list                  │
│  Updates last_updated = now()                           │
│  Appends assistant_message (after agent processing)    │
│  Returns ChatResponse with conversation_id             │
└────────────────────────┬────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────────┐
│  CONVERSATION RESET (user clicks "Reset")              │
│  ↓                                                      │
│  Backend deletes ConversationState[conversation_id]    │
│  Sets conversation_id = null in next request           │
│  Triggers NEW CONVERSATION flow above                  │
└─────────────────────────────────────────────────────────┘
```

**Lifecycle**:

1. **Birth**: Created when ChatRequest has null/missing conversation_id
2. **Growth**: New messages appended with each request (max ~50 messages per session recommended)
3. **Reset**: Conversation can be cleared by user clicking "Reset" (DELETE endpoint or frontend clears state)
4. **Death**: Discarded when browser tab closed or user navigates away (no persistence)

**Example ConversationState (in-memory)**:

```python
{
    "550e8400-e29b-41d4-a716-446655440000": {
        "conversation_id": "550e8400-e29b-41d4-a716-446655440000",
        "messages": [
            {
                "role": "user",
                "content": "What is URDF?",
                "timestamp": datetime(2025, 12, 23, 14, 30, 45),
                "citations": None
            },
            {
                "role": "assistant",
                "content": "URDF stands for Unified Robot Description Format. It's an XML format used in ROS to describe robot structure...",
                "timestamp": datetime(2025, 12, 23, 14, 30, 48),
                "citations": [
                    {
                        "module_name": "Module 2: Software Stack",
                        "chapter_id": "ch3-urdf-basics",
                        "source_url": "https://...",
                        "similarity_score": 0.95
                    }
                ]
            },
            {
                "role": "user",
                "content": "How do I create one?",
                "timestamp": datetime(2025, 12, 23, 14, 31, 10),
                "citations": None
            },
            {
                "role": "assistant",
                "content": "To create a URDF, you can use XML editor or Python libraries...",
                "timestamp": datetime(2025, 12, 23, 14, 31, 15),
                "citations": [...]
            }
        ],
        "created_at": datetime(2025, 12, 23, 14, 30, 45),
        "last_updated": datetime(2025, 12, 23, 14, 31, 15)
    }
}
```

---

## Entity 5: ChatMessage

**Purpose**: UI message object in React component representing a single message in conversation

**Scope**: Frontend-only (React state, not transmitted as-is to backend)

**TypeScript Interface**:

```typescript
interface ChatMessage {
  role: "user" | "assistant";
  content: string;
  timestamp: Date;
  citations: Citation[] | null;
}

interface Citation {
  module_name: string;
  chapter_id: string;
  source_url: string;
  similarity_score: number;
}
```

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `role` | "user" \| "assistant" | ✅ Yes | Determines message styling and layout |
| `content` | string | ✅ Yes | Non-empty message text (markdown allowed for assistant) |
| `timestamp` | Date | ✅ Yes | JavaScript Date object (UTC) |
| `citations` | Citation[] \| null | ✅ Yes | null for user messages, array for assistant messages |

**Validation Rules**:

1. **role**:
   - ✅ Must be exactly "user" or "assistant"
   - ✅ Determines UI layout (user right-aligned, assistant left-aligned)
   - ✅ Determines styling (user: blue bubble, assistant: gray bubble)

2. **content**:
   - ✅ Non-empty string after trimming
   - ✅ For assistant messages: can contain markdown formatting
   - ✅ Max display length: 10,000 characters (truncate if longer with "...see more" button)

3. **timestamp**:
   - ✅ JavaScript Date object (UTC timezone)
   - ✅ Used for chronological ordering in UI
   - ✅ Displayed in human-readable format (e.g., "2:45 PM")

4. **citations**:
   - ✅ null for user messages (no citations on user messages)
   - ✅ Citation[] for assistant messages
   - ✅ Empty array [] if no sources available
   - ✅ Rendered as clickable links below assistant message

**Example ChatMessage Objects**:

```typescript
// User message
{
  role: "user",
  content: "What is Isaac Sim?",
  timestamp: new Date("2025-12-23T14:30:45Z"),
  citations: null
}

// Assistant message with citations
{
  role: "assistant",
  content: "Isaac Sim is NVIDIA's physics simulation platform...",
  timestamp: new Date("2025-12-23T14:30:48Z"),
  citations: [
    {
      module_name: "Module 3: Isaac",
      chapter_id: "ch1-introduction-to-isaac",
      source_url: "https://docs.docusaurus.ai/module-3-isaac/ch1-introduction",
      similarity_score: 0.92
    }
  ]
}

// Assistant message without citations
{
  role: "assistant",
  content: "I don't have information about that in the textbook.",
  timestamp: new Date("2025-12-23T14:31:02Z"),
  citations: []
}
```

---

## Entity 6: ChatWidgetState

**Purpose**: Frontend React component state managing the chatbot widget UI and interactions

**Scope**: React component state (useState hook)

**TypeScript Interface**:

```typescript
interface ChatWidgetState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
}

interface ChatMessage {
  role: "user" | "assistant";
  content: string;
  timestamp: Date;
  citations: Citation[] | null;
}

interface Citation {
  module_name: string;
  chapter_id: string;
  source_url: string;
  similarity_score: number;
}
```

**Attributes**:

| Attribute | Type | Required | Constraints |
|-----------|------|----------|-------------|
| `messages` | ChatMessage[] | ✅ Yes | Ordered list of messages in current conversation |
| `isOpen` | boolean | ✅ Yes | true = widget expanded, false = widget collapsed |
| `isLoading` | boolean | ✅ Yes | true = waiting for backend response, false = idle |
| `error` | string \| null | ✅ Yes | null = no error, string = error message to display |
| `conversationId` | string \| null | ✅ Yes | null = new conversation, string = UUID of existing conversation |

**Validation Rules**:

1. **messages**:
   - ✅ Array of ChatMessage objects
   - ✅ Maintained in chronological order (oldest first)
   - ✅ Max ~100 messages before truncation warning (prevent memory bloat)
   - ✅ Loaded from sessionStorage on component mount if available

2. **isOpen**:
   - ✅ boolean - true to show widget, false to hide/collapse
   - ✅ Toggled by user clicking open/close button
   - ✅ Can be programmatically set when user selects text and triggers "Ask Chatbot"

3. **isLoading**:
   - ✅ boolean - true while waiting for backend response
   - ✅ Mutually exclusive with error (can't be loading and errored simultaneously)
   - ✅ Prevents user from submitting multiple messages while loading
   - ✅ Shows loading spinner/animation when true

4. **error**:
   - ✅ null when no error or error cleared
   - ✅ string with user-friendly error message when backend fails
   - ✅ Examples:
     - "Unable to connect to Q&A service. Please try again later."
     - "Backend error: Request timeout. Please try again."
     - "Invalid response from server. Please refresh page."
   - ✅ Auto-clear error after 5 seconds or when user submits new message

5. **conversationId**:
   - ✅ null for brand new conversation (backend generates UUID)
   - ✅ UUID v4 string for existing conversation
   - ✅ Persisted in sessionStorage for session lifetime
   - ✅ Cleared when user clicks "Reset Conversation"

**State Transitions**:

```
┌───────────────────────────────────────────────────────────┐
│ INITIAL STATE (Component Mount)                           │
│                                                           │
│ messages: [] (empty)                                      │
│ isOpen: false                                             │
│ isLoading: false                                          │
│ error: null                                               │
│ conversationId: null (from sessionStorage if available)   │
└────────────────────┬──────────────────────────────────────┘
                     │
         ┌───────────┴───────────┐
         ↓                       ↓
┌──────────────────────┐  ┌──────────────────────────┐
│ USER CLICKS OPEN     │  │ USER SELECTS TEXT        │
│                      │  │ (triggers context pass)  │
│ isOpen: false → true │  │                          │
│ error cleared        │  │ isOpen: false → true     │
└──────────────────────┘  │ (auto-expand widget)     │
                          └──────────────────────────┘
                     │
                     └───────────┬────────────────┐
                                 ↓                ↓
                    ┌──────────────────────┐  ┌─────────────────┐
                    │ USER TYPES MESSAGE   │  │ (widget ready   │
                    │ (updates input text) │  │  for interaction)
                    │                      │  └─────────────────┘
                    │ messages unchanged   │
                    │ isOpen: true         │
                    │ isLoading: false     │
                    │ error: null          │
                    └──────────┬───────────┘
                               │
                               ↓
                    ┌──────────────────────────┐
                    │ USER SUBMITS MESSAGE     │
                    │                          │
                    │ messages: [..., user]    │ ← Add user msg
                    │ isLoading: true          │ ← Disable input
                    │ error: null              │
                    │ conversationId: UUID     │ ← Generate or use existing
                    └──────────┬───────────────┘
                               │
                    HTTP POST /api/chat/query
                               │
                ┌──────────────┴──────────────┐
                ↓                             ↓
    ┌───────────────────────────┐  ┌────────────────────┐
    │ BACKEND SUCCESS (200)     │  │ BACKEND ERROR      │
    │                           │  │ (4xx, 5xx, timeout)│
    │ messages: [..., user,     │  │                    │
    │           assistant]      │  │ messages: [...,    │
    │ isLoading: false          │  │          user]     │
    │ error: null               │  │ isLoading: false   │
    │ conversationId: UUID      │  │ error: "msg"       │
    │                           │  │ conversationId:    │
    │ (display answer + cite)   │  │ (persisted or null)│
    │                           │  │                    │
    │ (show citations as links) │  │ (show retry button)│
    └───────────────────────────┘  └────────────────────┘
         │                               │
         └───────────┬───────────────────┘
                     │
        ┌────────────┴────────────┐
        ↓                         ↓
    ┌────────────────┐  ┌─────────────────────┐
    │ USER ASKS      │  │ USER CLICKS RESET   │
    │ FOLLOW-UP      │  │                     │
    │ (repeat cycle) │  │ messages: []        │
    │                │  │ isOpen: true        │
    │ Include conv   │  │ isLoading: false    │
    │ _id to backend │  │ error: null         │
    └────────────────┘  │ conversationId: null│
                        │ (clear sessionStorage)
                        └─────────────────────┘
```

**Example React Implementation**:

```typescript
import { useState, useEffect } from 'react';

interface ChatWidgetState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
}

export function ChatbotWidget() {
  const [state, setState] = useState<ChatWidgetState>({
    messages: [],
    isOpen: false,
    isLoading: false,
    error: null,
    conversationId: null,
  });

  // Load conversation from sessionStorage on mount
  useEffect(() => {
    const saved = sessionStorage.getItem('chatbotState');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        setState(parsed);
      } catch (e) {
        console.warn('Failed to restore chat state', e);
      }
    }
  }, []);

  // Persist state to sessionStorage
  useEffect(() => {
    sessionStorage.setItem('chatbotState', JSON.stringify(state));
  }, [state]);

  const handleToggle = () => {
    setState(prev => ({ ...prev, isOpen: !prev.isOpen }));
  };

  const handleSubmitMessage = async (query: string, context?: string) => {
    // Add user message
    const userMsg: ChatMessage = {
      role: 'user',
      content: query,
      timestamp: new Date(),
      citations: null,
    };

    setState(prev => ({
      ...prev,
      messages: [...prev.messages, userMsg],
      isLoading: true,
      error: null,
    }));

    try {
      // Send to backend
      const response = await fetch('http://localhost:8000/api/chat/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query,
          conversation_id: state.conversationId,
          context: context || undefined,
        }),
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.statusText}`);
      }

      const data = await response.json();

      // Add assistant message
      const assistantMsg: ChatMessage = {
        role: 'assistant',
        content: data.answer,
        timestamp: new Date(),
        citations: data.citations || [],
      };

      setState(prev => ({
        ...prev,
        messages: [...prev.messages, assistantMsg],
        isLoading: false,
        conversationId: data.conversation_id,
      }));
    } catch (err) {
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: err instanceof Error ? err.message : 'Unknown error',
      }));
    }
  };

  const handleReset = () => {
    setState({
      messages: [],
      isOpen: true,
      isLoading: false,
      error: null,
      conversationId: null,
    });
  };

  return (
    <div className="chatbot-widget">
      {/* Toggle button */}
      <button onClick={handleToggle} className="chatbot-toggle">
        {state.isOpen ? 'Close' : 'Chat'}
      </button>

      {/* Widget window */}
      {state.isOpen && (
        <div className="chatbot-window">
          {/* Messages */}
          <div className="messages">
            {state.messages.map((msg, idx) => (
              <div key={idx} className={`message ${msg.role}`}>
                <p>{msg.content}</p>
                {msg.citations && msg.citations.length > 0 && (
                  <div className="citations">
                    {msg.citations.map((cite, cidx) => (
                      <a
                        key={cidx}
                        href={cite.source_url}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {cite.module_name} - {cite.chapter_id}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}
          </div>

          {/* Loading indicator */}
          {state.isLoading && <div className="loading">Loading...</div>}

          {/* Error message */}
          {state.error && <div className="error">{state.error}</div>}

          {/* Input */}
          <input
            type="text"
            placeholder="Ask a question..."
            disabled={state.isLoading}
            onKeyDown={e => {
              if (e.key === 'Enter' && e.currentTarget.value) {
                handleSubmitMessage(e.currentTarget.value);
                e.currentTarget.value = '';
              }
            }}
          />

          {/* Reset button */}
          <button onClick={handleReset} disabled={state.isLoading}>
            Reset
          </button>
        </div>
      )}
    </div>
  );
}
```

---

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                     USER INTERACTION FLOW                        │
└──────────────────────────────────────────────────────────────────┘

1. USER TYPES QUESTION IN CHATBOT WIDGET
   │
   ├─ Frontend: Update ChatWidgetState.messages (add user ChatMessage)
   ├─ Frontend: Set isLoading = true
   ├─ Frontend: Get conversationId from state (null or UUID)
   │
   └─ Convert to ChatRequest JSON
      ├─ query: string (from user input)
      ├─ conversation_id: string | null
      └─ context: string | null (optional, from selected text)

2. HTTP POST TO /api/chat/query
   │
   ├─ Validate ChatRequest
   │  ├─ Check query non-empty and <= 500 chars
   │  ├─ Check context <= 2000 chars if provided
   │  └─ Validate conversation_id UUID format if provided
   │
   ├─ Backend: Generate conversation_id if null
   │
   ├─ Backend: Look up or create ConversationState
   │  ├─ Append user message to state.messages
   │  └─ Update state.last_updated = now()
   │
   ├─ Call agent.py.generate_answer(query, context, message_history)
   │  └─ Returns: answer, citations (from retrieve_context)
   │
   ├─ Append assistant message to ConversationState.messages
   │
   └─ Format ChatResponse JSON
      ├─ answer: string
      ├─ citations: List[Citation]
      ├─ conversation_id: UUID (same as request)
      └─ metadata: { retrieval_count, tokens_used, grounded }

3. HTTP RESPONSE (200 OK)
   │
   ├─ Frontend: Parse ChatResponse JSON
   ├─ Frontend: Create ChatMessage from response
   │  ├─ role: "assistant"
   │  ├─ content: response.answer
   │  ├─ timestamp: new Date()
   │  └─ citations: response.citations
   │
   ├─ Frontend: Update ChatWidgetState
   │  ├─ Append ChatMessage to messages
   │  ├─ Set conversationId = response.conversation_id
   │  ├─ Set isLoading = false
   │  └─ Clear error
   │
   └─ Render: Display answer + citations in chat UI

4. MULTI-TURN FOLLOW-UP
   │
   ├─ User asks follow-up question
   ├─ Frontend: Include conversationId from state
   ├─ Backend: Look up ConversationState[conversationId]
   ├─ Backend: Pass full message history to agent
   │  └─ agent.py maintains context across turns
   └─ Repeat from step 2

5. ERROR HANDLING (4xx or 5xx response)
   │
   ├─ Frontend: Catch error response
   ├─ Frontend: Extract error message
   ├─ Frontend: Update ChatWidgetState
   │  ├─ Set isLoading = false
   │  ├─ Set error = "error message"
   │  └─ Keep messages and conversationId intact
   │
   └─ Render: Display error message with retry button
```

---

## Validation Summary Table

| Entity | Validation Layer | Rules |
|--------|------------------|-------|
| **ChatRequest** | Frontend (client-side) + Backend (server-side) | Non-empty query, max 500 chars; UUID format for conversation_id; max 2000 chars context |
| **ChatResponse** | Backend (before sending) | Non-empty answer; valid Citation objects; UUID for conversation_id; metadata always present |
| **Citation** | Backend (from agent.py) | Valid module_name; kebab-case chapter_id; HTTPS URL; similarity_score 0-1 |
| **ConversationState** | Backend (memory) | UUID key; chronological messages; immutable created_at; updated last_updated |
| **ChatMessage** | Frontend (React state) | Valid role enum; non-empty content; Date timestamp; null or Citation[] citations |
| **ChatWidgetState** | Frontend (React state) | Array of ChatMessage; boolean flags; null or UUID conversation_id; error string or null |

---

## Persistence & Storage

| Entity | Storage | Lifetime | Scope |
|--------|---------|----------|-------|
| **ChatRequest** | HTTP request body | Transient (milliseconds) | HTTP boundary only |
| **ChatResponse** | HTTP response body | Transient (milliseconds) | HTTP boundary only |
| **Citation** | Embedded in ChatResponse | Transient (session) | From backend → frontend |
| **ConversationState** | Python dict (in-memory) | Browser session | Backend-only |
| **ChatMessage** | React state + sessionStorage | Browser session | Frontend-only |
| **ChatWidgetState** | React state + sessionStorage | Browser session | Frontend-only |

**Notes**:
- No database persistence (out of scope)
- Browser refresh clears frontend state but doesn't affect backend state for that conversation_id
- Backend state discarded when process exits
- sessionStorage used for client-side persistence of ChatWidgetState and messages

---

## Related Documents

- **spec.md**: Feature specification and user stories
- **plan.md**: Implementation architecture and structure decisions
- **contracts/api-schema.yaml**: OpenAPI schema for REST endpoints
- **quickstart.md**: Setup and usage instructions
- **tasks.md**: Implementation task breakdown (created by `/sp.tasks`)

---

**Document Status**: Complete | **Last Updated**: 2025-12-23 | **Author**: Agent (Claude Code)
