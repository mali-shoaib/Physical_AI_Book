# Research Document: 010-Docusaurus-Chatbot Integration

**Feature**: Frontend-Backend Integration of RAG Chatbot in Docusaurus Book
**Branch**: 010-docusaurus-chatbot
**Date**: 2025-12-23
**Status**: Research Phase

## Overview

This document captures research and decision-making for 7 critical architectural questions in integrating the FastAPI-based RAG backend (feature 009) with the Docusaurus React frontend. Each question includes the decision made, rationale, and alternatives considered to support all 3 user stories while minimizing complexity and maximizing reuse of existing code.

---

## 1. FastAPI Backend Architecture: How to Wrap agent.py

### Question
How should the FastAPI backend be structured to wrap the existing `agent.py` module from feature 009 and expose it as REST API endpoints?

### Decision
**Create a thin FastAPI application with a single module file (`app.py`) that:**
1. Imports the agent executor and retrieval functions from feature 009's `agent.py`
2. Wraps conversation state in a simple in-memory dict keyed by `conversation_id`
3. Exposes REST endpoints that map HTTP requests directly to agent function calls
4. Returns standardized JSON responses with embedded citations from retrieval results

**File structure**:
```
backend/
├── main.py                 # FastAPI app initialization and CORS config
├── routes/
│   └── chat.py            # POST /api/chat/query endpoint
├── models.py              # Pydantic request/response schemas
├── state.py               # Conversation state management (dict-based)
└── requirements.txt       # FastAPI, Pydantic, python-dotenv, etc.
```

### Rationale
This minimalist approach:
- **Maximizes code reuse**: No re-implementing the agent logic; we directly call existing functions from feature 009
- **Simplicity for local dev**: Single dict-based state management avoids the complexity of Redis, databases, or message queues
- **Aligns with constraints**: The spec explicitly states "no chat history persistence" and "local development only", so in-memory state is sufficient
- **Fast integration**: Developers can launch `uvicorn main.py` and start testing within minutes
- **Supports all user stories**: Single `/api/chat/query` endpoint handles US1 (basic queries), US2 (context via conversation_id), and US3 (context via optional `context` field)

### Alternatives Considered

**Alternative 1: Full-featured microservice with persistence**
- *Pros*: Scalable, production-ready, supports cross-device sessions, database-backed history
- *Cons*: Over-engineered for local dev scope, requires Redis/PostgreSQL setup, adds 10+ hours of work, violates "no persistence" constraint
- *Verdict*: Rejected – Spec explicitly excludes persistence and production deployment

**Alternative 2: Serverless/AWS Lambda wrapper**
- *Pros*: Auto-scaling, pay-per-use, deployed ready
- *Cons*: Incompatible with local-only requirement, adds AWS credentials/IAM complexity, deployment overhead conflicts with 1-week timeline
- *Verdict*: Rejected – Spec requires local development environment; cloud adds deployment burden

**Alternative 3: Direct node-to-node integration (no separate backend)**
- *Pros*: Zero network latency, single deployment unit
- *Cons*: Requires Python library in Node.js (complex wrapper), loses architectural separation, couples frontend and backend
- *Verdict*: Rejected – Spec explicitly calls for "FastAPI backend" and "REST API"; clean separation supports testing and future scaling

---

## 2. API Endpoint Design: Single vs Multiple Endpoints

### Question
Should the backend expose a single multi-purpose endpoint (e.g., `/api/chat/query`) or multiple specialized endpoints (e.g., `/api/chat/query`, `/api/chat/context`, `/api/chat/reset`)?

### Decision
**Use a single POST endpoint `/api/chat/query` with optional fields in the request payload**

**Endpoint specification**:
```
POST /api/chat/query
Content-Type: application/json

{
  "query": "What is Isaac Sim?",           # Required
  "conversation_id": "uuid-123",           # Optional (generate if missing)
  "context": "Selected paragraph text...", # Optional (for US3)
  "reset": false                           # Optional (reset conversation if true)
}

200 OK Response:
{
  "answer": "Isaac Sim is...",
  "citations": [
    {
      "module": "module-3-isaac",
      "chapter": "ch1-intro",
      "url": "/docs/module-3-isaac/ch1-intro",
      "relevance_score": 0.92
    }
  ],
  "conversation_id": "uuid-123",
  "metadata": {
    "retrieval_count": 3,
    "tokens_used": 245
  }
}
```

### Rationale
A single endpoint with optional fields:
- **Minimizes API surface**: Easier to test, document, and maintain (all query logic in one place)
- **Supports all use cases**: Optional `context` field handles US3 (selected text), optional `conversation_id` enables US2 (multi-turn), basic `query` covers US1
- **REST-compliant**: Single POST endpoint with stateless responses follows REST principles for idempotent, cacheable operations
- **Client-side simplicity**: Frontend makes one type of request; no logic to choose between different endpoints
- **Backward compatible**: Adding new optional fields later (e.g., `temperature`, `model`) doesn't break existing clients
- **Matches spec requirements**: FR-002 and FR-003 explicitly define this single-endpoint contract

### Alternatives Considered

**Alternative 1: Multiple specialized endpoints** (`/query`, `/context`, `/reset`, `/history`)
- *Pros*: Explicit semantics, easier to scale individual operations, clearer separation of concerns
- *Cons*: Frontend must implement routing logic to choose endpoints, requires more documentation, harder to maintain backward compatibility when adding features
- *Verdict*: Rejected – Over-engineered for a single query operation; single endpoint sufficient with optional fields

**Alternative 2: GraphQL mutation-based interface**
- *Pros*: Flexible query language, self-documenting schema, excellent for varying response shapes
- *Cons*: Overkill for simple request/response, GraphQL requires new client library (Apollo Client or similar), adds frontend complexity, not needed for this linear conversation model
- *Verdict*: Rejected – REST is simpler and matches spec; GraphQL doesn't solve any real problem here

**Alternative 3: WebSocket-based streaming**
- *Pros*: Real-time answer streaming (show answer as it's generated), bidirectional communication, can push updates
- *Cons*: Spec says "no real-time features"; WebSocket adds connection state complexity, requires frontend WebSocket library, harder to test, doesn't improve perceived latency (p95 target is 10 seconds anyway)
- *Verdict*: Rejected – Spec explicitly excludes real-time; HTTP request/response sufficient and easier to debug

---

## 3. React Component Integration Approach: Theme Swizzling vs Plugin

### Question
How should the chatbot React component be integrated into the Docusaurus site? Via Swizzling (wrapping Docusaurus components) or as a standalone plugin (custom component in theme)?

### Decision
**Use Docusaurus theme swizzling + custom component approach:**
1. Create a custom React component `ChatbotWidget` in `src/components/ChatbotWidget.jsx`
2. Swap out the default Docusaurus `Layout` component via theme swizzling to inject `ChatbotWidget` at the bottom of every page
3. Use Docusaurus Infima CSS for styling (no new CSS frameworks)
4. Leverage Docusaurus's built-in hooks (`useLocation()`, `useWindowSize()`) for responsive behavior

**File structure**:
```
src/
├── components/
│   └── ChatbotWidget.jsx     # Main chatbot component (state, UI, API calls)
├── theme/
│   └── Layout/index.js        # Swizzled Layout wrapping ChatbotWidget
└── css/
    └── chatbot.module.css     # Styles using Infima variables
```

### Rationale
Theme swizzling for integration:
- **Leverages Docusaurus ecosystem**: Built-in swizzling mechanism is designed exactly for this use case; no fighting the framework
- **Minimal boilerplate**: No plugin configuration needed; just override `Layout` component
- **Automatic on all pages**: Swizzled Layout wraps every page automatically; no manual mounting needed on each route
- **Access to Docusaurus utilities**: Can use `useLocation()` (get current page), `useSidebarBreadcrumbs()` (show context), `useWindowSize()` (responsive design)
- **Easy to disable/enable**: Can wrap ChatbotWidget with feature flag or environment variable
- **Reuses existing styles**: Infima CSS framework already in Docusaurus; no new dependencies
- **Supports US3**: Can access selected text anywhere in the DOM since ChatbotWidget is global

### Alternatives Considered

**Alternative 1: Standalone plugin (esbuild plugin or Docusaurus plugin)**
- *Pros*: Isolated from core Docusaurus, can be published as npm package, explicit plugin configuration
- *Cons*: Requires plugin interface (complex), need to handle mounting/unmounting on all pages manually, harder to debug, plugin routing can be fragile
- *Verdict*: Rejected – Swizzling is the idiomatic Docusaurus way; plugins add unnecessary indirection

**Alternative 2: Inject via docusaurus.config.js (clientModule)**
- *Pros*: Centralized configuration, no swizzling needed
- *Cons*: Client modules are harder to debug, limited access to React context/hooks, timing of injection can be unpredictable
- *Verdict*: Rejected – Swizzling with Layout override gives better control and debugging

**Alternative 3: iFrame-based widget (like Intercom or Zendesk)**
- *Pros*: Complete isolation from Docusaurus styles, can be dropped into any site, no styling conflicts
- *Cons*: iFrame adds network request overhead, can't easily access page context (selected text, URL), CORS headers required, worse mobile experience (iFrame resizing is buggy), feels detached from page
- *Verdict*: Rejected – Not needed; we own the Docusaurus site and can style properly. iFrame complexity unjustified.

---

## 4. Conversation State Management: Where to Store It (Frontend sessionStorage vs Backend Dict)

### Question
Should conversation state (message history, conversation_id, context) be maintained on the frontend (browser's sessionStorage) or backend (in-memory dict)?

### Decision
**Hybrid approach: Store state on backend (in-memory dict), use frontend sessionStorage for caching conversation_id**

**Backend state management** (`backend/state.py`):
```python
# In-memory store: {conversation_id: {"messages": [...], "created_at": ...}}
conversations = {}

def get_conversation(conversation_id: str) -> dict:
    return conversations.get(conversation_id, {"messages": []})

def add_message(conversation_id: str, role: str, content: str):
    if conversation_id not in conversations:
        conversations[conversation_id] = {"messages": [], "created_at": time.time()}
    conversations[conversation_id]["messages"].append({
        "role": role,
        "content": content,
        "timestamp": time.time()
    })

def reset_conversation(conversation_id: str):
    if conversation_id in conversations:
        del conversations[conversation_id]
```

**Frontend caching** (`src/components/ChatbotWidget.jsx`):
```javascript
// Store conversation_id in sessionStorage to persist across page navigations
const [conversationId, setConversationId] = useState(
  () => sessionStorage.getItem("chatbot_conversation_id") || generateUUID()
);

useEffect(() => {
  sessionStorage.setItem("chatbot_conversation_id", conversationId);
}, [conversationId]);

// Render conversation history stored in React state (NOT sessionStorage)
const [messages, setMessages] = useState([]);
```

### Rationale
Hybrid backend + sessionStorage caching:
- **Backend truth**: State of record for multi-turn context; if frontend crashes or page reloads, backend has the conversation history
- **SessionStorage caching**: Persists conversation_id across page navigations (user stays in same conversation when browsing), avoids repeated lookups
- **SessionStorage limitations**: Browser session ends when user closes tab; aligns with "conversation lifetime" constraint in spec (conversations persist only during session)
- **No database complexity**: In-memory dict is sufficient for local dev scope; cleared on backend restart (acceptable per spec)
- **Supports all user stories**:
  - US1: Single query needs backend state to track conversation
  - US2: Multi-turn requires backend to store message history and context
  - US3: Selected text passed in payload; backend stores in message history
- **Failure resilience**: Frontend reload doesn't lose conversation (backend still has it); backend restart loses conversation (acceptable for local dev)

### Alternatives Considered

**Alternative 1: Frontend-only (sessionStorage/localStorage)**
- *Pros*: Zero backend complexity, client-side rendering, frontend can work offline, no network latency for state lookup
- *Cons*: Browser DevTools can see entire conversation history (privacy concern), message history bloats JSON payload sent with each request, harder to debug (state split across frontend and network), doesn't support US2 well (pronoun resolution in agent requires full context)
- *Verdict*: Rejected – Agent needs full conversation context on backend to resolve pronouns and provide coherent follow-ups. Shipping full history in payloads is inefficient.

**Alternative 2: Backend PostgreSQL/MongoDB persistence**
- *Pros*: Survives backend restart, enables future analytics/debugging, can support multiple devices
- *Cons*: Violates spec's "no persistence" constraint, requires database setup (adds 2-3 hours), overkill for local dev, migrations overhead
- *Verdict*: Rejected – Spec explicitly excludes persistence; in-memory sufficient for 1-week local development

**Alternative 3: Frontend Redux/Zustand + sessionStorage (no backend state)**
- *Pros*: Centralized frontend state management, Redux DevTools for debugging
- *Cons*: Agent on backend can't see full conversation (requires sending entire history in each request), bloats payloads, doesn't scale to 50+ turns, harder to implement context window management in agent
- *Verdict*: Rejected – Defeats purpose of backend state; agent needs history without client re-sending it each time

---

## 5. CORS Configuration for Local Development

### Question
How should CORS be configured to allow the Docusaurus frontend (localhost:3000) to communicate with the FastAPI backend (localhost:8000)?

### Decision
**Use FastAPI's `CORSMiddleware` with lenient local development settings**

**Implementation** (`backend/main.py`):
```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# Local development CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
    expose_headers=["X-Conversation-ID"],  # Custom header for conversation tracking
    max_age=3600,  # Cache preflight response for 1 hour
)
```

**Frontend fetch call** (`src/components/ChatbotWidget.jsx`):
```javascript
const response = await fetch("http://localhost:8000/api/chat/query", {
  method: "POST",
  headers: {
    "Content-Type": "application/json",
  },
  credentials: "include",  // Include cookies if needed later
  body: JSON.stringify({
    query: userInput,
    conversation_id: conversationId,
    context: selectedText,
  }),
});
```

### Rationale
Lenient CORS for local development:
- **Required for browser security**: Fetch API enforces CORS; without proper headers, requests fail silently
- **Whitelist localhost origins**: Allows both `localhost` and `127.0.0.1` (some systems resolve differently)
- **Allow credentials**: Enables future support for authentication tokens if needed
- **Expose custom headers**: Frontend can read `X-Conversation-ID` from response (optional, for debugging)
- **Preflight caching**: `max_age=3600` reduces OPTIONS preflight requests during development
- **GET and POST methods**: GET for future health checks, POST for queries

### Alternatives Considered

**Alternative 1: No CORS (disable all checks)**
- *Pros*: Simplest configuration, requests always work
- *Cons*: Breaks browser security model, won't work in production, not good practice even for dev, makes developers sloppy about CORS later
- *Verdict*: Rejected – Bad habit; proper CORS handling from day 1 prevents issues when scaling

**Alternative 2: Proxy through Docusaurus build (no direct backend calls)**
- *Pros*: Single origin (no CORS needed), can cache responses, easier deployment
- *Cons*: Adds complexity to Docusaurus webpack config, breaks live reload if backend restarts, makes debugging harder (network calls hidden), adds latency
- *Verdict*: Rejected – Direct backend calls are cleaner for development; proxy adds complexity that's premature

**Alternative 3: Use jsonp or serverless proxy**
- *Pros*: Works without CORS, can bypass browser restrictions
- *Cons*: JSONP is deprecated for good reasons (security, no error handling), serverless proxy adds network hop and latency, defeats local-first goal
- *Verdict*: Rejected – CORS is the standard; using it properly is better than workarounds

---

## 6. Error Handling Strategy: HTTP Status Codes and JSON Error Format

### Question
How should the backend handle and report errors to the frontend? What HTTP status codes and error response format should be used?

### Decision
**Use standard HTTP status codes with consistent JSON error format**

**Error response format** (`backend/models.py`):
```python
from pydantic import BaseModel
from typing import Optional, List

class ErrorResponse(BaseModel):
    """Standard error response format"""
    error: str                    # Machine-readable error code (e.g., "BAD_REQUEST")
    message: str                  # Human-readable error message for user
    details: Optional[str] = None # Debug details (only in dev mode)
    path: Optional[str] = None    # Request path that failed
    timestamp: float              # ISO 8601 timestamp

class ErrorDetail(BaseModel):
    """Detail per field for validation errors"""
    field: str
    message: str
```

**HTTP status codes and scenarios** (`backend/routes/chat.py`):
```python
# 200 OK - Success
{
  "answer": "...",
  "citations": [...],
  "conversation_id": "..."
}

# 400 Bad Request - Invalid query
{
  "error": "BAD_REQUEST",
  "message": "Query is required and cannot be empty"
}

# 414 URI Too Long - Query exceeds limits
{
  "error": "QUERY_TOO_LONG",
  "message": "Query exceeds 500 character limit. Please ask a more concise question"
}

# 429 Too Many Requests - Rate limited
{
  "error": "RATE_LIMITED",
  "message": "Too many requests. Please wait before sending another query"
}

# 503 Service Unavailable - Backend failure
{
  "error": "AGENT_ERROR",
  "message": "Unable to generate answer. Please try again later"
}

# 500 Internal Server Error - Unexpected failure
{
  "error": "INTERNAL_ERROR",
  "message": "An unexpected error occurred. Please check server logs"
}
```

**Frontend error handling** (`src/components/ChatbotWidget.jsx`):
```javascript
try {
  const response = await fetch("/api/chat/query", { ... });

  if (!response.ok) {
    const error = await response.json();

    if (response.status === 429) {
      showUserMessage("Too many requests. Please wait a moment.");
    } else if (response.status === 503) {
      showUserMessage("Q&A service is temporarily unavailable. Please try again.");
    } else if (response.status === 400 || response.status === 414) {
      showUserMessage(error.message);
    } else {
      showUserMessage("Unexpected error. Please check console.");
    }
    return;
  }

  // Handle 200 OK
  const data = await response.json();
  addBotMessage(data.answer, data.citations);

} catch (err) {
  // Network error
  showUserMessage("Unable to connect to Q&A service. Please try again later.");
}
```

### Rationale
Standard HTTP status codes + JSON format:
- **Follows REST conventions**: HTTP status codes are universally understood; 400 = client error, 500 = server error, 429 = rate limited
- **Machine-readable codes**: `error` field allows frontend to implement specific UI responses (retry logic for 429, etc.)
- **User-friendly messages**: `message` field is safe to display to users (no technical jargon or secrets)
- **Optional debug details**: `details` field omitted in production but available in dev mode for debugging
- **Consistent across all endpoints**: Same error format everywhere; frontend can build generic error handler
- **Supports all user stories**:
  - US1: Handles out-of-scope questions with 400 Bad Request
  - US2: Handles malformed conversation_id with 400 Bad Request
  - US3: Query with context handles 414 if total exceeds limits
- **Aligns with spec**: FR-012 requires graceful error display; this format enables it

### Alternatives Considered

**Alternative 1: Plain text error responses**
- *Pros*: Simple, no parsing needed
- *Cons*: Frontend can't distinguish error types, no structured metadata, hard to implement retry logic, can't localize messages
- *Verdict*: Rejected – JSON provides essential structure for robust error handling

**Alternative 2: HTML error pages (like Django default errors)**
- *Pros*: Works in browser (can visit URL directly to debug)
- *Cons*: Frontend can't parse HTML, requires Content-Type switching, violates REST (returns different media types), hard to version
- *Verdict*: Rejected – API should always return JSON; HTML responses break client assumptions

**Alternative 3: Custom error codes (1000, 2000, 3000 ranges)**
- *Pros*: Custom domain-specific codes can be more expressive
- *Cons*: Frontend developers need documentation to understand codes, loses benefits of standard HTTP status codes, inconsistent with HTTP spec, harder to debug with standard tools
- *Verdict*: Rejected – HTTP status codes are sufficient and standard; custom codes add confusion

---

## 7. Citation Display in UI: How to Format and Show Sources

### Question
How should citations (source references) be formatted and displayed in the chatbot UI to show users where answers come from?

### Decision
**Display citations inline below bot message with clickable links to source chapters**

**Citation format and display**:
```
Bot response:
"Isaac Sim is a suite of tools for simulation-based robotics development. It provides physics engines, sensor simulation, and digital twin capabilities..."

Citations (below message):
┌─ Sources ─────────────────────────────────┐
│ 📚 Module 3: Isaac → Chapter 1: Introduction     │
│    (Relevance: 92%) [View in textbook]            │
│                                                    │
│ 📚 Module 3: Isaac → Chapter 6: Capstone         │
│    (Relevance: 87%) [View in textbook]            │
└────────────────────────────────────────────┘

// Links: /docs/module-3-isaac/ch1-intro, /docs/module-3-isaac/ch6-capstone
```

**Frontend component** (`src/components/CitationBlock.jsx`):
```javascript
export function CitationBlock({ citations }) {
  return (
    <div className="citations-block">
      <div className="citations-header">Sources</div>
      {citations.map((citation, idx) => (
        <div key={idx} className="citation-item">
          <div className="citation-path">
            📚 {citation.module} → {citation.chapter}
          </div>
          <div className="citation-metadata">
            Relevance: {(citation.relevance_score * 100).toFixed(0)}%
          </div>
          <a
            href={citation.url}
            target="_blank"
            className="citation-link"
          >
            View in textbook
          </a>
        </div>
      ))}
    </div>
  );
}
```

**Styling** (`src/css/chatbot.module.css`):
```css
.citations-block {
  margin-top: 12px;
  padding: 12px;
  background-color: var(--ifm-background-surface-secondary);
  border-left: 3px solid var(--ifm-color-info);
  border-radius: 4px;
  font-size: 0.875rem;
}

.citations-header {
  font-weight: 600;
  margin-bottom: 8px;
  color: var(--ifm-heading-color);
}

.citation-item {
  margin-bottom: 8px;
  padding-bottom: 8px;
  border-bottom: 1px solid var(--ifm-color-emphasis-200);
}

.citation-path {
  font-weight: 500;
  color: var(--ifm-heading-color);
}

.citation-metadata {
  font-size: 0.8rem;
  color: var(--ifm-color-emphasis-700);
  margin-top: 2px;
}

.citation-link {
  color: var(--ifm-color-info);
  text-decoration: none;
  font-weight: 500;
}

.citation-link:hover {
  text-decoration: underline;
}
```

### Rationale
Inline citations with clickable links:
- **Transparency and trust**: Shows users exactly where answers come from; critical for academic/reference material
- **Relevance scores**: Helps users understand how confident the system is (92% > 87%); shows ranking
- **Clickable links**: Readers can drill down into full chapter context; supports learning and verification
- **Minimal UI clutter**: Compact format doesn't overwhelm small chatbot widget; citations appear only when present
- **Docusaurus integration**: Uses existing module/chapter structure from spec; URLs map directly to Docusaurus routes
- **Supports all user stories**:
  - US1: Basic answers include citations (spec requirement FR-003)
  - US2: Multi-turn answers cite relevant passages
  - US3: Selected text context may produce highly relevant citations (shows context helped)
- **Accessible**: Semantic structure (headings, links) works with screen readers
- **Reuses Infima**: Uses CSS variables (`--ifm-color-*`) for consistency with Docusaurus theme

### Alternatives Considered

**Alternative 1: Superscript numbers with footnotes**
- *Pros*: Compact, doesn't interrupt answer text, familiar from academic papers
- *Cons*: Requires clicking to see sources, not obvious they're clickable, small numbers hard to see on mobile, doesn't show relevance scores
- *Verdict*: Rejected – Loses transparency; inline format is clearer for web UI

**Alternative 2: Separate "Sources" tab or modal**
- *Pros*: Doesn't clutter answer UI, can show detailed metadata (retrieval count, token usage)
- *Cons*: Users must click to verify sources (reduces trust), breaks flow of reading, modal overhead on mobile, extra cognitive load
- *Verdict*: Rejected – Citations should be visible by default; hiding them reduces transparency

**Alternative 3: Highlight quoted text in answer + show source**
- *Pros*: Shows exact passage being cited, very transparent
- *Cons*: Complex implementation (requires parsing answer to identify cited passages), can make answer hard to read (too many colors/highlights), doesn't work if agent paraphrases (which it often does)
- *Verdict*: Rejected – Agent may paraphrase content, not always quote directly; highlights would be incomplete

**Alternative 4: Citation badges in message (e.g., [1] [2] [3])**
- *Pros*: Compact, doesn't take space below answer
- *Cons*: Number format outdated on web, requires mapping to sources elsewhere, can make text hard to read, less discoverable
- *Verdict*: Rejected – Inline citations below message is clearer and more native to web UI

---

## Summary: Decisions Across All 7 Questions

| Question | Decision | Key Benefit |
|----------|----------|------------|
| **1. FastAPI Architecture** | Thin wrapper around agent.py + dict-based state | Minimal code, maximum reuse |
| **2. API Endpoints** | Single `/api/chat/query` with optional fields | Simple, flexible, REST-compliant |
| **3. Component Integration** | Docusaurus theme swizzling (override Layout) | Automatic on all pages, minimal setup |
| **4. State Management** | Backend dict (truth) + sessionStorage (cache) | Stateless across page reloads, no DB |
| **5. CORS Config** | FastAPI CORSMiddleware with localhost whitelist | Standard browser security, development-friendly |
| **6. Error Handling** | HTTP status codes + consistent JSON format | Robust, debuggable, user-friendly |
| **7. Citation Display** | Inline citations below message with links | Transparent, clickable, mobile-friendly |

## Cross-Cutting Concerns

### Supports All 3 User Stories
- **US1** (Interactive Query): Covered by single endpoint, citations display, error handling
- **US2** (Follow-up Context): Covered by conversation_id state management, backend message history
- **US3** (Selected Text Context): Covered by optional `context` field in payload, no special endpoint needed

### Minimizes New Code
- Reuses agent.py from feature 009 (no agent logic rewrite)
- Reuses Docusaurus components and Infima CSS (no new CSS framework)
- Reuses React hooks (useLocation, useWindowSize) built into Docusaurus
- No complex state management library (Redux, Zustand) needed

### Keeps Local Dev Simple
- In-memory state; no database setup
- CORS config is straightforward (3 lines in FastAPI)
- Single API endpoint to test
- No authentication or session management

### Follows REST Best Practices
- Stateless HTTP requests (state in URL or body, not headers)
- Idempotent queries (same conversation_id + query = same answer unless context changes)
- Proper HTTP status codes (400, 429, 503, etc.)
- Standard Content-Type (application/json)
- Meaningful response structure (answer + citations + metadata)

---

## Open Questions for Clarification

1. **Agent context window limits**: How many turns should a conversation store? (Proposal: Unlimited during session; old messages truncated if total tokens > 8000)
2. **Rate limiting**: Should backend implement rate limiting per conversation or per IP? (Proposal: None for local dev; flag for future)
3. **Relevance score visibility**: Should relevance scores be shown to users? (Proposal: Yes, as percentage; helps users understand confidence)
4. **Long answers**: How long can answers be? Any truncation? (Proposal: No truncation; let agent control length)
5. **Mobile chatbot size**: Should chatbot be full-screen on mobile or floating widget? (Proposal: Floating widget; toggle full-screen mode)

---

## Next Steps

1. **Validate decisions** with team lead; no blocking concerns?
2. **Create ADRs** for decisions 1, 3, 4 (most architecturally significant)
3. **Begin implementation** with plan.md; start with FastAPI backend setup
4. **Set up local CORS test** early in development (before React component)
5. **Mock backend** in tests before real agent integration

---

## References

- Spec: `/specs/010-docusaurus-chatbot/spec.md` (User stories, requirements)
- Feature 009: `009-retrieval-agent` (Agent SDK, retrieval functions)
- Docusaurus docs: https://docusaurus.io/docs/swizzling
- FastAPI CORS: https://fastapi.tiangolo.com/tutorial/cors/
- REST API best practices: https://restfulapi.net/http-status-codes/
