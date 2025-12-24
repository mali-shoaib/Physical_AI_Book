# Quickstart Guide: Docusaurus Chatbot Integration

**Feature**: 010-docusaurus-chatbot
**Created**: 2025-12-23
**Status**: Setup Instructions
**Total Time**: ~25 minutes end-to-end

---

## Prerequisites ✅

Before starting, verify the following are in place:

### 1. Feature 009 Completion
- [ ] Feature 009 (retrieval-agent) is complete and tested
- [ ] `backend/agent.py` exists and exports:
  - `retrieve_context(query, context=None)` function
  - `generate_answer(query, context=None, conversation_history=None)` function
  - Proper error handling for missing content
- **Check**: Run `cd backend && python -c "from agent import retrieve_context, generate_answer; print('✅ Agent imports successful')"`

### 2. Data & Infrastructure
- [ ] Qdrant vector database is running and populated with embeddings from feature 007
- [ ] `.env` file in `backend/` contains valid `OPENAI_API_KEY`
- **Check**: `cd backend && python -c "from agent import qdrant_client; print(f'✅ Connected to Qdrant: {qdrant_client.get_collections()}')"`

### 3. System Requirements
- [ ] Python 3.10+ installed: `python --version`
- [ ] Node.js 18+ installed: `node --version` and `npm --version`
- [ ] Git available: `git --version`
- **Check CORS support**: Docusaurus dev server will run on `http://localhost:3000` and backend on `http://localhost:8000`

**Estimated time**: 5 minutes to verify

---

## Backend Setup (5 minutes)

### Step 1: Add FastAPI Dependencies

**File**: `backend/pyproject.toml`

Add to the `[project]` dependencies section:

```toml
[project]
dependencies = [
    "fastapi>=0.104.0",
    "uvicorn[standard]>=0.24.0",
    "python-multipart>=0.0.6",
    # ... existing dependencies
]
```

**Verification**:
```bash
cd backend
cat pyproject.toml | grep -A 5 "\[project\]"
```

### Step 2: Install Backend Dependencies

**Time**: 2-3 minutes

```bash
cd backend

# Install updated dependencies
pip install -e ".[dev]"

# Verify installations
python -c "import fastapi; print(f'✅ FastAPI {fastapi.__version__} installed')"
python -c "import uvicorn; print(f'✅ Uvicorn {uvicorn.__version__} installed')"
```

**Expected output**:
```
✅ FastAPI 0.104.0 installed
✅ Uvicorn 0.24.0 installed
```

### Step 3: Create FastAPI Backend

**File**: `backend/api.py`
**Status**: Implementation placeholder (will be implemented in tasks phase)

Create the file with this structure:

```python
"""
FastAPI backend for Docusaurus RAG Chatbot
Wraps agent.py from feature 009 with REST API endpoints
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import Optional, List
import uuid
from datetime import datetime
import json
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import agent functions from feature 009
from agent import retrieve_context, generate_answer

# ============================================================================
# Data Models (from contracts/api-schema.yaml)
# ============================================================================

class Citation(BaseModel):
    """Source reference in agent response"""
    module_name: str
    chapter_id: str
    source_url: str
    similarity_score: float

class ChatRequest(BaseModel):
    """HTTP request payload from frontend"""
    query: str = Field(..., min_length=1, max_length=500, description="User question")
    conversation_id: Optional[str] = Field(None, description="UUID v4 for multi-turn conversations")
    context: Optional[str] = Field(None, max_length=2000, description="Optional selected text context")

class ChatResponse(BaseModel):
    """HTTP response payload to frontend"""
    answer: str
    citations: List[Citation]
    conversation_id: str
    metadata: dict

class ErrorResponse(BaseModel):
    """Error response format"""
    error: str
    message: str
    details: Optional[List[dict]] = None
    timestamp: str

# ============================================================================
# Backend State Management
# ============================================================================

class ConversationManager:
    """In-memory conversation state manager for development"""
    def __init__(self):
        self.conversations = {}

    def get_or_create(self, conversation_id=None):
        """Get existing conversation or create new one"""
        if not conversation_id:
            conversation_id = str(uuid.uuid4())

        if conversation_id not in self.conversations:
            self.conversations[conversation_id] = {
                "conversation_id": conversation_id,
                "messages": [],
                "created_at": datetime.now().isoformat(),
                "last_updated": datetime.now().isoformat()
            }

        return conversation_id

# Initialize global conversation manager
conv_manager = ConversationManager()

# ============================================================================
# FastAPI Application Setup
# ============================================================================

app = FastAPI(
    title="Docusaurus Chatbot RAG API",
    description="REST API for interactive Q&A about textbook content using RAG",
    version="1.0.0",
    docs_url="/api/docs",
    openapi_url="/api/openapi.json"
)

# Configure CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Accept"],
)

# ============================================================================
# Health Check Endpoint
# ============================================================================

@app.get("/api/health", tags=["Health"])
async def health_check():
    """Check backend service health"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "service": "docusaurus-chatbot-api",
        "version": "1.0.0"
    }

# ============================================================================
# Chat Query Endpoint (Main Integration Point)
# ============================================================================

@app.post("/api/chat/query", response_model=ChatResponse, tags=["Chat"])
async def chat_query(request: ChatRequest):
    """
    Submit a chat query and receive a RAG-based answer

    Flow:
    1. Validate request payload
    2. Get or create conversation
    3. Call agent.py to retrieve context and generate answer
    4. Return response with citations
    """
    try:
        # Validate and get conversation ID
        conversation_id = conv_manager.get_or_create(request.conversation_id)

        logger.info(f"Processing query: {request.query[:50]}... (conv: {conversation_id})")

        # Call agent.py functions
        try:
            # Retrieve context from Qdrant (via agent.py)
            context_data = retrieve_context(
                query=request.query,
                context=request.context,
                top_k=5
            )

            # Generate answer using agent
            answer_data = generate_answer(
                query=request.query,
                context=context_data,
                conversation_history=conv_manager.conversations[conversation_id]["messages"]
            )

            # Format citations from agent response
            citations = [
                Citation(
                    module_name=cite.get("module_name", "Unknown"),
                    chapter_id=cite.get("chapter_id", ""),
                    source_url=cite.get("source_url", ""),
                    similarity_score=cite.get("similarity_score", 0.0)
                )
                for cite in answer_data.get("citations", [])
            ]

            # Update conversation history
            conv_manager.conversations[conversation_id]["messages"].append({
                "role": "user",
                "content": request.query,
                "timestamp": datetime.now().isoformat()
            })
            conv_manager.conversations[conversation_id]["messages"].append({
                "role": "assistant",
                "content": answer_data.get("answer", ""),
                "timestamp": datetime.now().isoformat(),
                "citations": len(citations)
            })

            # Build response
            response = ChatResponse(
                answer=answer_data.get("answer", "Unable to generate answer"),
                citations=citations,
                conversation_id=conversation_id,
                metadata={
                    "retrieved_chunks": len(citations),
                    "total_tokens_used": answer_data.get("tokens_used", 0),
                    "response_time_ms": answer_data.get("response_time_ms", 0),
                    "timestamp": datetime.now().isoformat()
                }
            )

            logger.info(f"✅ Query processed successfully: {len(citations)} citations")
            return response

        except Exception as agent_error:
            logger.error(f"❌ Agent error: {str(agent_error)}")
            raise HTTPException(
                status_code=503,
                detail={
                    "error": "service_unavailable",
                    "message": "Backend processing error",
                    "timestamp": datetime.now().isoformat()
                }
            )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"❌ Unexpected error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "internal_error",
                "message": str(e),
                "timestamp": datetime.now().isoformat()
            }
        )

# ============================================================================
# Conversation Reset Endpoint (Optional)
# ============================================================================

@app.post("/api/chat/reset/{conversation_id}", tags=["Chat"])
async def reset_conversation(conversation_id: str):
    """Clear conversation history (used by "New Chat" button in frontend)"""
    if conversation_id in conv_manager.conversations:
        conv_manager.conversations[conversation_id]["messages"] = []
        conv_manager.conversations[conversation_id]["last_updated"] = datetime.now().isoformat()

    return {
        "status": "reset",
        "conversation_id": conversation_id,
        "timestamp": datetime.now().isoformat()
    }

# ============================================================================
# Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
```

**Verification**: File created at `backend/api.py`
```bash
ls -la backend/api.py && python -c "import api; print('✅ api.py imports successfully')"
```

### Step 4: Run FastAPI Backend

**Time**: 1 minute

```bash
cd backend

# Start the development server with hot reload
uvicorn api:app --reload --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

**Keep this terminal open** - proceed with frontend setup in a new terminal.

### Step 5: Test Backend Health

**In new terminal** (keep backend running):

```bash
# Test health endpoint
curl -X GET http://localhost:8000/api/health

# Expected response:
# {
#   "status": "healthy",
#   "timestamp": "2025-12-23T15:30:45.123456",
#   "service": "docusaurus-chatbot-api",
#   "version": "1.0.0"
# }
```

✅ **Backend setup complete!**

---

## Frontend Setup (10 minutes)

### Step 1: Create ChatbotWidget Directory

**Time**: 1 minute

```bash
# Navigate to project root
cd /path/to/books

# Create directory structure
mkdir -p src/theme/ChatbotWidget

# Verify structure
ls -la src/theme/ChatbotWidget/
# Expected: Empty directory ready for component files
```

### Step 2: Create React Component Files

**File**: `src/theme/ChatbotWidget/types.ts`

```typescript
/**
 * TypeScript interfaces for Chatbot Widget
 * Defines component state, props, and data types
 */

export interface Citation {
  module_name: string;
  chapter_id: string;
  source_url: string;
  similarity_score: number;
}

export interface ChatMessage {
  role: "user" | "assistant";
  content: string;
  timestamp: string;
  citations?: Citation[];
}

export interface ChatResponse {
  answer: string;
  citations: Citation[];
  conversation_id: string;
  metadata: {
    retrieved_chunks: number;
    total_tokens_used: number;
    response_time_ms: number;
    timestamp: string;
  };
}

export interface ChatWidgetState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  conversationId: string | null;
  input: string;
}

export interface ChatbotWidgetProps {
  apiBaseUrl?: string;
  position?: "bottom-right" | "bottom-left";
  theme?: "light" | "dark";
}
```

**File**: `src/theme/ChatbotWidget/styles.module.css`

```css
/* Chatbot Widget Styles */

.chatbotWidget {
  position: fixed;
  bottom: 20px;
  right: 20px;
  z-index: 1000;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
}

.chatbotToggle {
  width: 56px;
  height: 56px;
  border-radius: 50%;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
  transition: all 0.3s ease;
  color: white;
  font-size: 24px;
}

.chatbotToggle:hover {
  transform: scale(1.1);
  box-shadow: 0 6px 16px rgba(102, 126, 234, 0.6);
}

.chatbotContainer {
  position: fixed;
  bottom: 90px;
  right: 20px;
  width: 400px;
  max-height: 600px;
  background: white;
  border-radius: 12px;
  box-shadow: 0 5px 40px rgba(0, 0, 0, 0.16);
  display: flex;
  flex-direction: column;
  animation: slideUp 0.3s ease;
}

@keyframes slideUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.chatbotHeader {
  padding: 16px;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-radius: 12px 12px 0 0;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.chatbotTitle {
  font-weight: 600;
  font-size: 16px;
  margin: 0;
}

.closeButton {
  background: none;
  border: none;
  color: white;
  cursor: pointer;
  font-size: 20px;
  padding: 0;
}

.messagesContainer {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.message {
  display: flex;
  margin-bottom: 8px;
}

.userMessage {
  justify-content: flex-end;
}

.botMessage {
  justify-content: flex-start;
}

.messageBubble {
  max-width: 75%;
  padding: 12px 16px;
  border-radius: 12px;
  line-height: 1.5;
  font-size: 14px;
}

.userBubble {
  background: #667eea;
  color: white;
  border-bottom-right-radius: 2px;
}

.botBubble {
  background: #f0f0f0;
  color: #333;
  border-bottom-left-radius: 2px;
}

.citations {
  margin-top: 8px;
  padding-top: 8px;
  border-top: 1px solid #e0e0e0;
  font-size: 12px;
}

.citation {
  margin: 4px 0;
  color: #666;
}

.citationLink {
  color: #667eea;
  text-decoration: none;
  cursor: pointer;
}

.citationLink:hover {
  text-decoration: underline;
}

.inputContainer {
  padding: 12px;
  border-top: 1px solid #e0e0e0;
  display: flex;
  gap: 8px;
}

.inputField {
  flex: 1;
  border: 1px solid #ddd;
  border-radius: 20px;
  padding: 8px 16px;
  font-size: 14px;
  outline: none;
  font-family: inherit;
}

.inputField:focus {
  border-color: #667eea;
  box-shadow: 0 0 0 3px rgba(102, 126, 234, 0.1);
}

.sendButton {
  width: 36px;
  height: 36px;
  border-radius: 50%;
  background: #667eea;
  color: white;
  border: none;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background 0.2s;
}

.sendButton:hover {
  background: #764ba2;
}

.sendButton:disabled {
  background: #ccc;
  cursor: not-allowed;
}

.loadingSpinner {
  display: inline-block;
  width: 16px;
  height: 16px;
  border: 2px solid #f0f0f0;
  border-top: 2px solid #667eea;
  border-radius: 50%;
  animation: spin 0.6s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.errorMessage {
  background: #fee;
  color: #c00;
  padding: 12px;
  border-radius: 8px;
  border-left: 4px solid #c00;
  margin: 8px 0;
}

/* Mobile responsive */
@media (max-width: 768px) {
  .chatbotContainer {
    width: 90vw;
    max-width: 400px;
    max-height: 70vh;
  }

  .messageBubble {
    max-width: 90%;
  }
}

/* Dark mode support */
@media (prefers-color-scheme: dark) {
  .chatbotContainer {
    background: #2d2d2d;
    color: #fff;
  }

  .messagesContainer {
    background: #1a1a1a;
  }

  .botBubble {
    background: #333;
    color: #fff;
  }

  .inputField {
    background: #333;
    color: #fff;
    border-color: #444;
  }

  .inputField:focus {
    border-color: #667eea;
  }
}
```

**File**: `src/theme/ChatbotWidget/index.tsx`

```typescript
/**
 * Chatbot Widget Component
 * Embedded React component that provides interactive Q&A interface
 * Integrates with FastAPI backend for RAG-based responses
 */

import React, { useState, useRef, useEffect } from "react";
import axios from "axios";
import styles from "./styles.module.css";
import { ChatMessage, ChatWidgetState, ChatbotWidgetProps, Citation } from "./types";

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || "http://localhost:8000";

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  apiBaseUrl = API_BASE_URL,
  position = "bottom-right",
  theme = "light",
}) => {
  const [state, setState] = useState<ChatWidgetState>({
    messages: [],
    isOpen: false,
    isLoading: false,
    error: null,
    conversationId: null,
    input: "",
  });

  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [state.messages]);

  // Initialize new conversation
  const initializeConversation = () => {
    if (!state.conversationId) {
      // Backend will generate new ID on first request
      setState((prev) => ({ ...prev, conversationId: undefined as any }));
    }
  };

  // Handle sending message
  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!state.input.trim() || state.isLoading) return;

    const userMessage = state.input.trim();

    // Add user message to UI immediately
    setState((prev) => ({
      ...prev,
      messages: [
        ...prev.messages,
        {
          role: "user",
          content: userMessage,
          timestamp: new Date().toISOString(),
        },
      ],
      input: "",
      isLoading: true,
      error: null,
    }));

    try {
      // Send request to backend
      const response = await axios.post(
        `${apiBaseUrl}/api/chat/query`,
        {
          query: userMessage,
          conversation_id: state.conversationId,
          context: null,
        },
        {
          timeout: 30000, // 30 second timeout
        }
      );

      const { answer, citations, conversation_id, metadata } = response.data;

      // Add bot response with citations
      setState((prev) => ({
        ...prev,
        messages: [
          ...prev.messages,
          {
            role: "assistant",
            content: answer,
            timestamp: new Date().toISOString(),
            citations: citations || [],
          },
        ],
        isLoading: false,
        conversationId: conversation_id,
      }));
    } catch (error) {
      let errorMessage = "Unable to connect to Q&A service. Please try again later.";

      if (axios.isAxiosError(error)) {
        if (error.code === "ECONNABORTED") {
          errorMessage = "Request timeout. Backend may be unavailable.";
        } else if (error.response?.status === 400) {
          errorMessage = "Invalid request. Please try again with a shorter question.";
        } else if (error.response?.status === 429) {
          errorMessage = "Too many requests. Please wait before asking again.";
        } else if (error.response?.status === 503) {
          errorMessage = "Service temporarily unavailable. Please try again.";
        }
      }

      setState((prev) => ({
        ...prev,
        isLoading: false,
        error: errorMessage,
      }));
    }
  };

  // Toggle widget visibility
  const toggleWidget = () => {
    setState((prev) => {
      if (!prev.isOpen) {
        initializeConversation();
      }
      return { ...prev, isOpen: !prev.isOpen };
    });
  };

  // Reset conversation
  const handleReset = async () => {
    if (state.conversationId) {
      try {
        await axios.post(`${apiBaseUrl}/api/chat/reset/${state.conversationId}`);
      } catch (error) {
        console.error("Failed to reset conversation:", error);
      }
    }

    setState({
      messages: [],
      isOpen: true,
      isLoading: false,
      error: null,
      conversationId: null,
      input: "",
    });
  };

  return (
    <div className={styles.chatbotWidget}>
      {/* Chat Toggle Button */}
      <button className={styles.chatbotToggle} onClick={toggleWidget} title="Open chatbot">
        {state.isOpen ? "✕" : "💬"}
      </button>

      {/* Chat Container */}
      {state.isOpen && (
        <div className={styles.chatbotContainer}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <h3 className={styles.chatbotTitle}>Textbook Q&A</h3>
            <button
              className={styles.closeButton}
              onClick={toggleWidget}
              title="Close chatbot"
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {state.messages.length === 0 ? (
              <div style={{ color: "#999", fontSize: "14px", textAlign: "center" }}>
                👋 Hi! Ask me anything about the textbook. Try "What is ROS 2?" or "Explain
                Isaac Sim"
              </div>
            ) : (
              state.messages.map((msg, idx) => (
                <div
                  key={idx}
                  className={`${styles.message} ${
                    msg.role === "user" ? styles.userMessage : styles.botMessage
                  }`}
                >
                  <div
                    className={`${styles.messageBubble} ${
                      msg.role === "user" ? styles.userBubble : styles.botBubble
                    }`}
                  >
                    {msg.content}
                    {msg.citations && msg.citations.length > 0 && (
                      <div className={styles.citations}>
                        <strong>📚 Sources:</strong>
                        {msg.citations.map((cite, cidx) => (
                          <div key={cidx} className={styles.citation}>
                            {cite.module_name} •{" "}
                            <a
                              href={`/${cite.source_url}`}
                              className={styles.citationLink}
                              target="_blank"
                              rel="noopener noreferrer"
                            >
                              {cite.chapter_id}
                            </a>{" "}
                            ({Math.round(cite.similarity_score * 100)}%)
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                </div>
              ))
            )}

            {state.isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageBubble} style={{ background: "#f0f0f0" }}>
                  <div className={styles.loadingSpinner}></div>
                </div>
              </div>
            )}

            {state.error && <div className={styles.errorMessage}>{state.error}</div>}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <form onSubmit={handleSendMessage} className={styles.inputContainer}>
            <input
              type="text"
              className={styles.inputField}
              placeholder="Ask about the book..."
              value={state.input}
              onChange={(e) => setState((prev) => ({ ...prev, input: e.target.value }))}
              disabled={state.isLoading}
              maxLength={500}
            />
            <button
              type="submit"
              className={styles.sendButton}
              disabled={state.isLoading || !state.input.trim()}
              title="Send message"
            >
              {state.isLoading ? <div className={styles.loadingSpinner} /> : "→"}
            </button>
          </form>

          {/* Reset button (optional) */}
          {state.messages.length > 0 && (
            <div style={{ padding: "8px 12px", textAlign: "center" }}>
              <button
                onClick={handleReset}
                style={{
                  background: "none",
                  border: "none",
                  color: "#667eea",
                  cursor: "pointer",
                  fontSize: "12px",
                  textDecoration: "underline",
                }}
              >
                Start New Conversation
              </button>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;
```

**Verification**: All three files created
```bash
ls -la src/theme/ChatbotWidget/
# Expected: index.tsx, styles.module.css, types.ts
```

### Step 2: Install Frontend Dependencies

**Time**: 2-3 minutes

```bash
cd /path/to/books

# Install axios (HTTP client for API calls)
npm install axios

# Verify installation
npm ls axios
# Expected: axios@^1.x.x
```

### Step 3: Configure Docusaurus Theme Integration

**File**: `docusaurus.config.js` (or `.config.ts`)
Add ChatbotWidget to the theme configuration:

```javascript
// In your docusaurus.config.js, add to swizzling configuration:

module.exports = {
  // ... existing config

  swizzleConfig: {
    components: [
      // ... existing swizzle config
    ],
  },

  // Custom theme component configuration
  customTheme: {
    components: {
      ChatbotWidget: {
        apiBaseUrl: process.env.API_BASE_URL || "http://localhost:8000",
        position: "bottom-right",
        theme: "light",
      },
    },
  },
};
```

**Alternative** (simpler): Import directly in `src/pages/_app.tsx` or layout wrapper:

```typescript
import ChatbotWidget from "@site/src/theme/ChatbotWidget";

export default function RootLayout({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget apiBaseUrl="http://localhost:8000" />
    </>
  );
}
```

### Step 4: Start Docusaurus Dev Server

**Time**: 1-2 minutes

```bash
cd /path/to/books

# Start Docusaurus with hot reload
npm start
```

**Expected output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000
```

**Browser**: Open http://localhost:3000

✅ **Frontend setup complete!**

---

## Quick Start Test (5 minutes)

### Verify Both Services Running

**Terminal 1 - Backend** (should still be running):
```bash
cd backend
# Ctrl+C to stop if needed, then restart
uvicorn api:app --reload --port 8000
# Output: Uvicorn running on http://0.0.0.0:8000
```

**Terminal 2 - Frontend**:
```bash
cd /path/to/books
npm start
# Output: Docusaurus website is running at: http://localhost:3000
```

### Test the Chatbot

**Step 1**: Open http://localhost:3000 in browser

**Step 2**: Look for chatbot button
- 💬 button in **bottom-right corner**
- Styled with purple gradient
- Should be visible on all pages

**Step 3**: Click to open chatbot widget
- Widget slides up from bottom-right
- Shows greeting message
- Input field ready for questions

**Step 4**: Test basic query
```
Input: "What is ROS 2?"
Expected output:
- Message appears on right (user bubble)
- Loading spinner appears on left
- Bot response appears within 5-10 seconds
- Response includes answer text + 📚 Sources section
- Citations show module name, chapter link, similarity percentage
```

**Step 5**: Test error handling (stop backend momentarily)
```
1. Terminal 1: Press Ctrl+C to stop uvicorn
2. Ask another question in chatbot
3. Expected: Error message "Unable to connect to Q&A service"
4. Terminal 1: Restart: uvicorn api:app --reload --port 8000
5. Ask again - should work
```

**Step 6**: Test multi-turn conversation
```
Q1: "What is Navigation Stack?"
A1: [Answer with citations]

Q2: "How do I configure it?"
Expected: Backend understands "it" refers to Navigation Stack
Backend maintains conversation context
Answer should be about Navigation Stack configuration (not a new topic)
```

✅ **All systems operational!**

---

## Configuration

### Environment Variables

**File**: `backend/.env`

```env
# API Configuration
API_PORT=8000
API_HOST=0.0.0.0

# OpenAI Configuration (from feature 009)
OPENAI_API_KEY=sk-...  # Your OpenAI API key

# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION_NAME=textbook_embeddings

# CORS Configuration
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Logging
LOG_LEVEL=INFO
```

### CORS Configuration

The FastAPI backend includes CORS middleware configured for local development:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=False,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Accept"],
)
```

**For production deployment**, modify in `backend/api.py`:
```python
# Change from hardcoded list to environment variable
allowed_origins = os.getenv("ALLOWED_ORIGINS", "").split(",")
app.add_middleware(CORSMiddleware, allow_origins=allowed_origins, ...)
```

### API Configuration

**Frontend** (`src/theme/ChatbotWidget/index.tsx`):
```typescript
const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || "http://localhost:8000";
```

**Set via environment**:
```bash
export REACT_APP_API_BASE_URL=http://localhost:8000
npm start
```

---

## Troubleshooting

### CORS Error in Browser Console
```
Error: Access to XMLHttpRequest at 'http://localhost:8000/api/chat/query'
from origin 'http://localhost:3000' has been blocked by CORS policy
```

**Solution**:
1. Verify FastAPI CORS middleware is in `backend/api.py`
2. Check allowed_origins includes `http://localhost:3000`
3. Backend must be running on port 8000
4. Clear browser cache: DevTools → Application → Clear Storage

**Command to test CORS**:
```bash
curl -X OPTIONS http://localhost:8000/api/chat/query \
  -H "Origin: http://localhost:3000" \
  -H "Access-Control-Request-Method: POST" -v
```

### Backend Not Running Error
```
Unable to connect to Q&A service. Please try again later.
```

**Solution**:
1. Check backend is running:
   ```bash
   curl http://localhost:8000/api/health
   ```
2. If not running, start it:
   ```bash
   cd backend && uvicorn api:app --reload --port 8000
   ```
3. Verify port 8000 is not blocked:
   ```bash
   lsof -i :8000  # macOS/Linux
   netstat -ano | findstr :8000  # Windows
   ```

### No Response from Chatbot
```
Question submitted, but chatbot doesn't respond (no loading spinner)
```

**Solution**:
1. Check browser console (F12 → Console tab) for JavaScript errors
2. Verify agent.py functions exist:
   ```bash
   cd backend
   python -c "from agent import retrieve_context, generate_answer; print('✅')"
   ```
3. Check Qdrant is running and has data:
   ```bash
   python -c "from agent import qdrant_client; print(qdrant_client.get_collections())"
   ```
4. Check backend logs for errors:
   ```
   Terminal 1 (backend): Look for exception messages
   ```

### Query Timeout
```
Browser request hangs for >30 seconds, then "Request timeout" error
```

**Solution**:
1. Increase timeout in `src/theme/ChatbotWidget/index.tsx`:
   ```typescript
   timeout: 60000  // 60 seconds instead of 30
   ```
2. Check backend performance:
   ```bash
   # Add timing to backend logs
   # Monitor uvicorn output for slow processing
   ```
3. Check network latency:
   ```bash
   # Time a simple health check
   curl -w "@curl-format.txt" -o /dev/null -s http://localhost:8000/api/health
   ```

### Conversation Context Lost
```
Multi-turn question doesn't understand previous context
```

**Verification**:
1. Backend should be maintaining `conversations` dict
2. Check `conversation_id` is being passed in requests (F12 → Network tab)
3. Verify agent.py accepts `conversation_history` parameter

### Mobile Responsiveness Issues
```
Chatbot widget not visible or broken on mobile
```

**Solution**: Styles already include mobile breakpoints in `styles.module.css`:
```css
@media (max-width: 768px) {
  .chatbotContainer {
    width: 90vw;
    max-width: 400px;
  }
}
```

**Test on mobile**:
```bash
# Use DevTools device emulation (F12 → Device Toolbar)
# Or test on actual device: http://<your-ip>:3000
```

---

## Testing Scenarios

### Test Case 1: Basic Query (User Story 1)
**Scenario**: Reader asks basic question about textbook content

```
Setup:
- Chatbot widget visible
- Backend running on :8000
- Qdrant populated with embeddings

Steps:
1. Click 💬 button to open chatbot
2. Type: "What is ROS 2?"
3. Click send (or press Enter)

Expected Results:
✅ Message appears on right (user bubble)
✅ Loading spinner appears briefly
✅ Response appears within 10 seconds
✅ Answer includes: description of ROS 2
✅ 📚 Sources section shows 1-3 citations
✅ Each citation shows: Module name, Chapter link, % similarity
✅ No error messages or console errors

Acceptance Criteria:
- Question successfully sent to backend
- Backend calls agent.py and retrieves context
- Response grounded in textbook (not generic)
- Citations match retrieved chunks
```

### Test Case 2: Multi-Turn Conversation (User Story 2)
**Scenario**: Reader has contextual follow-up conversation

```
Setup:
- Same as Test Case 1, with backend running

Steps:
1. Ask first question: "What is Navigation Stack in ROS 2?"
2. Wait for response with citations
3. Ask follow-up: "How do I configure it?"
4. Verify context is maintained

Expected Results:
✅ First question answered with citations
✅ Second question understood to refer to "Navigation Stack"
✅ Answer focuses on "Navigation Stack configuration" not generic config
✅ Same conversation_id appears in both responses
✅ Backend maintains message history

Acceptance Criteria:
- conversation_id returned from first query is reused in second
- Backend message history shows both messages
- Second answer references first question context
- No timeout errors, <10s response time
```

### Test Case 3: Selected Text Context (User Story 3)
**Scenario**: Reader highlights text and asks context-specific question

```
Setup:
- Docusaurus page with text about specific topic (e.g., VSLAM)
- Chatbot widget open

Steps:
1. Highlight a paragraph about VSLAM on a page
2. Right-click → "Ask Chatbot about this" (if implemented)
   OR manually copy text and add as context
3. Ask: "Explain this in simpler terms"

Expected Results:
✅ Selected text is passed to backend as `context` parameter
✅ Backend passes context to agent.py
✅ Answer focuses on the selected text
✅ Response is more targeted than generic explanation
✅ Citations relevant to selected topic

Acceptance Criteria:
- Context parameter received by backend
- Agent uses context in retrieval (check metadata)
- Answer quality improved vs. no-context query
```

### Test Case 4: Error Handling
**Scenario**: Various error conditions

```
Test 4a - Backend Unavailable:
1. Stop backend: Ctrl+C in backend terminal
2. Ask question in chatbot: "What is Isaac Sim?"
3. Expected: ❌ "Unable to connect to Q&A service" error within 5s
4. Restart backend: uvicorn api:app --reload --port 8000
5. Ask same question again
6. Expected: ✅ Works again (recovery verified)

Test 4b - Very Long Question:
1. Paste 600+ word text into chatbot
2. Expected: ❌ "Please ask a more concise question" or auto-truncate
3. Verify max_length=500 validation in backend

Test 4c - Network Timeout:
1. Add artificial delay to agent.py (for testing)
2. Ask question with <30s timeout configured
3. Expected: ❌ "Request timeout" error message
4. Remove delay
5. Expected: ✅ Response within normal time

Test 4d - Empty Question:
1. Click send with empty input
2. Expected: ❌ Send button disabled (no request sent)
3. Type space, click send
4. Expected: ❌ Validation error or ignored
```

### Test Case 5: Mobile Responsiveness
**Scenario**: Test on different screen sizes

```
Setup:
- F12 DevTools → Device Toolbar (or real mobile device)

Steps:
1. Set to iPhone 12 size (390x844)
2. Open http://localhost:3000
3. Verify chatbot button visible and clickable
4. Click to open widget
5. Verify widget doesn't exceed screen width
6. Type and send message
7. Verify message bubbles wrap properly
8. Test landscape mode (844x390)

Expected Results:
✅ Widget fits on small screens
✅ Input field remains accessible
✅ Messages remain readable
✅ Citations formatted properly on mobile
✅ No horizontal scrolling
✅ No overlapping elements
```

---

## Success Checklist

**Complete all items before marking feature as done:**

- [ ] **Prerequisites Met**
  - [ ] Feature 009 (retrieval-agent) complete and tested
  - [ ] Qdrant running and populated
  - [ ] Python 3.10+, Node.js 18+ installed
  - [ ] OPENAI_API_KEY in backend/.env

- [ ] **Backend Setup**
  - [ ] FastAPI and uvicorn added to backend/pyproject.toml
  - [ ] backend/api.py created with all endpoints
  - [ ] Health endpoint responds: curl http://localhost:8000/api/health
  - [ ] CORS configured for localhost:3000
  - [ ] Backend runs without errors: uvicorn api:app --reload --port 8000

- [ ] **Frontend Setup**
  - [ ] src/theme/ChatbotWidget/ directory created
  - [ ] types.ts, styles.module.css, index.tsx files created
  - [ ] axios installed: npm install axios
  - [ ] Docusaurus dev server runs: npm start
  - [ ] Chatbot widget visible on http://localhost:3000

- [ ] **Integration Testing**
  - [ ] Chatbot widget renders on all Docusaurus pages
  - [ ] Basic query works: "What is ROS 2?" receives answer with citations
  - [ ] Backend health check passes
  - [ ] Frontend sends requests to http://localhost:8000/api/chat/query
  - [ ] Responses include answer, citations, conversation_id

- [ ] **Feature Requirements (from spec.md)**
  - [ ] FR-001: FastAPI backend with REST API ✅
  - [ ] FR-002: /api/chat/query endpoint ✅
  - [ ] FR-003: JSON response format ✅
  - [ ] FR-004: Integration with agent.py ✅
  - [ ] FR-005: Qdrant retrieval ✅
  - [ ] FR-006: Embedded chatbot widget ✅
  - [ ] FR-007: Persistent across navigation ✅
  - [ ] FR-008: Open/close toggle ✅
  - [ ] FR-009: HTTP POST requests ✅
  - [ ] FR-010: Conversational UI ✅
  - [ ] FR-011: Loading indicators ✅
  - [ ] FR-012: Error handling ✅
  - [ ] FR-013: Selected text context ✅
  - [ ] FR-014: Conversation state ✅
  - [ ] FR-015: Interaction logging ✅

- [ ] **Success Criteria (from spec.md)**
  - [ ] SC-001: <10s response time (p95) ✅
  - [ ] SC-002: Renders on all browsers/sizes ✅
  - [ ] SC-003: 95%+ queries reach backend ✅
  - [ ] SC-004: Multi-turn conversation works ✅
  - [ ] SC-005: Error messages user-friendly ✅
  - [ ] SC-006: Context (2000 chars) supported ✅
  - [ ] SC-007: UI feedback for all states ✅
  - [ ] SC-008: Setup completed in <30 minutes ✅

- [ ] **No Errors**
  - [ ] Browser console: no JavaScript errors
  - [ ] Backend logs: no exception stack traces
  - [ ] No CORS warnings
  - [ ] No 500 errors from API

- [ ] **Documentation**
  - [ ] This quickstart.md completed
  - [ ] Relevant files documented with code references
  - [ ] Configuration environment variables set
  - [ ] Troubleshooting section covers common issues

---

## Related Files

**Backend Implementation**:
- 🔗 [backend/api.py](../../backend/api.py) - FastAPI application with endpoints
- 🔗 [backend/agent.py](../../backend/agent.py) - Feature 009 integration (existing)
- 🔗 [backend/pyproject.toml](../../backend/pyproject.toml) - Dependencies
- 🔗 [backend/.env](../../backend/.env) - Environment configuration

**Frontend Implementation**:
- 🔗 [src/theme/ChatbotWidget/index.tsx](../../src/theme/ChatbotWidget/index.tsx) - React component
- 🔗 [src/theme/ChatbotWidget/styles.module.css](../../src/theme/ChatbotWidget/styles.module.css) - Styling
- 🔗 [src/theme/ChatbotWidget/types.ts](../../src/theme/ChatbotWidget/types.ts) - TypeScript types
- 🔗 [docusaurus.config.js](../../docusaurus.config.js) - Docusaurus configuration

**Specification & Design**:
- 🔗 [specs/010-docusaurus-chatbot/spec.md](spec.md) - Feature specification
- 🔗 [specs/010-docusaurus-chatbot/plan.md](plan.md) - Implementation plan
- 🔗 [specs/010-docusaurus-chatbot/data-model.md](data-model.md) - Data model
- 🔗 [specs/010-docusaurus-chatbot/contracts/api-schema.yaml](contracts/api-schema.yaml) - OpenAPI schema

**Prerequisites**:
- 🔗 [specs/009-retrieval-agent/spec.md](../009-retrieval-agent/spec.md) - Feature 009 (required)
- 🔗 [specs/007-embedding-ingestion/spec.md](../007-embedding-ingestion/spec.md) - Feature 007 (Qdrant data)

---

## Next Steps

### After Quickstart Completion

**Phase 1 - Verification** (5-10 minutes):
1. ✅ Complete all success checklist items
2. ✅ Run through all 5 test cases
3. ✅ Document any issues or customizations

**Phase 2 - Implementation** (via `/sp.tasks`):
1. Generate task breakdown: Run `/sp.tasks`
2. Implement full backend/api.py with error handling
3. Enhance frontend with additional features
4. Add comprehensive logging
5. Create unit tests for backend endpoints

**Phase 3 - Optimization** (optional):
1. Measure and optimize response time
2. Implement request queuing for concurrent users
3. Add conversation persistence (beyond scope for MVP)
4. Deploy to staging environment

**Phase 4 - Documentation**:
1. Create API documentation from OpenAPI schema
2. Write troubleshooting guide for deployment
3. Record demo video of feature
4. Update project README with chatbot section

---

## Support & Debugging

### Logging Backend Requests

**In `backend/api.py`**, logs are created at INFO level:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Processing query: What is ROS 2?... (conv: abc123)
INFO:     ✅ Query processed successfully: 2 citations
```

**Enable DEBUG logging**:
```bash
# Start with debug mode
uvicorn api:app --reload --port 8000 --log-level debug
```

### Viewing Network Requests

**Browser DevTools** (F12):
1. Open DevTools → Network tab
2. Ask question in chatbot
3. Click on `query` POST request
4. View:
   - Request payload (JSON body)
   - Response payload (answer + citations)
   - Response headers (timing, CORS headers)
   - Status code (200 for success, 4xx/5xx for errors)

### Checking Frontend State

**Browser DevTools** (F12):
1. Open DevTools → Console tab
2. Inspect React component state:
   ```javascript
   // Requires React DevTools extension
   // Or add debug logging in index.tsx
   ```
3. Monitor errors and warnings

---

**Last Updated**: 2025-12-23
**Feature**: 010-docusaurus-chatbot
**Status**: Ready for Implementation & Testing
