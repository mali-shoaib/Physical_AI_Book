"""
FastAPI Backend for RAG Chatbot

This module provides REST API endpoints for the Docusaurus chatbot frontend.
It wraps the existing agent.py from feature 009 and exposes chat functionality
via HTTP.

Features:
- POST /api/chat/query - Main chat endpoint
- GET /api/health - Health check endpoint
- POST /api/chat/reset - Reset conversation history

Usage:
    uvicorn backend.api:app --reload --port 8000
"""

import os
import sys
from datetime import datetime
from typing import List, Dict, Optional
import uuid
from dotenv import load_dotenv

# Third-party imports
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, validator
from loguru import logger
import uvicorn

# ============================================================================
# Environment Configuration
# ============================================================================

# Load environment variables
load_dotenv()

# Configure logger
logger.remove()
logger.add(
    sys.stderr,
    format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {message}",
    level="INFO"
)

logger.info("FastAPI RAG Chatbot Backend initializing...")

# Import agent.py functions from feature 009
sys.path.append(os.path.dirname(__file__))
from agent import retrieve_context, ask_question, initialize_agent

# ============================================================================
# Pydantic Models
# ============================================================================

class Citation(BaseModel):
    """Source citation for grounded answers"""
    module_name: str = Field(..., description="Module name from textbook")
    chapter_id: str = Field(..., description="Chapter ID (kebab-case)")
    source_url: str = Field(..., description="HTTPS URL to source page")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score 0.0-1.0")


class ChatRequest(BaseModel):
    """Request payload for chat queries"""
    query: str = Field(..., min_length=1, max_length=500, description="User question")
    conversation_id: Optional[str] = Field(None, description="Optional conversation UUID")
    context: Optional[str] = Field(None, max_length=2000, description="Optional selected text context")

    @validator('query')
    def query_must_not_be_empty(cls, v):
        if not v.strip():
            raise ValueError('Query cannot be empty or whitespace')
        return v.strip()


class ResponseMetadata(BaseModel):
    """Metadata about the response"""
    retrieval_count: int = Field(..., description="Number of chunks retrieved")
    tokens_used: int = Field(default=0, description="Total tokens consumed")
    grounded: bool = Field(..., description="Whether answer is based on retrieved context")


class ChatResponse(BaseModel):
    """Response payload with answer and citations"""
    answer: str = Field(..., description="Agent's response text")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    conversation_id: str = Field(..., description="Conversation UUID")
    metadata: ResponseMetadata = Field(..., description="Response metadata")


class ErrorResponse(BaseModel):
    """Error response format"""
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[str] = Field(None, description="Additional error details")


# ============================================================================
# Conversation State Management
# ============================================================================

# In-memory conversation storage (session_id -> conversation_history)
conversations: Dict[str, Dict] = {}

# Configuration for conversation history limits
MAX_HISTORY_MESSAGES = 50  # Maximum number of messages to keep in history


def get_conversation(conversation_id: str) -> Optional[Dict]:
    """Retrieve conversation state by ID"""
    return conversations.get(conversation_id)


def create_conversation(conversation_id: str = None) -> str:
    """Create new conversation and return ID"""
    if conversation_id is None:
        conversation_id = str(uuid.uuid4())

    conversations[conversation_id] = {
        "session_id": conversation_id,
        "conversation_history": [],
        "created_at": datetime.now(),
        "last_updated": datetime.now()
    }

    logger.info(f"Created conversation {conversation_id}")
    return conversation_id


def add_message_to_conversation(conversation_id: str, role: str, content: str) -> None:
    """
    Add message to conversation history with sliding window.

    Implements sliding window to limit history size to MAX_HISTORY_MESSAGES.
    When limit is exceeded, oldest messages are removed to maintain context
    while preventing token overflow.
    """
    if conversation_id in conversations:
        conversations[conversation_id]["conversation_history"].append({
            "role": role,
            "content": content,
            "timestamp": datetime.now()
        })
        conversations[conversation_id]["last_updated"] = datetime.now()

        # Implement sliding window: keep only the last MAX_HISTORY_MESSAGES messages
        history = conversations[conversation_id]["conversation_history"]
        if len(history) > MAX_HISTORY_MESSAGES:
            # Remove oldest messages to stay within limit
            removed_count = len(history) - MAX_HISTORY_MESSAGES
            conversations[conversation_id]["conversation_history"] = history[-MAX_HISTORY_MESSAGES:]
            logger.debug(f"Sliding window applied: removed {removed_count} oldest messages from conversation {conversation_id}")


# ============================================================================
# FastAPI Application
# ============================================================================

app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="REST API for Physical AI Textbook RAG Chatbot"
)

# Configure CORS middleware
# Allow production and local development origins
allowed_origins = [
    # Production
    "https://physical-ai-robotics-textbook-xi.vercel.app",
    # Local development
    "http://localhost:3000",
    "http://127.0.0.1:3000",
    "http://localhost:3001",
    "http://127.0.0.1:3001",
    "http://localhost:3002",
    "http://127.0.0.1:3002",
    "http://localhost:3005",
    "http://127.0.0.1:3005",
    "http://localhost:3006",
    "http://127.0.0.1:3006",
    "http://localhost:3007",
    "http://127.0.0.1:3007"
]

# Add environment-specific origins
if os.getenv("ALLOWED_ORIGINS"):
    additional_origins = os.getenv("ALLOWED_ORIGINS").split(",")
    allowed_origins.extend(additional_origins)
    logger.info(f"Added {len(additional_origins)} additional CORS origins from environment")

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"]
)

logger.info("FastAPI application initialized with CORS enabled")


# ============================================================================
# Health Check Endpoint
# ============================================================================

@app.get("/api/health")
async def health_check():
    """Health check endpoint to verify API is running"""
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "version": "1.0.0"
    }


# ============================================================================
# Chat Query Endpoint
# ============================================================================

@app.post("/api/chat/query", response_model=ChatResponse)
async def chat_query(request: ChatRequest):
    """
    Main chat endpoint - accepts user queries and returns grounded answers with citations.

    This endpoint wraps the agent.py ask_question() function from feature 009.
    """
    try:
        # Log incoming request
        logger.info(f"Chat query received: '{request.query[:50]}...' (conversation_id={request.conversation_id})")

        # Get or create conversation
        conversation_id = request.conversation_id
        if conversation_id is None or conversation_id not in conversations:
            conversation_id = create_conversation(conversation_id)
        else:
            logger.debug(f"Using existing conversation {conversation_id}")

        # Prepare query (include context if provided)
        query = request.query
        if request.context:
            query = f"Context from textbook:\n{request.context}\n\nQuestion: {request.query}"
            logger.debug(f"Added context ({len(request.context)} chars)")

        # Get conversation history
        conversation_data = conversations[conversation_id]
        conversation_history = conversation_data.get("conversation_history", [])

        # Call agent.py ask_question() function
        try:
            agent_response = ask_question(
                question=query,
                session_id=conversation_id,
                conversation_history=conversation_history.copy() if conversation_history else None
            )
        except Exception as agent_error:
            logger.error(f"Agent error: {agent_error}")
            raise HTTPException(
                status_code=503,
                detail=f"Agent service unavailable: {str(agent_error)}"
            )

        # Format citations from agent response
        citations = []
        if "citations" in agent_response and agent_response["citations"]:
            for cit in agent_response["citations"]:
                citations.append(Citation(
                    module_name=cit.get("module_name", "Unknown"),
                    chapter_id=cit.get("chapter_id", "unknown"),
                    source_url=cit.get("source_url", ""),
                    similarity_score=cit.get("similarity_score", 0.0)
                ))

        # Update conversation history
        add_message_to_conversation(conversation_id, "user", request.query)
        add_message_to_conversation(conversation_id, "assistant", agent_response["answer"])

        # Create response
        response = ChatResponse(
            answer=agent_response["answer"],
            citations=citations,
            conversation_id=conversation_id,
            metadata=ResponseMetadata(
                retrieval_count=agent_response.get("retrieval_count", 0),
                tokens_used=agent_response.get("tokens_used", 0),
                grounded=agent_response.get("grounded", False)
            )
        )

        logger.info(f"Query processed successfully (grounded={response.metadata.grounded}, citations={len(citations)})")
        return response

    except HTTPException:
        raise
    except ValueError as ve:
        logger.error(f"Validation error: {ve}")
        raise HTTPException(status_code=400, detail=str(ve))
    except Exception as e:
        logger.error(f"Unexpected error in chat_query: {e}")
        raise HTTPException(
            status_code=503,
            detail=f"Service temporarily unavailable: {str(e)}"
        )


# ============================================================================
# Reset Conversation Endpoint
# ============================================================================

@app.post("/api/chat/reset/{conversation_id}")
async def reset_conversation(conversation_id: str):
    """Reset conversation history for a given conversation ID"""
    if conversation_id in conversations:
        conversations[conversation_id]["conversation_history"] = []
        conversations[conversation_id]["last_updated"] = datetime.now()
        logger.info(f"Reset conversation {conversation_id}")
        return {
            "conversation_id": conversation_id,
            "message": "Conversation reset successfully"
        }
    else:
        raise HTTPException(
            status_code=404,
            detail=f"Conversation {conversation_id} not found"
        )


logger.info("FastAPI backend ready - all endpoints registered")
