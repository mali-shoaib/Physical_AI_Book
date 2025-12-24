"""
RAG Agent for Physical AI Textbook

This module implements a retrieval-enabled agent using OpenAI Agents SDK that queries
the Qdrant vector database and generates grounded answers strictly based on textbook content.

Features:
- Single-question Q&A mode
- Interactive conversation mode
- Explicit grounding with citations
- Function calling for retrieval

Usage:
    # Single question
    python agent.py "What is ROS 2?"

    # Interactive mode
    python agent.py --interactive
"""

import os
import sys
import argparse
import uuid
from datetime import datetime
from typing import List, Dict, Optional, Tuple, Any
from dotenv import load_dotenv

# Third-party imports
import openai
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from loguru import logger
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type


# ============================================================================
# Environment Configuration
# ============================================================================

def load_environment() -> None:
    """
    Load environment variables from .env file and validate required variables.

    Raises:
        SystemExit: If required environment variables are missing
    """
    # Load .env file
    load_dotenv()

    # Required environment variables
    required_vars = [
        "OPENAI_API_KEY",
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY"
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        logger.error("Please check your .env file and ensure all required variables are set")
        sys.exit(1)

    logger.debug("Environment variables loaded successfully")


# ============================================================================
# Logger Configuration
# ============================================================================

def configure_logger(log_level: str = "INFO") -> None:
    """
    Configure loguru logger with custom format and log level.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
    """
    # Remove default logger
    logger.remove()

    # Add custom logger with timestamp, level, and message
    logger.add(
        sys.stderr,
        format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {message}",
        level=log_level,
        colorize=True
    )

    logger.debug(f"Logger configured with level: {log_level}")


# ============================================================================
# Global Configuration
# ============================================================================

# Load environment and configure logger on module import
load_environment()
log_level = os.getenv("LOG_LEVEL", "INFO").upper()
configure_logger(log_level)

# Configuration from environment
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4")
AGENT_MAX_HISTORY = int(os.getenv("AGENT_MAX_HISTORY", "20"))
QDRANT_COLLECTION = "rag_embedding"

# Initialize API clients (global singletons)
openai_client = openai.Client(api_key=OPENAI_API_KEY)
cohere_client = cohere.Client(api_key=COHERE_API_KEY)

logger.info("RAG Agent initialized")
logger.debug(f"Configuration: model={OPENAI_MODEL}, max_history={AGENT_MAX_HISTORY}, collection={QDRANT_COLLECTION}")


# ============================================================================
# Phase 3: Agent Configuration
# ============================================================================

# System prompt with explicit grounding instructions
SYSTEM_PROMPT = """You are a helpful assistant for the Physical AI textbook. Your role is to answer questions STRICTLY based on the provided textbook content.

**CRITICAL RULES:**
1. Answer ONLY using information from the retrieved context provided to you
2. If the context does not contain relevant information, respond EXACTLY with: "I don't have information about that in the textbook"
3. NEVER use external knowledge or make assumptions beyond the provided context
4. Include inline citations in your answers using the format: [Source: Module Name - Chapter]
5. If the question asks about multiple concepts, ensure you have context for ALL concepts before answering
6. Be concise but complete - prioritize accuracy over length

**Citation Requirements:**
- Every factual claim must reference its source from the context
- Use the module name and chapter ID provided in the context metadata
- Format: [Source: <module_name> - <chapter_id>]

**Failure Mode:**
- If no context is provided or context similarity is too low, respond with the exact phrase: "I don't have information about that in the textbook"
- Do NOT attempt to answer questions outside the textbook scope

Your primary goal is to help readers understand the Physical AI textbook content through grounded, cited answers."""


# ============================================================================
# Phase 2: Foundational Functions
# ============================================================================

def embed_query(query_text: str) -> List[float]:
    """
    Generate Cohere embedding for query text.

    Uses Cohere embed-english-v3.0 model with input_type="search_query" to generate
    embeddings compatible with the Qdrant collection from feature 007.

    Args:
        query_text: Query string to embed

    Returns:
        1024-dimensional embedding vector

    Raises:
        ValueError: If query_text is empty
        Exception: If Cohere API call fails
    """
    if not query_text or not query_text.strip():
        logger.error("Query text cannot be empty")
        raise ValueError("Query text cannot be empty")

    try:
        logger.debug(f"Generating embedding for query: {query_text[:50]}...")
        response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"  # Critical: different from document ingestion
        )
        embedding = response.embeddings[0]
        logger.debug(f"Generated embedding vector (dimension: {len(embedding)})")
        return embedding
    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        raise



def init_qdrant_client() -> QdrantClient:
    """
    Initialize Qdrant client and verify connection.

    Connects to the Qdrant instance specified in environment variables and verifies
    that the collection exists.

    Returns:
        QdrantClient: Connected Qdrant client instance

    Raises:
        Exception: If connection fails or collection doesn't exist
    """
    try:
        logger.debug(f"Connecting to Qdrant at {QDRANT_URL}")
        client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=30
        )

        # Verify collection exists
        try:
            collection_info = client.get_collection(QDRANT_COLLECTION)
            logger.info(f"Connected to Qdrant collection '{QDRANT_COLLECTION}' ({collection_info.points_count} points)")
        except Exception as e:
            logger.error(f"Collection '{QDRANT_COLLECTION}' not found: {e}")
            raise

        return client

    except Exception as e:
        logger.error(f"Failed to initialize Qdrant client: {e}")
        raise


def retrieve_context(query: str, top_k: int = 5, threshold: float = 0.70) -> List[Dict]:
    """
    Retrieve relevant chunks from Qdrant based on semantic similarity.

    Generates an embedding for the query using Cohere, searches the Qdrant collection,
    and returns chunks above the similarity threshold.

    Args:
        query: Search query text
        top_k: Number of results to retrieve (default: 5)
        threshold: Minimum similarity score 0.0-1.0 (default: 0.70)

    Returns:
        List of dictionaries containing retrieved chunks with metadata:
        - chunk_id: Unique identifier
        - chunk_text: Text content
        - similarity_score: Cosine similarity (0-1)
        - source_url: URL to source page
        - chapter_id: Chapter identifier
        - module_name: Module name
        - heading_hierarchy: List of section headings
        - token_count: Number of tokens
        - chunk_index: Position in source

    Raises:
        ValueError: If query is empty or parameters are invalid
        Exception: If Qdrant search fails
    """
    if not query or not query.strip():
        raise ValueError("Query cannot be empty")

    if not (1 <= top_k <= 10):
        raise ValueError("top_k must be between 1 and 10")

    if not (0.0 <= threshold <= 1.0):
        raise ValueError("threshold must be between 0.0 and 1.0")

    try:
        # Generate query embedding
        logger.debug(f"Retrieving context for query: '{query[:50]}...'")
        query_embedding = embed_query(query)

        # Initialize Qdrant client
        qdrant_client = init_qdrant_client()

        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION,
            query_vector=query_embedding,
            limit=top_k,
            score_threshold=threshold
        )

        # Format results
        results = []
        for hit in search_results:
            result = {
                "chunk_id": str(hit.id),
                "chunk_text": hit.payload.get("chunk_text", ""),
                "similarity_score": hit.score,
                "source_url": hit.payload.get("source_url", ""),
                "chapter_id": hit.payload.get("chapter_id", ""),
                "module_name": hit.payload.get("module_name", ""),
                "heading_hierarchy": hit.payload.get("heading_hierarchy", []),
                "token_count": hit.payload.get("token_count", 0),
                "chunk_index": hit.payload.get("chunk_index", 0)
            }
            results.append(result)

        logger.info(f"Retrieved {len(results)} chunks (similarity >= {threshold})")
        if results:
            logger.debug(f"Top result: {results[0]['module_name']} (score: {results[0]['similarity_score']:.3f})")

        return results

    except Exception as e:
        logger.error(f"Failed to retrieve context: {e}")
        raise


# ============================================================================
# Phase 3: Agent Helper Functions
# ============================================================================

def format_context_for_agent(results: List[Dict]) -> str:
    """
    Format retrieved chunks with citations for OpenAI agent.

    Takes retrieval results from Qdrant and formats them into a structured context
    string that the agent can use to generate grounded answers.

    Args:
        results: List of retrieval results from retrieve_context()

    Returns:
        Formatted context string with chunks and citation metadata

    Example:
        >>> results = retrieve_context("What is ROS 2?", top_k=3)
        >>> context = format_context_for_agent(results)
        >>> print(context)
        Retrieved Context (3 sources):

        [1] Module: Introduction to ROS 2 | Chapter: ros2-basics | Similarity: 0.92
        Source: https://example.com/module-1/ros2-basics
        Content: ROS 2 is the next generation of the Robot Operating System...
        ---
    """
    if not results:
        logger.warning("No results to format - returning empty context message")
        return "No relevant context found in the textbook."

    formatted_parts = [f"Retrieved Context ({len(results)} sources):\n"]

    for idx, result in enumerate(results, 1):
        chunk_text = result.get("chunk_text", "").strip()
        module_name = result.get("module_name", "Unknown Module")
        chapter_id = result.get("chapter_id", "unknown-chapter")
        source_url = result.get("source_url", "")
        similarity_score = result.get("similarity_score", 0.0)

        # Format each chunk with metadata
        chunk_section = f"""
[{idx}] Module: {module_name} | Chapter: {chapter_id} | Similarity: {similarity_score:.2f}
Source: {source_url}
Content: {chunk_text}
---"""
        formatted_parts.append(chunk_section)

    formatted_context = "\n".join(formatted_parts)
    logger.debug(f"Formatted context with {len(results)} chunks (total length: {len(formatted_context)} chars)")

    return formatted_context


def create_retrieval_tool_schema() -> Dict:
    """
    Define OpenAI function tool schema for retrieval.

    Creates the JSON schema that OpenAI's function calling API uses to understand
    how to call the retrieve_context function.

    Returns:
        Dictionary containing the function tool schema

    Example:
        >>> schema = create_retrieval_tool_schema()
        >>> print(schema["function"]["name"])
        retrieve_context
    """
    schema = {
        "type": "function",
        "function": {
            "name": "retrieve_context",
            "description": "Retrieve relevant content from the Physical AI textbook based on semantic similarity. Use this function to find information needed to answer user questions. Only call this function when you need textbook content to answer a question.",
            "parameters": {
                "type": "object",
                "properties": {
                    "query": {
                        "type": "string",
                        "description": "The search query to find relevant textbook content. Should be a clear, specific question or topic (e.g., 'What is ROS 2?', 'How to create URDF files')"
                    },
                    "top_k": {
                        "type": "integer",
                        "description": "Number of relevant chunks to retrieve (1-10). Default is 5. Use higher values for broad topics.",
                        "default": 5,
                        "minimum": 1,
                        "maximum": 10
                    },
                    "threshold": {
                        "type": "number",
                        "description": "Minimum similarity score (0.0-1.0). Default is 0.70. Higher values return only very relevant results.",
                        "default": 0.70,
                        "minimum": 0.0,
                        "maximum": 1.0
                    }
                },
                "required": ["query"]
            }
        }
    }

    logger.debug("Created retrieval tool schema")
    return schema


def initialize_agent(model: str = None) -> Tuple[str, List[Dict]]:
    """
    Initialize OpenAI agent with retrieval tool.

    Creates a new agent session with a unique session ID and registers the
    retrieval tool for function calling.

    Args:
        model: OpenAI model to use (default: from OPENAI_MODEL env var)

    Returns:
        Tuple of (session_id, initial_messages) where initial_messages contains
        the system prompt

    Example:
        >>> session_id, messages = initialize_agent()
        >>> print(session_id)
        '550e8400-e29b-41d4-a716-446655440000'
    """
    if model is None:
        model = OPENAI_MODEL

    # Generate unique session ID
    session_id = str(uuid.uuid4())

    # Initialize conversation with system prompt
    initial_messages = [
        {"role": "system", "content": SYSTEM_PROMPT}
    ]

    logger.info(f"Initialized agent session {session_id} with model {model}")
    logger.debug(f"System prompt length: {len(SYSTEM_PROMPT)} chars")

    return session_id, initial_messages


def execute_function_call(function_name: str, arguments: Dict) -> str:
    """
    Handle function calls from OpenAI agent.

    Executes the requested function (currently only supports retrieve_context)
    and returns formatted results.

    Args:
        function_name: Name of function to execute (e.g., "retrieve_context")
        arguments: Dictionary of function arguments

    Returns:
        Formatted string result to send back to agent

    Raises:
        ValueError: If function_name is not supported

    Example:
        >>> args = {"query": "What is ROS 2?", "top_k": 5}
        >>> result = execute_function_call("retrieve_context", args)
        >>> print(result[:50])
        'Retrieved Context (3 sources):'
    """
    logger.debug(f"Executing function call: {function_name} with args: {arguments}")

    if function_name == "retrieve_context":
        try:
            # Extract arguments with defaults
            query = arguments.get("query")
            top_k = arguments.get("top_k", 5)
            threshold = arguments.get("threshold", 0.70)

            # Call retrieval function
            results = retrieve_context(query=query, top_k=top_k, threshold=threshold)

            # Format results for agent
            formatted_context = format_context_for_agent(results)

            logger.info(f"Function call successful: retrieved {len(results)} chunks for query '{query[:30]}...'")
            return formatted_context

        except Exception as e:
            error_msg = f"Error retrieving context: {str(e)}"
            logger.error(error_msg)
            return error_msg

    else:
        error_msg = f"Unknown function: {function_name}"
        logger.error(error_msg)
        raise ValueError(error_msg)


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((openai.RateLimitError, openai.APIConnectionError))
)
def ask_question(question: str, session_id: str = None, conversation_history: List[Dict] = None, model: str = None) -> Dict:
    """
    Send question to OpenAI agent and handle function calls.

    Manages the full interaction loop with the OpenAI API including function calling
    for retrieval. Returns structured response with answer and metadata.

    Args:
        question: User's question
        session_id: Optional session ID (generates new one if not provided)
        conversation_history: Optional conversation history (creates new if not provided)
        model: Optional model override (uses OPENAI_MODEL if not provided)

    Returns:
        Dictionary containing:
        - answer: Agent's response text
        - citations: List of source citations
        - retrieval_count: Number of chunks retrieved
        - tokens_used: Total tokens consumed
        - grounded: Whether answer is based on retrieved context
        - session_id: Session identifier

    Raises:
        Exception: If OpenAI API call fails after retries

    Example:
        >>> response = ask_question("What is ROS 2?")
        >>> print(response["answer"])
        'ROS 2 is the next generation Robot Operating System...'
        >>> print(response["grounded"])
        True
    """
    if model is None:
        model = OPENAI_MODEL

    # Initialize session if needed
    if session_id is None or conversation_history is None:
        session_id, conversation_history = initialize_agent(model)

    # Add user question to conversation
    conversation_history.append({"role": "user", "content": question})

    # Get retrieval tool schema
    tools = [create_retrieval_tool_schema()]

    logger.info(f"Processing question: '{question[:50]}...'")
    start_time = datetime.now()

    try:
        # Initial API call
        response = openai_client.chat.completions.create(
            model=model,
            messages=conversation_history,
            tools=tools,
            tool_choice="auto"  # Let the model decide when to call functions
        )

        response_message = response.choices[0].message
        retrieval_count = 0
        citations = []

        # Handle function calls
        while response_message.tool_calls:
            logger.debug(f"Agent requested {len(response_message.tool_calls)} function call(s)")

            # Add assistant's response with tool calls to conversation
            conversation_history.append(response_message)

            # Execute each tool call
            for tool_call in response_message.tool_calls:
                function_name = tool_call.function.name
                function_args = eval(tool_call.function.arguments)  # Parse JSON string to dict

                logger.debug(f"Calling function: {function_name}")

                # Execute function
                function_response = execute_function_call(function_name, function_args)
                retrieval_count += 1

                # Extract citations from function response
                if function_name == "retrieve_context":
                    # Parse results to extract citation info
                    results = retrieve_context(**function_args)
                    for result in results:
                        citations.append({
                            "module_name": result["module_name"],
                            "chapter_id": result["chapter_id"],
                            "source_url": result["source_url"],
                            "similarity_score": result["similarity_score"]
                        })

                # Add function response to conversation
                conversation_history.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "name": function_name,
                    "content": function_response
                })

            # Get next response from agent
            response = openai_client.chat.completions.create(
                model=model,
                messages=conversation_history,
                tools=tools,
                tool_choice="auto"
            )
            response_message = response.choices[0].message

        # Final answer from agent
        final_answer = response_message.content
        conversation_history.append(response_message)

        # Calculate duration and tokens
        duration = (datetime.now() - start_time).total_seconds()
        tokens_used = response.usage.total_tokens

        logger.info(f"Question answered in {duration:.2f}s ({tokens_used} tokens, {retrieval_count} retrievals)")

        # Return structured response
        return {
            "answer": final_answer,
            "citations": citations,
            "retrieval_count": retrieval_count,
            "tokens_used": tokens_used,
            "grounded": retrieval_count > 0,
            "session_id": session_id
        }

    except openai.RateLimitError as e:
        logger.error(f"Rate limit exceeded: {e}")
        raise
    except openai.APIConnectionError as e:
        logger.error(f"API connection error: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in ask_question: {e}")
        raise


# ============================================================================
# Phase 4: Conversation Session Management
# ============================================================================

class ConversationSession:
    """
    Manages multi-turn conversation state.

    Maintains conversation history, session metadata, and enforces history limits
    using a sliding window approach.

    Attributes:
        session_id: Unique session identifier (UUID)
        model: OpenAI model being used
        conversation_history: List of message dictionaries (role + content)
        max_history: Maximum number of messages to retain (sliding window)
        started_at: Session creation timestamp
        last_activity: Last message timestamp

    Example:
        >>> session = ConversationSession(model="gpt-4", max_history=20)
        >>> print(session.session_id)
        '550e8400-e29b-41d4-a716-446655440000'
        >>> add_message(session, "user", "What is ROS 2?")
    """

    def __init__(self, model: str = None, max_history: int = None):
        """
        Initialize a new conversation session.

        Args:
            model: OpenAI model to use (default: OPENAI_MODEL from env)
            max_history: Maximum conversation history length (default: AGENT_MAX_HISTORY from env)
        """
        self.session_id = str(uuid.uuid4())
        self.model = model if model else OPENAI_MODEL
        self.max_history = max_history if max_history else AGENT_MAX_HISTORY
        self.started_at = datetime.now()
        self.last_activity = datetime.now()

        # Initialize with system prompt
        self.conversation_history = [
            {"role": "system", "content": SYSTEM_PROMPT}
        ]

        logger.info(f"Created conversation session {self.session_id} (model={self.model}, max_history={self.max_history})")


def add_message(session: ConversationSession, role: str, content: str) -> None:
    """
    Add a message to conversation history with sliding window enforcement.

    Appends the message and enforces max_history limit by removing oldest
    user-assistant pairs when limit is exceeded. System message is always preserved.

    Args:
        session: ConversationSession instance
        role: Message role ("user", "assistant", "system", "tool")
        content: Message content

    Example:
        >>> session = ConversationSession()
        >>> add_message(session, "user", "What is ROS 2?")
        >>> add_message(session, "assistant", "ROS 2 is...")
        >>> print(len(session.conversation_history))
        3  # system + user + assistant
    """
    # Add the message
    session.conversation_history.append({
        "role": role,
        "content": content
    })
    session.last_activity = datetime.now()

    # Enforce max_history (keep system message + max_history messages)
    max_total = session.max_history + 1  # +1 for system message
    if len(session.conversation_history) > max_total:
        # Calculate how many to remove
        excess = len(session.conversation_history) - max_total

        # Remove oldest messages (skip system message at index 0)
        # Remove in pairs (user + assistant) to maintain conversation flow
        messages_to_remove = excess if excess % 2 == 0 else excess + 1

        logger.debug(f"Enforcing max_history: removing {messages_to_remove} oldest messages")

        # Keep system message, remove from index 1 onwards
        session.conversation_history = (
            [session.conversation_history[0]] +  # System message
            session.conversation_history[1 + messages_to_remove:]  # Recent messages
        )

    logger.debug(f"Session {session.session_id}: {len(session.conversation_history)} messages in history")


def reset_conversation(session: ConversationSession) -> None:
    """
    Reset conversation history to initial state.

    Clears all messages except the system prompt and resets the session timestamp.

    Args:
        session: ConversationSession instance

    Example:
        >>> session = ConversationSession()
        >>> add_message(session, "user", "Question 1")
        >>> add_message(session, "assistant", "Answer 1")
        >>> reset_conversation(session)
        >>> print(len(session.conversation_history))
        1  # Only system message remains
    """
    logger.info(f"Resetting conversation {session.session_id} ({len(session.conversation_history) - 1} messages cleared)")

    # Keep only system message
    session.conversation_history = [
        {"role": "system", "content": SYSTEM_PROMPT}
    ]
    session.last_activity = datetime.now()

    logger.debug(f"Session {session.session_id} reset to initial state")


# ============================================================================
# CLI Interface Functions
# ============================================================================

def single_question_mode(question: str, args: argparse.Namespace) -> int:
    """
    Handle single-question mode for CLI.

    Processes a single question, prints the answer and optionally citations,
    then exits.

    Args:
        question: User's question
        args: Parsed command-line arguments

    Returns:
        Exit code (0 for success, 1 for failure)

    Example:
        >>> args = parse_arguments()
        >>> exit_code = single_question_mode("What is ROS 2?", args)
        >>> # Prints answer and exits
    """
    try:
        logger.info(f"Single-question mode: '{question}'")
        start_time = datetime.now()

        # Get model and retrieval parameters from args
        model = args.model if hasattr(args, 'model') else OPENAI_MODEL
        threshold = args.threshold if hasattr(args, 'threshold') else 0.70
        top_k = args.top_k if hasattr(args, 'top_k') else 5

        # Store for potential function call overrides
        # (Note: These would be used in a more sophisticated implementation)

        # Ask the question
        response = ask_question(question=question, model=model)

        # Print answer
        print("\n" + "=" * 80)
        print("ANSWER:")
        print("=" * 80)
        print(response["answer"])
        print()

        # Print citations if requested
        if hasattr(args, 'show_citations') and args.show_citations and response["citations"]:
            print("=" * 80)
            print("CITATIONS:")
            print("=" * 80)
            for idx, citation in enumerate(response["citations"], 1):
                print(f"[{idx}] {citation['module_name']} - {citation['chapter_id']}")
                print(f"    URL: {citation['source_url']}")
                print(f"    Similarity: {citation['similarity_score']:.3f}")
                print()

        # Print metadata
        duration = (datetime.now() - start_time).total_seconds()
        print("=" * 80)
        print(f"Duration: {duration:.2f}s | Tokens: {response['tokens_used']} | Retrievals: {response['retrieval_count']} | Grounded: {response['grounded']}")
        print("=" * 80)

        logger.info(f"Single-question mode completed successfully in {duration:.2f}s")
        return 0

    except Exception as e:
        logger.error(f"Single-question mode failed: {e}")
        print(f"\n❌ Error: {e}\n", file=sys.stderr)
        return 1


def interactive_mode(args: argparse.Namespace) -> int:
    """
    Handle interactive conversation mode for CLI.

    Creates a conversation session and enters a loop where the user can ask
    multiple questions, with context maintained across turns. Supports special
    commands like reset, help, and exit.

    Args:
        args: Parsed command-line arguments

    Returns:
        Exit code (0 for success, 1 for failure)

    Example:
        >>> args = parse_arguments()
        >>> exit_code = interactive_mode(args)
        >>> # Enters interactive loop
    """
    try:
        # Get parameters from args
        model = args.model if hasattr(args, 'model') else OPENAI_MODEL
        max_history = args.max_history if hasattr(args, 'max_history') else AGENT_MAX_HISTORY

        # Create conversation session
        session = ConversationSession(model=model, max_history=max_history)

        # Print welcome message
        print("\n" + "=" * 80)
        print("RAG Agent - Interactive Mode")
        print("=" * 80)
        print(f"Session ID: {session.session_id}")
        print(f"Model: {session.model}")
        print(f"Max History: {session.max_history} messages")
        print("\nAsk questions about the Physical AI textbook. Type 'exit', 'quit', or 'bye' to quit.")
        print("Special commands: 'reset' (clear history), 'help' (show commands)")
        print("=" * 80 + "\n")

        logger.info(f"Started interactive mode with session {session.session_id}")

        # Main conversation loop
        turn_count = 0
        while True:
            try:
                # Get user input
                user_input = input("\n💬 You: ").strip()

                if not user_input:
                    continue

                # Handle special commands
                if user_input.lower() in ['exit', 'quit', 'bye', 'q']:
                    print("\n👋 Goodbye! Session ended.\n")
                    logger.info(f"Interactive mode ended after {turn_count} turns")
                    break

                elif user_input.lower() == 'reset':
                    reset_conversation(session)
                    turn_count = 0
                    print("\n✅ Conversation reset. Starting fresh!\n")
                    continue

                elif user_input.lower() == 'help':
                    print("\n📚 Available Commands:")
                    print("  - Type any question to get an answer")
                    print("  - 'reset'  : Clear conversation history")
                    print("  - 'help'   : Show this help message")
                    print("  - 'exit'   : Exit interactive mode (also: quit, bye, q)")
                    print(f"\n📊 Current Session:")
                    print(f"  - Messages: {len(session.conversation_history) - 1} (max: {session.max_history})")
                    print(f"  - Turns: {turn_count}")
                    print(f"  - Started: {session.started_at.strftime('%Y-%m-%d %H:%M:%S')}")
                    continue

                # Process question
                turn_count += 1
                logger.info(f"Turn {turn_count}: Processing question in session {session.session_id}")
                start_time = datetime.now()

                # Ask question with conversation history
                response = ask_question(
                    question=user_input,
                    session_id=session.session_id,
                    conversation_history=session.conversation_history.copy(),
                    model=session.model
                )

                # Update session history with the new messages
                # Note: ask_question modifies the passed conversation_history
                # We need to sync it back to the session
                # The response already includes the full updated history from ask_question
                # So we extract just the new user message and assistant response

                # Add user message (if not already added by ask_question)
                if session.conversation_history[-1]["role"] != "user" or session.conversation_history[-1]["content"] != user_input:
                    add_message(session, "user", user_input)

                # Add assistant response
                add_message(session, "assistant", response["answer"])

                # Print answer
                duration = (datetime.now() - start_time).total_seconds()
                print(f"\n🤖 Agent:")
                print("-" * 80)
                print(response["answer"])
                print("-" * 80)

                # Print citations if available
                if response["citations"]:
                    print(f"\n📖 Sources ({len(response['citations'])}):")
                    for idx, citation in enumerate(response["citations"][:3], 1):  # Show top 3
                        print(f"  [{idx}] {citation['module_name']} - {citation['chapter_id']} (score: {citation['similarity_score']:.2f})")

                # Print metadata
                print(f"\n⏱️  {duration:.2f}s | 🔍 {response['retrieval_count']} retrievals | 🎯 Grounded: {response['grounded']}")

            except KeyboardInterrupt:
                print("\n\n⚠️  Use 'exit' or 'quit' to end the session.\n")
                continue

            except Exception as e:
                logger.error(f"Error in interactive loop: {e}")
                print(f"\n❌ Error processing question: {e}\n", file=sys.stderr)
                continue

        return 0

    except Exception as e:
        logger.error(f"Interactive mode failed: {e}")
        print(f"\n❌ Fatal error in interactive mode: {e}\n", file=sys.stderr)
        return 1


def parse_arguments() -> argparse.Namespace:
    """
    Parse command-line arguments.

    Returns:
        Parsed arguments namespace

    Example:
        >>> args = parse_arguments()
        >>> print(args.question)
        'What is ROS 2?'
    """
    parser = argparse.ArgumentParser(
        description="RAG Agent for Physical AI Textbook - Ask questions and get grounded answers from the textbook content",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Single question
  python agent.py "What is ROS 2?"

  # Show citations
  python agent.py "What is Isaac Sim?" --show-citations

  # Use different model
  python agent.py "How do I create URDF?" --model gpt-4-turbo

  # Adjust retrieval parameters
  python agent.py "What is VSLAM?" --top-k 3 --threshold 0.75

  # Interactive mode (future feature)
  python agent.py --interactive
        """
    )

    # Positional argument
    parser.add_argument(
        "question",
        nargs="?",
        type=str,
        help="Question to ask about the textbook (required unless using --interactive)"
    )

    # Optional arguments
    parser.add_argument(
        "--model",
        type=str,
        default=OPENAI_MODEL,
        help=f"OpenAI model to use (default: {OPENAI_MODEL})"
    )

    parser.add_argument(
        "--show-citations",
        action="store_true",
        help="Display detailed citation information"
    )

    parser.add_argument(
        "--log-level",
        type=str,
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level (default: INFO)"
    )

    parser.add_argument(
        "--threshold",
        type=float,
        default=0.70,
        help="Minimum similarity threshold for retrieval (0.0-1.0, default: 0.70)"
    )

    parser.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="Number of chunks to retrieve (1-10, default: 5)"
    )

    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Start interactive conversation mode"
    )

    parser.add_argument(
        "--max-history",
        type=int,
        default=AGENT_MAX_HISTORY,
        help=f"Maximum conversation history length for interactive mode (default: {AGENT_MAX_HISTORY})"
    )

    return parser.parse_args()


def main() -> int:
    """
    Main entry point for CLI.

    Parses arguments, validates environment, and routes to appropriate mode.

    Returns:
        Exit code (0 for success, 1 for failure)
    """
    try:
        # Parse arguments
        args = parse_arguments()

        # Update log level if specified
        if args.log_level != log_level:
            configure_logger(args.log_level)

        # Validate environment is loaded
        if not OPENAI_API_KEY or not COHERE_API_KEY:
            logger.error("Environment variables not properly loaded")
            print("❌ Error: Missing required API keys. Check your .env file.", file=sys.stderr)
            return 1

        # Check mode
        if args.interactive:
            # Interactive conversation mode
            logger.info("Starting interactive mode")
            return interactive_mode(args)

        # Validate question provided
        if not args.question:
            logger.error("No question provided")
            print("❌ Error: Please provide a question as an argument.", file=sys.stderr)
            print("Example: python agent.py \"What is ROS 2?\"", file=sys.stderr)
            print("Run with --help for more information.", file=sys.stderr)
            return 1

        # Run single-question mode
        return single_question_mode(args.question, args)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        print("\n\nInterrupted by user.", file=sys.stderr)
        return 1
    except Exception as e:
        logger.error(f"Unexpected error in main: {e}")
        print(f"\n❌ Unexpected error: {e}\n", file=sys.stderr)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
