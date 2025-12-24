# Quickstart: Retrieval-Enabled Agent

**Feature**: 009-retrieval-agent
**Script**: `backend/agent.py`
**Purpose**: Command-line RAG agent for answering questions based on Physical AI textbook content

---

## Prerequisites

1. **Completed Embedding Pipeline** (Feature 007): Qdrant collection "rag_embedding" must be populated
2. **Python Environment**: Backend virtual environment with dependencies installed
3. **API Access**: Valid OpenAI API key (new requirement) + existing Cohere and Qdrant credentials
4. **Retrieval Validation** (Feature 008): Recommended to run validation tool first to verify retrieval quality

**Verify Prerequisites**:
```bash
# Check backend environment exists
ls backend/.venv/

# Check .env has required keys
cat backend/.env | grep -E "OPENAI_API_KEY|COHERE_API_KEY|QDRANT"

# Verify Qdrant has data (feature 008 validation tool)
cd backend
.venv/Scripts/python.exe retrieve.py
```

---

## Installation

### 1. Add OpenAI dependency to backend

```bash
cd backend

# Option A: Using UV (recommended)
uv pip install openai>=1.0.0

# Option B: Using pip
.venv/Scripts/pip.exe install openai>=1.0.0
```

### 2. Update .env with OpenAI API key

```bash
# Edit backend/.env and add:
OPENAI_API_KEY=your_openai_key_here
OPENAI_MODEL=gpt-4  # Optional, defaults to gpt-4
AGENT_MAX_HISTORY=20  # Optional conversation history limit
```

### 3. Verify installation

```bash
cd backend
.venv/Scripts/python.exe -c "import openai; print(openai.__version__)"
# Should output: 1.x.x
```

---

## Quick Start (5 Minutes)

### Single Question Mode

Ask one question and get an answer:

```bash
cd backend
.venv/Scripts/python.exe agent.py "What is ROS 2?"
```

**Expected Output**:
```
2025-12-23 18:30:00 | INFO     | Initializing RAG Agent (model: gpt-4)
2025-12-23 18:30:01 | INFO     | Connected to Qdrant at https://your-qdrant.io
2025-12-23 18:30:02 | INFO     | Retrieved 3 relevant chunks (similarity >= 0.70)

ROS 2 is a flexible framework for writing robot software [Source: Module 1 - ROS 2 Basics].
It provides services including hardware abstraction, device drivers, and message-passing between
processes.

2025-12-23 18:30:10 | INFO     | Response generated in 8.2 seconds
```

### Interactive Conversation Mode

Have a multi-turn conversation:

```bash
cd backend
.venv/Scripts/python.exe agent.py --interactive
```

**Expected Interaction**:
```
2025-12-23 18:30:00 | INFO     | Starting interactive mode...
2025-12-23 18:30:01 | INFO     | Type 'exit' to quit, 'reset' to clear history, 'help' for commands

You: What is Isaac Sim?
Assistant: Isaac Sim is NVIDIA's robotics simulation platform [Source: Module 3 - Isaac]...

You: How do I use it for synthetic data?
Assistant: To use Isaac Sim for synthetic data generation [Source: Module 3 - Isaac]...

You: exit
Goodbye!
```

---

## Configuration

### Environment Variables (.env)

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `OPENAI_API_KEY` | ✅ Yes | - | OpenAI API key for agent |
| `COHERE_API_KEY` | ✅ Yes | - | Cohere API key for query embeddings |
| `QDRANT_URL` | ✅ Yes | - | Qdrant instance URL |
| `QDRANT_API_KEY` | ✅ Yes | - | Qdrant API key |
| `OPENAI_MODEL` | No | `gpt-4` | OpenAI model to use |
| `AGENT_MAX_HISTORY` | No | `20` | Max conversation history messages |
| `LOG_LEVEL` | No | `INFO` | Logging level |

### CLI Arguments

```bash
python agent.py [question] [options]

Positional:
  question              Question to ask (single-shot mode)

Options:
  --interactive         Start interactive conversation mode
  --model MODEL         OpenAI model to use (default: gpt-4)
  --max-history N       Max conversation history (default: 20)
  --show-citations      Display detailed citations
  --log-level LEVEL     Logging level (DEBUG, INFO, WARNING, ERROR)
  --threshold FLOAT     Min similarity threshold (default: 0.70)
  --top-k N             Number of chunks to retrieve (default: 5)
```

---

## Common Use Cases

### 1. Quick Answer Lookup

```bash
# Get a quick answer to a specific question
.venv/Scripts/python.exe agent.py "What is URDF?"
```

### 2. Learning Session

```bash
# Start interactive mode to explore topics
.venv/Scripts/python.exe agent.py --interactive

You: What is ROS 2?
Assistant: ...

You: What are ROS 2 topics?
Assistant: ...

You: Give me an example of publishing to a topic
Assistant: ...
```

### 3. Debugging with Detailed Citations

```bash
# Show full citation details for verification
.venv/Scripts/python.exe agent.py "What is VSLAM?" --show-citations
```

**Output**:
```
VSLAM (Visual Simultaneous Localization and Mapping) is...

Citations:
1. Chunk ID: uuid-abc123
   Module: Module 3 Isaac
   Chapter: VSLAM with Isaac ROS
   Similarity: 0.89
   URL: https://physical-ai-robotics-textbook-xi.vercel.app/docs/module-3-isaac/ch4-vslam
   Tokens: 512

2. Chunk ID: uuid-def456
   Module: Module 3 Isaac
   Similarity: 0.82
   ...
```

### 4. Using Different Models

```bash
# Use GPT-4 Turbo for faster responses
.venv/Scripts/python.exe agent.py --interactive --model gpt-4-turbo

# Use GPT-3.5 for cost optimization (less accurate)
.venv/Scripts/python.exe agent.py "What is Nav2?" --model gpt-3.5-turbo
```

### 5. Adjusting Retrieval Parameters

```bash
# More chunks for complex questions
.venv/Scripts/python.exe agent.py "Explain the entire ROS 2 architecture" --top-k 10

# Lower threshold for broader retrieval
.venv/Scripts/python.exe agent.py "robotics basics" --threshold 0.60
```

---

## Troubleshooting

### Issue: "OPENAI_API_KEY not set"

**Cause**: Missing OpenAI API key in .env file

**Fix**:
```bash
# Edit backend/.env and add your key
nano backend/.env

# Add this line:
OPENAI_API_KEY=sk-...your-key...

# Verify
cat backend/.env | grep OPENAI_API_KEY
```

### Issue: "No relevant content found"

**Cause**: Query doesn't match any textbook content (similarity < threshold)

**Fix**:
1. Rephrase query to use terms from the textbook
2. Lower similarity threshold: `--threshold 0.60`
3. Check if content exists in Qdrant: `python retrieve.py`

**Example**:
```bash
# Instead of generic query:
.venv/Scripts/python.exe agent.py "how to build robots"

# Use textbook-specific terminology:
.venv/Scripts/python.exe agent.py "How do I create a URDF model?"
```

### Issue: "Rate limit exceeded"

**Cause**: OpenAI API rate limit hit

**Fix**:
1. Wait and retry (automatic exponential backoff after 3 attempts)
2. Reduce conversation frequency
3. Upgrade OpenAI API plan

### Issue: "Token limit exceeded"

**Cause**: Conversation history too long

**Fix**:
```bash
# In interactive mode, type:
You: reset

# Or reduce max history:
.venv/Scripts/python.exe agent.py --interactive --max-history 10
```

### Issue: "Agent hallucinating or using external knowledge"

**Cause**: System prompt not enforcing grounding correctly

**Fix**:
1. Enable debug logging to inspect retrieval:
   ```bash
   .venv/Scripts/python.exe agent.py "..." --log-level DEBUG
   ```
2. Verify retrieved chunks are relevant
3. Report issue with specific query and response

### Issue: "Qdrant connection failed"

**Cause**: Qdrant URL or API key incorrect

**Fix**:
```bash
# Verify Qdrant configuration
cat backend/.env | grep QDRANT

# Test connection (feature 008 tool)
.venv/Scripts/python.exe retrieve.py
```

---

## Advanced Usage

### Scripting and Automation

```bash
# Answer multiple questions in batch
for question in "What is ROS 2?" "What is URDF?" "What is Isaac Sim?"; do
    .venv/Scripts/python.exe agent.py "$question"
done

# Save answers to file
.venv/Scripts/python.exe agent.py "What is Nav2?" > answer.txt
```

### Custom System Prompts (Future Enhancement)

Currently, system prompt is hardcoded for grounding. Future versions may support custom prompts via `--system-prompt` flag.

### Conversation Export (Future Enhancement)

Future versions may support exporting conversation history to JSON:
```bash
.venv/Scripts/python.exe agent.py --interactive --export conversation.json
```

---

## Testing the Agent

### Test with Known Questions

Use test queries from feature 008 validation tool:

```bash
# From retrieve.py TEST_QUERIES
.venv/Scripts/python.exe agent.py "What is ROS 2?"
.venv/Scripts/python.exe agent.py "How do I create a URDF file for a humanoid robot?"
.venv/Scripts/python.exe agent.py "What is Isaac Sim?"
```

### Test Edge Cases

**Out-of-scope question**:
```bash
.venv/Scripts/python.exe agent.py "What's the weather today?"
# Expected: "I don't have information about that in the textbook..."
```

**Ambiguous question**:
```bash
.venv/Scripts/python.exe agent.py "What is it?"
# Expected: Agent asks for clarification
```

**Multi-part question**:
```bash
.venv/Scripts/python.exe agent.py "What is URDF and how is it used in ROS 2?"
# Expected: Answer covers both concepts
```

---

## Performance Expectations

| Metric | Target | Typical |
|--------|--------|---------|
| **Retrieval Latency** | <2s (p95) | 1-1.5s |
| **End-to-End Response** | <10s (p95) | 6-8s |
| **Grounded Answers** | 90%+ | 95%+ |
| **Out-of-Scope Refusal** | 100% | 100% |
| **Citation Accuracy** | 95%+ | 98%+ |

**Factors Affecting Performance**:
- OpenAI model choice (gpt-4 vs gpt-4-turbo vs gpt-3.5)
- Number of retrieved chunks (top-k)
- Qdrant instance location (latency)
- Network speed

---

## Next Steps

1. **Run validation tool** (feature 008) to verify retrieval quality before using agent
2. **Test single-question mode** with a few known questions
3. **Try interactive mode** for a learning session
4. **Experiment with parameters** (model, top-k, threshold) to find optimal settings
5. **Report issues** or suggest improvements via PHR or feature request

---

## Success Checklist

- [ ] OpenAI dependency installed (`openai>=1.0.0`)
- [ ] OPENAI_API_KEY added to `.env`
- [ ] Agent runs without errors (`python agent.py "What is ROS 2?"`)
- [ ] Retrieval works (chunks retrieved with similarity >= 0.70)
- [ ] Answer is grounded (cites sources from textbook)
- [ ] Out-of-scope questions are refused (e.g., "What's the weather?")
- [ ] Interactive mode works (multi-turn conversation with context)
- [ ] Response time meets targets (<10s end-to-end)

**If all checks pass**: ✅ RAG agent is ready for use!

---

## See Also

- **Feature 007 (Embedding Pipeline)**: How textbook content is ingested into Qdrant
- **Feature 008 (Retrieval Validation)**: How to validate retrieval quality before using agent
- **spec.md**: Complete feature specification with user stories and requirements
- **data-model.md**: Entity definitions and data structures
- **contracts/agent-functions.yaml**: Detailed function signatures and schemas