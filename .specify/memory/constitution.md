<!--
Sync Impact Report - Version 1.0.0
═══════════════════════════════════════════════════════════════════════════════
Version Change: NEW → 1.0.0 (Initial constitution for Unified RAG System)

Modified Principles: N/A (initial creation)

Added Sections:
  - Core Principles (6 principles)
  - Technical Architecture
  - Development Workflow
  - Governance

Removed Sections: N/A

Template Updates:
  ✅ Constitution created
  ⚠️ Plan template (.specify/templates/plan-template.md) - needs review for RAG principles
  ⚠️ Spec template (.specify/templates/spec-template.md) - needs review for RAG principles
  ⚠️ Tasks template (.specify/templates/tasks-template.md) - needs review for RAG principles

Follow-up TODOs:
  - Review and update plan/spec/tasks templates to align with RAG grounding principles
  - Add RAG-specific quality gates to templates
  - Update command files to reference new constitution principles

Rationale for Version 1.0.0:
  - Initial constitution establishment for the Physical AI Textbook RAG System
  - Defines foundational principles for grounded RAG implementation
  - Sets governance structure for the project
═══════════════════════════════════════════════════════════════════════════════
-->

# Physical AI Textbook RAG System Constitution

## Core Principles

### I. Grounded Answers Only (NON-NEGOTIABLE)

**All answers MUST be strictly derived from retrieved content stored in the vector database.**

- ❌ No answer generation if similarity score < threshold
- ✅ If insufficient context: Return **"Information not found in the book."**
- ❌ No external knowledge, hallucinations, or assumptions
- ✅ Agent responses MUST cite retrieved chunks
- ✅ Every answer traceable to specific book content

**Rationale**: Trust and accuracy are paramount. Students and developers rely on factual,
grounded information. Hallucinated or assumed content undermines the textbook's credibility
and educational value.

### II. Cohere Embeddings Standard

**Embeddings MUST use Cohere's embedding models exclusively.**

- ✅ Cohere API for all text-to-vector conversions
- ✅ Consistent embedding model across ingestion and retrieval
- ❌ No mixing of embedding providers (OpenAI, HuggingFace, etc.)
- ✅ Model version pinning for reproducibility

**Rationale**: Embedding consistency ensures semantic search quality. Mixing providers
creates vector space mismatches that degrade retrieval accuracy and system reliability.

### III. Qdrant Vector Storage

**All embedded content MUST be stored and retrieved from Qdrant vector database.**

- ✅ Qdrant as the single source of truth for vector storage
- ✅ Similarity search with configurable threshold
- ✅ Metadata preservation (source URL, chunk ID, chapter, module)
- ✅ Collection structure: one collection per deployment environment
- ❌ No fallback to alternative vector stores in production

**Rationale**: Qdrant provides high-performance vector search with filtering capabilities
essential for chapter-level context scoping and retrieval accuracy.

### IV. Content Extraction Pipeline

**Web content extraction MUST follow: Crawl → Clean → Chunk → Embed → Store.**

- ✅ URL crawling of deployed Docusaurus site
- ✅ HTML-to-text cleaning (preserve semantic structure)
- ✅ Chunking strategy: semantic boundaries (headers, paragraphs)
- ✅ Chunk size: 512-1024 tokens (configurable)
- ✅ Chunk overlap: 10-20% for context continuity
- ❌ No manual content ingestion without validation

**Rationale**: Structured pipeline ensures content quality, reproducibility, and
consistent retrieval performance. Semantic chunking preserves educational context.

### V. Agent-Based Retrieval

**Query processing MUST use OpenAI Agents SDK with retrieval tool integration.**

- ✅ FastAPI backend exposing retrieval endpoints
- ✅ Agent tool: `retrieve_context(query, top_k, threshold)`
- ✅ Agent MUST ground answers in retrieved chunks
- ✅ Threshold enforcement: reject low-similarity results
- ❌ No direct LLM calls without retrieval step

**Rationale**: Agents provide structured reasoning while enforcing grounding constraints.
Tool-based retrieval ensures every answer is validated against stored content.

### VI. Docusaurus Chatbot Integration

**Frontend chatbot MUST integrate seamlessly with Docusaurus React application.**

- ✅ React component embedded in Docusaurus theme
- ✅ API calls to FastAPI backend for RAG responses
- ✅ Display retrieved sources/citations to user
- ✅ Conversation history (optional, session-based)
- ❌ No standalone chatbot deployments outside Docusaurus

**Rationale**: Integration provides unified user experience. Students interact with
content and chatbot in a single interface, improving learning workflow.

## Technical Architecture

### Stack Requirements

**Frontend**:
- Docusaurus 3.x
- React 18+
- TypeScript (preferred)
- Chatbot component: custom React component

**Backend**:
- FastAPI (Python 3.10+)
- OpenAI Agents SDK
- Cohere Python SDK
- Qdrant Python client

**Infrastructure**:
- Qdrant Cloud (or self-hosted Qdrant)
- Vercel/Netlify for Docusaurus deployment
- Backend deployment: Railway/Render/AWS Lambda

### Data Flow

```
User Query → Docusaurus Chatbot → FastAPI Backend → Cohere Embedding →
Qdrant Similarity Search → Retrieved Chunks → OpenAI Agent →
Grounded Answer → FastAPI Response → Chatbot Display
```

### Quality Thresholds

- **Similarity Score Minimum**: 0.70 (configurable)
- **Top-K Retrieval**: 3-5 chunks
- **Chunk Size**: 512-1024 tokens
- **Chunk Overlap**: 10-20%
- **Response Time Target**: < 3s (p95)

## Development Workflow

### Ingestion Workflow

1. **Crawl**: Extract all published Docusaurus pages from production URL
2. **Clean**: Remove navigation, footers, ads; preserve markdown structure
3. **Chunk**: Semantic chunking by headers/paragraphs
4. **Embed**: Generate Cohere embeddings for each chunk
5. **Store**: Upload to Qdrant with metadata (URL, chapter, module)
6. **Validate**: Query test cases to verify retrieval accuracy

### Development Cycle

1. **Local Testing**: End-to-end test (URL → Answer) before deployment
2. **Threshold Tuning**: Adjust similarity threshold based on retrieval quality
3. **Chunk Optimization**: Iterate chunk size/overlap if retrieval fails
4. **Agent Refinement**: Update agent prompts to enforce grounding
5. **Integration Testing**: Verify Docusaurus chatbot → backend → Qdrant flow

### Quality Gates

**Before Deployment**:
- ✅ At least one retrieved chunk above threshold for 95% of test queries
- ✅ Zero hallucinated answers in manual review (50+ sample queries)
- ✅ End-to-end local test passes: URL → Embedding → Storage → Retrieval → Answer
- ✅ Docusaurus chatbot returns valid RAG responses
- ✅ Backend API response time < 3s (p95)

**Post-Deployment**:
- ✅ Weekly retrieval quality audit (sample 100 queries)
- ✅ User feedback loop for "Information not found" responses
- ✅ Embedding model version tracking and upgrade path

## Governance

### Amendment Process

1. **Proposal**: Document proposed change with rationale
2. **Impact Analysis**: Assess effects on code, templates, deployment
3. **Approval**: Team review and consensus
4. **Migration Plan**: Define rollout steps, backward compatibility
5. **Version Bump**: Update constitution version (semantic versioning)
6. **Template Sync**: Update plan/spec/tasks templates to reflect changes

### Versioning Policy

- **MAJOR (X.0.0)**: Backward-incompatible changes (e.g., remove Cohere, change stack)
- **MINOR (X.Y.0)**: New principles, significant additions (e.g., add authentication)
- **PATCH (X.Y.Z)**: Clarifications, typo fixes, non-semantic refinements

### Compliance Review

- ✅ All PRs/reviews MUST verify compliance with Core Principles
- ✅ Grounding violations (hallucination, external knowledge) MUST be rejected
- ✅ Embedding provider changes require MAJOR version bump
- ✅ Constitution supersedes all other practices and documentation

### Runtime Guidance

For detailed development guidance, see:
- **CLAUDE.md**: Agent-specific development rules
- **README.md**: Project setup and contribution guide
- **specs/*/plan.md**: Feature-specific architecture decisions

**Version**: 1.0.0 | **Ratified**: 2025-12-23 | **Last Amended**: 2025-12-23
