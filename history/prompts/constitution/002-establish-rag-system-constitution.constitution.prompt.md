---
id: 002
title: Establish RAG System Constitution
stage: constitution
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 006-docusaurus-landing
user: mali-shoaib
command: /sp.constitution
labels: ["constitution", "rag", "governance", "principles"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Manual validation of constitution structure
  - Template consistency check
---

## Prompt

Unified RAG System

## Purpose
Build a **grounded RAG system** that extracts content from a deployed Docusaurus book, embeds it, stores it in Qdrant, retrieves relevant chunks, and answers user queries **strictly from stored content**.

---

## Scope
**Included**
- URL crawling, text cleaning, chunking
- Cohere embeddings
- Qdrant vector storage & similarity search
- Retrieval-enabled agent using OpenAI Agents SDK
- FastAPI backend
- Docusaurus (React) chatbot integration

**Excluded**
- Authentication / user management
- Advanced UI/UX
- Production deployment
- External knowledge usage

---

## Hard Constraints
- **Embeddings**: Cohere
- ❌ No answer if similarity score < threshold
- ✅ If insufficient context: **"Information not found in the book."**

---

## Quality Gates
- At least one retrieved chunk above threshold
- Agent answers must be fully grounded in retrieved chunks
- End-to-end local test must pass:
  URL → Embedding → Storage → Retrieval → Answer
- Docusaurus chatbot must return valid RAG responses

---

## Definition of Success
A reader can open the Docusaurus book, ask a question via the chatbot, and receive an answer **only** based on the book's embedded content, verified locally end-to-end.

## Response snapshot

Created Physical AI Textbook RAG System Constitution v1.0.0 with 6 core principles:

1. **Grounded Answers Only (NON-NEGOTIABLE)**: All answers strictly from retrieved content
2. **Cohere Embeddings Standard**: Exclusive use of Cohere for embeddings
3. **Qdrant Vector Storage**: Single source of truth for vector storage
4. **Content Extraction Pipeline**: Crawl → Clean → Chunk → Embed → Store
5. **Agent-Based Retrieval**: OpenAI Agents SDK with retrieval tool integration
6. **Docusaurus Chatbot Integration**: Seamless React component integration

Added sections:
- Technical Architecture (stack requirements, data flow, quality thresholds)
- Development Workflow (ingestion, development cycle, quality gates)
- Governance (amendment process, versioning policy, compliance review)

Key constraints enforced:
- Similarity threshold minimum: 0.70
- Zero hallucination tolerance
- Cohere-only embeddings (no provider mixing)
- Mandatory retrieval step before all answers

Constitution file: `.specify/memory/constitution.md`

## Outcome

- ✅ Impact: Established foundational governance for RAG system development
- 🧪 Tests: Manual validation passed, structure verified
- 📁 Files: Created constitution.md (218 lines)
- 🔁 Next prompts: Review and update plan/spec/tasks templates for RAG alignment
- 🧠 Reflection: Constitution provides clear guardrails for grounded RAG implementation

## Evaluation notes (flywheel)

- Failure modes observed: None - initial constitution creation
- Graders run and results (PASS/FAIL): Manual review PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Create RAG system spec using /sp.specify
