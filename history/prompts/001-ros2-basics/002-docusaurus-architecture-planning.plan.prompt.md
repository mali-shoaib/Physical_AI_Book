---
id: 002
title: Docusaurus Architecture Planning
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: 001-ros2-basics
branch: 001-ros2-basics
user: mali-shoaib
command: /sp.plan
labels: ["planning", "architecture", "docusaurus", "rag", "book-structure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-ros2-basics/plan.md
 - specs/001-ros2-basics/research.md
 - specs/001-ros2-basics/data-model.md
 - specs/001-ros2-basics/quickstart.md
 - specs/001-ros2-basics/contracts/sidebar-schema.json
 - specs/001-ros2-basics/contracts/frontmatter-schema.yaml
 - specs/001-ros2-basics/contracts/rag-api.yaml
tests:
 - null
---

## Prompt

```text
Create: book architecture sketch, chapter structure, writing workflow, and quality-validation steps.

Decisions to document:
- Docusaurus structure (folders, sidebar, versioning)
- Chapter organization for all modules
- Code-example format (ROS 2, URDF, Python)
- RAG chatbot placement and integration strategy

Testing strategy:
- Validate all chapters build in Docusaurus
- Check code blocks for syntax accuracy
- Ensure links, images, and sidebar routing work
- Confirm book passes RAG extraction tests (text chunking, embedding, retrieval)

Technical details:
- Write and refine chapters incrementally
- Organize by phases: Draft → Build → Test → Refine → Publish
- Output in Markdown (Docusaurus-compatible)
```

## Response snapshot

Completed comprehensive planning for Docusaurus-based Physical AI & Humanoid Robotics textbook with integrated RAG chatbot.

**Phase 0: Research (completed)**
- Researched Docusaurus 3.x configuration with 4 parallel agents (sidebar config, code examples, RAG integration, GitHub Pages deployment)
- Key decisions:
  - Manual sidebars.js with categorical hierarchy (better control for 12+ chapters)
  - Hybrid code approach: inline (<20 lines) + external links (>50 lines)
  - RAG chunking: heading-based boundaries + semantic segmentation (500-1000 tokens)
  - Include code blocks in embeddings as-is (better retrieval quality)
  - FastAPI backend + React component integration
  - GitHub Actions with `deploy-pages@v4` + multi-layer caching

**Phase 1: Design (completed)**
- Created `data-model.md` defining 6 entities:
  - Module, Chapter, ChapterMetadata, CodeExample, RAGChunk, SidebarConfig
  - Validation rules and ER diagram
- Generated contracts:
  - `sidebar-schema.json`: JSON Schema for sidebar validation
  - `frontmatter-schema.yaml`: YAML template for chapter frontmatter
  - `rag-api.yaml`: OpenAPI 3.0 spec for RAG endpoints (query, chapters, reindex)
- Wrote `quickstart.md` with guides for:
  - Content authors (adding chapters, code examples, diagrams)
  - Developers (project setup, React components, validation scripts)
  - CI/CD automation (workflows, deployment, testing)

**Architecture Highlights**:
- 3-level hierarchy: Module → Chapter → Section
- 12+ chapters across 5-6 modules
- Built-in Mermaid diagrams, Docusaurus versioning
- RAG system: OpenAI embeddings + Qdrant + Neon Postgres
- CI/CD: Link validation, code syntax checks, Docker-based ROS 2 testing
- Deployment: GitHub Pages with automatic build artifacts

**Constitution Check**: All 7 principles validated (Technical Accuracy, Educational Clarity, Reproducibility, Multi-Platform Support, RAG Accuracy, AI-Native Workflow, Citation Standards) - PASS ✅

## Outcome

- ✅ Impact: Established complete architecture for AI-native robotics textbook with RAG chatbot integration
- 🧪 Tests: Constitution Check passed; validation scripts designed for code/URDF/links
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, 3 contract files
- 🔁 Next prompts: `/sp.tasks` to generate task breakdown for Docusaurus setup + Module 1 chapters
- 🧠 Reflection: Comprehensive research phase with 4 parallel agents provided strong technical foundation; hybrid approaches (code inline/external, heading/semantic chunking) balance educational value with maintainability

## Evaluation notes (flywheel)

- Failure modes observed: None - planning workflow completed successfully
- Graders run and results (PASS/FAIL): PASS - All Phase 0 research questions resolved, Phase 1 artifacts generated with validation schemas
- Prompt variant (if applicable): Standard planning workflow with parallel research agents
- Next experiment (smallest change to try): Implement first validation script (Python code syntax) to test workflow before full /sp.tasks execution
