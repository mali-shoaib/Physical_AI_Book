---
description: "Task list for Docusaurus book architecture and Module 1 (ROS 2) implementation"
---

# Tasks: Docusaurus Book Architecture & Module 1 (ROS 2)

**Input**: Design documents from `/specs/001-ros2-basics/`
**Prerequisites**: plan.md (required), research.md, data-model.md, contracts/, quickstart.md

**Tests**: Tests are NOT explicitly requested in this feature. Focus on implementation and build validation.

**Organization**: Tasks are grouped by implementation phase to enable systematic setup, infrastructure, content creation, and deployment.

## Format: `[TaskID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: Root of repository
- **Content**: `docs/` directory
- **React components**: `src/components/`
- **Static assets**: `static/img/`, `static/code/`
- **Configuration**: `docusaurus.config.js`, `sidebars.js`, `package.json`
- **Validation scripts**: `scripts/` directory
- **CI/CD**: `.github/workflows/`

---

## Phase 1: Project Initialization

**Purpose**: Initialize Docusaurus project and configure basic infrastructure

- [X] T001 Initialize Docusaurus project with @docusaurus/preset-classic in repository root
- [X] T002 Configure docusaurus.config.js with project metadata (title, URL, baseUrl, theme)
- [X] T003 [P] Install additional dependencies (Mermaid theme, ideal-image plugin) in package.json
- [X] T004 [P] Create custom CSS file in src/css/custom.css for branding
- [X] T005 Create initial sidebars.js with manual configuration structure
- [X] T006 [P] Setup .gitignore for node_modules/, build/, .docusaurus/
- [X] T007 [P] Create README.md with project overview and setup instructions

---

## Phase 2: Directory Structure & Configuration

**Purpose**: Establish project structure and core configuration files

- [X] T008 Create docs/ directory structure (module-1-ros2/, module-2-simulation/, etc.)
- [X] T009 [P] Create static/ directory structure (img/, code/, files/)
- [X] T010 [P] Create src/components/ directory for custom React components
- [X] T011 [P] Create scripts/ directory for validation tooling
- [X] T012 Configure Mermaid diagram support in docusaurus.config.js (markdown.mermaid: true)
- [X] T013 [P] Configure Prism syntax highlighting for python, xml, bash, yaml in docusaurus.config.js
- [X] T014 Create package.json scripts (start, build, serve, validate-links, lint)
- [X] T015 [P] Create .env.example with environment variable templates (RAG_API_URL, etc.)

---

## Phase 3: Validation Scripts Setup

**Purpose**: Create automated validation tools for code examples, URDF files, and links

- [ ] T016 [P] Create scripts/validate_examples.py for Python code syntax validation (AST parsing)
- [ ] T017 [P] Create scripts/validate_urdf.py for URDF file validation (XML parsing)
- [ ] T018 [P] Create scripts/validate_imports.py to check Python import statements
- [ ] T019 [P] Create scripts/test_code_examples.py for extracting and testing code blocks
- [ ] T020 [P] Install validation dependencies (ruff, pylint) in package.json devDependencies
- [ ] T021 Create scripts/validate_all.py master script to run all validations

---

## Phase 4: CI/CD Workflows

**Purpose**: Setup GitHub Actions for automated deployment and validation

- [ ] T022 Create .github/workflows/deploy.yml for GitHub Pages deployment
- [ ] T023 [P] Create .github/workflows/validate.yml for link and code validation on PRs
- [ ] T024 [P] Create .github/workflows/test-examples.yml for Docker-based ROS 2 code testing
- [ ] T025 Configure GitHub Pages settings (gh-pages branch deployment)
- [ ] T026 [P] Create Dockerfile.test for ROS 2 Humble test container

---

## Phase 5: Module 1 - Chapter 1: ROS 2 Basics

**Purpose**: Write Chapter 1 covering ROS 2 nodes, topics, and services

- [ ] T027 Create docs/intro.md landing page with textbook overview
- [ ] T028 Create docs/module-1-ros2/ directory
- [ ] T029 Create docs/module-1-ros2/ch1-ros2-basics.md with frontmatter and structure
- [ ] T030 Write Ch1 Section 1: ROS 2 Architecture Overview (what is ROS 2, middleware, DDS)
- [ ] T031 [P] Create Mermaid diagram for ROS 2 architecture (nodes, topics, services) in Ch1
- [ ] T032 Write Ch1 Section 2: Nodes (definition, lifecycle, creating nodes with rclpy)
- [ ] T033 [P] Create static/code/module-1/minimal_node.py example (create_node, spin)
- [ ] T034 Write Ch1 Section 3: Topics (pub/sub pattern, message types, QoS)
- [ ] T035 [P] Create static/code/module-1/publisher_example.py (String publisher)
- [ ] T036 [P] Create static/code/module-1/subscriber_example.py (String subscriber)
- [ ] T037 [P] Create Mermaid diagram for pub/sub communication flow in Ch1
- [ ] T038 Write Ch1 Section 4: Services (request/response pattern, service definitions)
- [ ] T039 [P] Create static/code/module-1/service_server.py (AddTwoInts service)
- [ ] T040 [P] Create static/code/module-1/service_client.py (AddTwoInts client)
- [ ] T041 Write Ch1 Section 5: Summary and Next Steps
- [ ] T042 Add Ch1 to sidebars.js under Module 1 category
- [ ] T043 Validate all code examples in Ch1 with scripts/validate_examples.py
- [ ] T044 Build Docusaurus site and verify Ch1 renders correctly (npm run build)

**Checkpoint**: Chapter 1 should be fully functional with all code examples validated

---

## Phase 6: Module 1 - Chapter 2: rclpy Control

**Purpose**: Write Chapter 2 on using rclpy to control humanoid robots

- [ ] T045 Create docs/module-1-ros2/ch2-rclpy-control.md with frontmatter
- [ ] T046 Write Ch2 Section 1: Introduction to rclpy for Robot Control
- [ ] T047 Write Ch2 Section 2: Publishing Joint Commands (JointState, JointTrajectory messages)
- [ ] T048 [P] Create static/code/module-1/joint_state_publisher.py example
- [ ] T049 [P] Create static/code/module-1/joint_trajectory_publisher.py example
- [ ] T050 [P] Create Mermaid diagram for joint control message flow
- [ ] T051 Write Ch2 Section 3: Subscribing to Sensor Feedback (joint states, IMU data)
- [ ] T052 [P] Create static/code/module-1/sensor_subscriber.py example
- [ ] T053 Write Ch2 Section 4: Creating a Simple Control Loop (read sensors → compute → command)
- [ ] T054 [P] Create static/code/module-1/simple_control_loop.py example
- [ ] T055 Write Ch2 Section 5: Error Handling and Safety Considerations
- [ ] T056 Write Ch2 Section 6: Summary and Practical Exercises
- [ ] T057 Add Ch2 to sidebars.js under Module 1 category
- [ ] T058 Validate all code examples in Ch2 with scripts/validate_examples.py
- [ ] T059 Build Docusaurus site and verify Ch2 renders correctly

**Checkpoint**: Chapter 2 should be complete with validated control examples

---

## Phase 7: Module 1 - Chapter 3: URDF for Humanoids

**Purpose**: Write Chapter 3 on defining humanoid robot structure with URDF

- [ ] T060 Create docs/module-1-ros2/ch3-urdf-humanoids.md with frontmatter
- [ ] T061 Write Ch3 Section 1: Introduction to URDF (Unified Robot Description Format)
- [ ] T062 Write Ch3 Section 2: Links (visual geometry, collision geometry, inertia)
- [ ] T063 [P] Create static/code/module-1/simple_link.urdf example (single link)
- [ ] T064 Write Ch3 Section 3: Joints (revolute, prismatic, fixed, joint limits)
- [ ] T065 [P] Create static/code/module-1/simple_joint.urdf example (link + joint)
- [ ] T066 Write Ch3 Section 4: Building a Humanoid Structure (torso, arms, legs, head)
- [ ] T067 [P] Create static/code/module-1/humanoid_upper_body.urdf (torso + arms)
- [ ] T068 [P] Create static/code/module-1/humanoid_full.urdf (complete simplified humanoid)
- [ ] T069 [P] Create Mermaid diagram showing link/joint hierarchy for humanoid
- [ ] T070 Write Ch3 Section 5: Visualizing URDF in RViz2
- [ ] T071 Write Ch3 Section 6: Common URDF Pitfalls and Debugging
- [ ] T072 Write Ch3 Section 7: Summary and Resources
- [ ] T073 Add Ch3 to sidebars.js under Module 1 category
- [ ] T074 Validate all URDF files with scripts/validate_urdf.py
- [ ] T075 Build Docusaurus site and verify Ch3 renders correctly

**Checkpoint**: Module 1 (all 3 chapters) should be complete and validated

---

## Phase 8: RAG Chatbot - Backend Setup

**Purpose**: Implement FastAPI backend for RAG query processing

- [ ] T076 Create rag-backend/ directory in repository root
- [ ] T077 Create rag-backend/requirements.txt (fastapi, uvicorn, openai, qdrant-client, pydantic)
- [ ] T078 Create rag-backend/main.py with FastAPI app initialization
- [ ] T079 [P] Implement POST /api/rag/query endpoint in rag-backend/main.py
- [ ] T080 [P] Implement GET /api/chapters endpoint in rag-backend/main.py
- [ ] T081 [P] Implement POST /api/rag/reindex endpoint (admin) in rag-backend/main.py
- [ ] T082 Configure CORS middleware for Docusaurus frontend origins
- [ ] T083 [P] Create rag-backend/chunking.py for text chunking logic (heading-based + semantic)
- [ ] T084 [P] Create rag-backend/embedding.py for OpenAI embedding generation
- [ ] T085 [P] Create rag-backend/retrieval.py for Qdrant vector search
- [ ] T086 [P] Create rag-backend/schema.py with Pydantic models (RAGQueryRequest, RAGQueryResponse)
- [ ] T087 Create rag-backend/.env.example with API keys template
- [ ] T088 Create rag-backend/README.md with setup and deployment instructions

---

## Phase 9: RAG Chatbot - Frontend Integration

**Purpose**: Integrate RAG chatbot widget into Docusaurus UI

- [ ] T089 Create src/components/ChatbotWidget.tsx React component
- [ ] T090 [P] Create src/components/ChatbotWidget.module.css for styling
- [ ] T091 Implement chatbot UI (message list, input form, toggle button) in ChatbotWidget.tsx
- [ ] T092 Implement API call to RAG backend (/api/rag/query) in ChatbotWidget.tsx
- [ ] T093 Add loading state and error handling in ChatbotWidget.tsx
- [ ] T094 [P] Implement source citation display (links to book sections) in ChatbotWidget.tsx
- [ ] T095 Swizzle Layout component (npm run swizzle @docusaurus/theme-classic Layout)
- [ ] T096 Inject ChatbotWidget into swizzled Layout component
- [ ] T097 Configure RAG_API_URL environment variable in .env.production
- [ ] T098 Test chatbot widget locally with running FastAPI backend

---

## Phase 10: RAG Chatbot - Content Indexing

**Purpose**: Build system to extract, chunk, and embed Docusaurus content

- [ ] T099 Create plugins/docusaurus-rag-plugin/ directory
- [ ] T100 Create plugins/docusaurus-rag-plugin/index.js with plugin structure
- [ ] T101 Implement contentLoaded hook to extract markdown files
- [ ] T102 [P] Implement markdown parsing (frontmatter + body extraction)
- [ ] T103 [P] Implement chunking logic (H2 boundaries, 500-1000 tokens)
- [ ] T104 [P] Implement embedding generation (batch API calls to OpenAI)
- [ ] T105 [P] Implement caching (content hash → skip re-embedding)
- [ ] T106 Implement serialization to build/rag-index.json
- [ ] T107 Add plugin to docusaurus.config.js plugins array
- [ ] T108 Test plugin by running build and verifying rag-index.json output

---

## Phase 11: Documentation & Polish

**Purpose**: Create supporting documentation and improve UX

- [ ] T109 [P] Update README.md with complete setup instructions
- [ ] T110 [P] Create CONTRIBUTING.md with contribution guidelines
- [ ] T111 [P] Create docs/appendix/glossary.md with ROS 2 terminology
- [ ] T112 [P] Create docs/appendix/resources.md with external links (ROS 2 docs, papers)
- [ ] T113 Add glossary and resources to sidebars.js under Appendix category
- [ ] T114 [P] Optimize images (compress PNG/JPG, convert to WebP where appropriate)
- [ ] T115 [P] Add meta tags for SEO (Open Graph, Twitter Card) in docusaurus.config.js
- [ ] T116 Create custom homepage in src/pages/index.tsx (replace default)
- [ ] T117 [P] Add dark mode toggle testing
- [ ] T118 Run full validation suite (scripts/validate_all.py)

---

## Phase 12: Testing & Deployment

**Purpose**: Final testing and production deployment

- [ ] T119 Run npm run build -- --error-on-broken-links (validate all internal links)
- [ ] T120 Run npx linkinator ./build --recurse (validate external links)
- [ ] T121 [P] Test all code examples in Docker container (ROS 2 Humble)
- [ ] T122 [P] Test RAG chatbot with known-good queries (verify answers from book content)
- [ ] T123 [P] Test RAG chatbot with out-of-scope queries (verify refusal to hallucinate)
- [ ] T124 Run Lighthouse audit for performance, accessibility, SEO
- [ ] T125 Deploy to GitHub Pages (push to main triggers deploy.yml workflow)
- [ ] T126 Verify deployed site loads correctly (check URL, navigation, chatbot)
- [ ] T127 Monitor build logs and fix any deployment issues

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1** (Project Initialization): No dependencies - start immediately
- **Phase 2** (Directory Structure): Depends on Phase 1 completion
- **Phase 3** (Validation Scripts): Depends on Phase 2 completion
- **Phase 4** (CI/CD): Depends on Phase 2 completion (can run parallel with Phase 3)
- **Phase 5-7** (Module 1 Chapters): Depend on Phase 1-2 completion (chapters can be written in parallel)
- **Phase 8** (RAG Backend): Depends on Phase 2 completion (can run parallel with chapters)
- **Phase 9-10** (RAG Frontend/Indexing): Depends on Phase 8 completion
- **Phase 11** (Polish): Depends on Phase 5-7 completion (chapters must exist)
- **Phase 12** (Testing/Deployment): Depends on all previous phases

### Critical Path

1. Phase 1 → Phase 2 → Phase 5 → Phase 6 → Phase 7 → Phase 11 → Phase 12
2. Parallel stream: Phase 2 → Phase 8 → Phase 9 → Phase 10 → Phase 12

### Parallel Opportunities

**During Phase 3-4** (after Phase 2):
- Validation scripts (T016-T021) can be written in parallel
- CI/CD workflows (T022-T026) can be created in parallel with scripts

**During Phase 5-7** (after Phase 2):
- All three chapters can be drafted simultaneously by different authors
- Code examples within each chapter (marked [P]) can be written in parallel
- Diagrams within each chapter (marked [P]) can be created in parallel

**During Phase 8** (Backend setup):
- All backend modules (T083-T086) can be written in parallel after main.py exists

**During Phase 9** (Frontend integration):
- ChatbotWidget UI and styling (T089-T090) can be developed in parallel

**During Phase 11** (Polish):
- Documentation files (T109-T113) can be written in parallel
- Image optimization (T114) can run parallel with documentation

**During Phase 12** (Testing):
- Code testing (T121), chatbot testing (T122-T123), and Lighthouse audit (T124) can run in parallel

---

## Implementation Strategy

### MVP First (Phases 1-7 Only)

1. Complete Phase 1: Project Initialization
2. Complete Phase 2: Directory Structure
3. Complete Phase 3: Validation Scripts (enables quality gates)
4. Complete Phase 4: CI/CD (enables automated testing)
5. Complete Phase 5: Chapter 1
6. Complete Phase 6: Chapter 2
7. Complete Phase 7: Chapter 3
8. **STOP and VALIDATE**: Test Module 1 independently (npm run build)
9. Deploy/demo if ready

**MVP Delivers**: Fully functional Docusaurus book with Module 1 (ROS 2) - no chatbot yet

### Incremental Delivery

1. Complete Phases 1-7 → Deploy Module 1 (MVP)
2. Add Phases 8-10 → Deploy with RAG chatbot
3. Add Phase 11 → Deploy polished version
4. Phase 12 → Production deployment with full testing

### Parallel Team Strategy

With multiple developers:

1. **Team completes Phases 1-4 together** (foundation)
2. Once foundation is done:
   - **Developer A**: Chapter 1 (T027-T044)
   - **Developer B**: Chapter 2 (T045-T059)
   - **Developer C**: Chapter 3 (T060-T075)
   - **Developer D**: RAG Backend (T076-T088)
3. After chapters complete:
   - **Developer A+B**: RAG Frontend (T089-T098)
   - **Developer C+D**: RAG Indexing Plugin (T099-T108)
4. **All developers**: Polish & Testing (T109-T127)

---

## Task Summary

**Total Tasks**: 127
**Parallel Tasks**: 58 (45.6%)
**Critical Path Length**: ~40 sequential tasks

**Tasks by Phase**:
- Phase 1 (Initialization): 7 tasks
- Phase 2 (Structure): 8 tasks
- Phase 3 (Validation): 6 tasks
- Phase 4 (CI/CD): 5 tasks
- Phase 5 (Chapter 1): 18 tasks
- Phase 6 (Chapter 2): 15 tasks
- Phase 7 (Chapter 3): 16 tasks
- Phase 8 (RAG Backend): 13 tasks
- Phase 9 (RAG Frontend): 10 tasks
- Phase 10 (RAG Indexing): 10 tasks
- Phase 11 (Polish): 10 tasks
- Phase 12 (Testing/Deployment): 9 tasks

**Estimated Timeline** (single developer, sequential):
- Phases 1-2: 1-2 days
- Phases 3-4: 1-2 days
- Phases 5-7: 5-7 days (content writing is time-intensive)
- Phases 8-10: 3-4 days
- Phases 11-12: 1-2 days
- **Total**: 11-17 days

**With parallel execution** (3 developers):
- Foundation (Phases 1-4): 2-3 days
- Content + Backend (Phases 5-8): 4-5 days
- Integration (Phases 9-10): 2-3 days
- Polish + Deploy (Phases 11-12): 1-2 days
- **Total**: 9-13 days

---

## Notes

- All [P] tasks can run in parallel within their phase
- File paths are explicit for each task
- Validation tasks are integrated throughout (not separate test phase)
- Chapters are independently completable (MVP = Chapter 1 only if needed)
- RAG system is decoupled from content (can be added later)
- Commit after completing each chapter or logical group of tasks
- Stop at any checkpoint to validate independently before proceeding
