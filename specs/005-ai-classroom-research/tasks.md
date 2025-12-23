# Tasks: AI Classroom Efficiency Research Paper

**Input**: Design documents from `/specs/005-ai-classroom-research/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md, contracts/

**Tests**: No test tasks included - this is a research paper (written document), not software requiring unit/integration tests.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Deliverable**: `docs/research-papers/ai-classroom-efficiency-k12.md`
- **Supporting data**: `specs/005-ai-classroom-research/data/` (evidence collection)
- **Contracts**: `specs/005-ai-classroom-research/contracts/` (JSON schemas for structure)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and directory structure for research paper

- [x] T001 Create `docs/research-papers/` directory if it doesn't exist
- [x] T002 Create `specs/005-ai-classroom-research/data/` directory for evidence collection
- [x] T003 [P] Create evidence collection template files (3 JSON files following contracts/application-evidence.schema.json for automated-grading, intelligent-tutoring, ai-writing-feedback)
- [x] T004 [P] Create ROI calculation template files (3 JSON files following contracts/roi-calculation.schema.json, one per application)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Literature search and evidence gathering that MUST be complete before ANY writing can begin

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Run ERIC database search with Boolean queries from research.md Decision 7 (automated grading: `("automated grading" OR "automated assessment") AND K-12 AND ("teacher workload" OR "time savings") AND (2015:2025)`)
- [x] T006 [P] Run Google Scholar search with Boolean queries for automated grading systems
- [x] T007 [P] Run IEEE Xplore search with Boolean queries for intelligent tutoring systems (`("intelligent tutoring" OR ITS) AND K-12 AND ("learning outcomes" OR "student achievement") AND (2015:2025)`)
- [x] T008 [P] Run JSTOR search with Boolean queries for AI writing/feedback tools (`("writing feedback" OR "automated writing evaluation" OR AWE) AND K-12 AND (2015:2025)`)
- [x] T009 Screen abstracts from all searches against inclusion criteria (peer-reviewed, K-12, empirical data, 2015-2025) → create shortlist of 30-40 studies
- [x] T010 Full-text review of shortlisted studies → select final 15+ studies meeting quality standards
- [x] T011 Extract data from selected studies into evidence collection JSON files (populate contracts/application-evidence.schema.json fields: citation_apa, study_type, sample_size, context, key_findings)
- [x] T012 [P] Extract teacher workload metrics from evidence (hours saved, percentage reduction) → populate workload_metric objects in JSON files
- [x] T013 [P] Extract student outcome metrics from evidence (test scores, engagement, effect sizes) → populate outcome_metric objects in JSON files
- [x] T014 Research cost data for each application (vendor pricing, published implementation studies) → populate Cost Component fields in ROI JSON files
- [x] T015 Calculate ROI for each application using formula from research.md Decision 5: ROI = (Total Benefits - Total Costs) / Total Costs × 100, break-even = Total Costs / (Annual Benefits / 12)
- [x] T016 Validate all extracted data against JSON schemas (run JSON schema validators on all data files to ensure compliance)

**Checkpoint**: Foundation ready - all evidence collected, structured, and validated. Writing can now begin.

---

## Phase 3: User Story 1 - Evidence-Based Application Overview (Priority: P1) 🎯 MVP

**Goal**: Create paper sections presenting 3 AI applications with peer-reviewed evidence for teacher workload and student outcomes

**Independent Test**: Education administrator can read Sections 3-5 (AI Applications) and verify: (1) at least 3 distinct AI applications presented, (2) each cites peer-reviewed research, (3) each explains workload AND outcome impact with specific metrics

### Implementation for User Story 1

- [x] T017 [P] [US1] Create paper file structure: Initialize `docs/research-papers/ai-classroom-efficiency-k12.md` with APA-formatted header, title, and section stubs
- [x] T018 [P] [US1] Write Introduction section (400 words): Context on AI in K-12, purpose of paper, significance for administrators, preview of 3 applications
- [x] T019 [US1] Write Section 3 - Automated Grading & Assessment (600 words): Description, evidence from 2+ studies, quantified workload metric (hours/week saved), quantified outcome metric (test score impact), APA in-text citations
- [x] T020 [US1] Write Section 4 - Intelligent Tutoring Systems (600 words): Description, evidence from 2+ studies, quantified workload metric (lesson planning time), quantified outcome metric (learning gains with effect size), APA in-text citations
- [x] T021 [US1] Write Section 5 - AI Writing/Feedback Tools (600 words): Description, evidence from 2+ studies, quantified workload metric (feedback time saved), quantified outcome metric (writing quality improvement), APA in-text citations
- [x] T022 [US1] Create comparison table using contracts/comparison-table.schema.json: Populate markdown table with 3 applications × 5 metrics (workload impact, student outcomes, year 1 cost, ROI %, break-even), embed after Section 5
- [x] T023 [US1] Add equity considerations paragraph to each application section (FR-012): Document differential impact on under-resourced schools or diverse learners based on evidence
- [x] T024 [US1] Validate Section 2-5 word count (target: 400 intro + 1800 applications = 2200 words) and readability (run Flesch-Kincaid, target grade 10-12)

**Checkpoint**: At this point, User Story 1 (core evidence-based applications) should be complete and independently readable. Administrator can understand what AI applications exist and their impact.

---

## Phase 4: User Story 2 - ROI Justification Framework (Priority: P2)

**Goal**: Add ROI analysis section enabling administrators to build budget proposals with cost-benefit calculations

**Independent Test**: Finance-conscious administrator can review Section 6 (ROI Analysis) and create budget proposal with: (1) cost estimates per application, (2) quantified dollar-value benefits, (3) break-even timeline

### Implementation for User Story 2

- [x] T025 [US2] Write Section 6 - ROI Analysis Framework (500 words): Explain TCO methodology, present cost breakdown for each application (licensing, implementation, training, maintenance per FR-006), explain benefit calculation (time savings × teacher hourly rate)
- [x] T026 [P] [US2] Create detailed ROI example for one application (automated grading recommended): Show step-by-step calculation with assumptions (500-student school, 25 teachers, $40/hour rate), compute Year 1 ROI and break-even months
- [x] T027 [P] [US2] Add sensitivity analysis subsection: Present best-case and worst-case ROI scenarios (e.g., 100% vs 50% teacher adoption), document how ROI varies by school context (urban vs rural, elementary vs secondary)
- [x] T028 [US2] Cross-reference ROI section to comparison table from US1: Add callout directing readers to table for side-by-side ROI comparison across all 3 applications
- [x] T029 [US2] Validate ROI calculations against data-model.md validation rules: Ensure all 4 cost categories present, break-even timeline calculated, assumptions documented
- [x] T030 [US2] Validate Section 6 word count (target: 500 words) and ensure accessibility (define TCO and other financial terms for non-technical audience per FR-008)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Administrator has evidence (US1) and can now justify budget (US2).

---

## Phase 5: User Story 3 - Implementation Guidance (Priority: P3)

**Goal**: Add implementation section addressing deployment prerequisites, training needs, and common pitfalls

**Independent Test**: Administrator who decided to adopt AI can review Section 7 (Implementation) and identify: (1) prerequisite requirements, (2) teacher training needs, (3) deployment challenges and mitigation strategies

### Implementation for User Story 3

- [x] T031 [US3] Write Section 7 - Implementation Considerations (400 words): Document prerequisites (reliable internet, devices, IT support capacity), training duration and content recommendations (e.g., 4-hour workshop + follow-ups), common deployment challenges from research evidence
- [x] T032 [P] [US3] Create implementation checklist subsection: Bulleted list or table with 8-10 prerequisite items (infrastructure, staffing, budget, timeline) that administrators can use for planning
- [x] T033 [P] [US3] Add change management strategies paragraph: Evidence-based approaches to building teacher buy-in (based on studies documenting successful vs failed implementations)
- [x] T034 [US3] Document common pitfalls with mitigation strategies: Extract from research evidence (e.g., "inadequate training leads to 60% abandonment - mitigate with 2 follow-up sessions")
- [x] T035 [US3] Validate Section 7 word count (target: 400 words) and cross-reference to application sections (link implementation challenges to specific applications where relevant)

**Checkpoint**: All 3 user stories complete. Paper now provides evidence (US1), ROI justification (US2), and implementation guidance (US3).

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize paper with executive summary, limitations, conclusion, references, and quality validation

- [x] T036 [P] Write Section 1 - Executive Summary (300 words): Synthesize key findings from all 3 applications, highlight top ROI insights (from US2), provide 3-5 actionable recommendations, ensure readable in \<10 minutes per SC-006
- [x] T037 [P] Write Section 8 - Limitations & Future Directions (300 words): Disclose research limitations per FR-013 (sample sizes, study contexts, generalizability constraints), address edge cases from spec.md (conflicting findings, research gaps, emerging technologies)
- [x] T038 [P] Write Section 9 - Conclusion (200 words): Summarize evidence-based impact of 3 applications, reiterate ROI potential, close with call to action for administrators
- [x] T039 Create Section 10 - References (APA 7th edition): Compile all in-text citations into alphabetical reference list, ensure 15+ peer-reviewed sources per FR-015, validate APA format (author-date, DOI/URL included, hanging indent simulated)
- [x] T040 [P] Validate total word count: Run `wc -w` on markdown file, confirm 3000-5000 range (target 3500: 300 + 400 + 1800 + 500 + 400 + 300 + 200 = 3900 words + references), trim if over 5000
- [x] T041 [P] Run readability analysis: Use Readability Formulas or Hemingway Editor, confirm Flesch-Kincaid grade level 10-12 per SC-005, simplify language if exceeds grade 12
- [x] T042 [P] APA compliance check: Manual review or automated tool (ProWritingAid APA mode), validate heading levels (# for Level 1, ## for Level 2, ### for Level 3), in-text citation format, reference list entries
- [x] T043 Validate all functional requirements met: Cross-check against FR-001 to FR-015 checklist (3+ applications, peer-reviewed citations, quantified metrics, ROI analysis, accessible language, APA format, executive summary, equity considerations, limitations, comparison table, 15+ sources)
- [x] T044 Validate all success criteria met: Cross-check against SC-001 to SC-010 (3 applications with 2+ citations each, workload/outcome metrics quantified, ROI with break-even, readability 10-12, executive summary \<10 min, comparison table, 15+ sources 2015-2025, implementation guidance, equity addressed)
- [x] T045 [P] Create markdown rendering test: Preview paper in Markdown viewer (VS Code, Typora), ensure tables render correctly, headings display properly, citations format consistently
- [x] T046 Final proofreading pass: Check for typos, grammatical errors, consistency in terminology (e.g., "K-12" vs "K12"), ensure all acronyms defined on first use

**Checkpoint**: Paper complete and validated. Ready for distribution to education administrators.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately (4 tasks, ~1 hour)
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories (12 tasks, ~3-4 days for literature search and evidence synthesis)
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed sequentially in priority order (P1 → P2 → P3)
  - Or in parallel if multiple writers available (different sections)
- **Polish (Phase 6)**: Depends on all 3 user stories being complete (11 tasks, ~2 days)

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories (MVP = evidence-based applications)
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1 but references comparison table created in US1 (cross-reference added in T028)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2 but may cross-reference applications

### Within Each User Story

- **US1 (Evidence)**: Introduction → Application sections (can write in parallel) → Comparison table (requires all apps done) → Validation
- **US2 (ROI)**: ROI methodology → Detailed example (parallel) + Sensitivity analysis (parallel) → Cross-reference → Validation
- **US3 (Implementation)**: Main section → Checklist (parallel) + Change management (parallel) → Pitfalls → Validation

### Parallel Opportunities

**Phase 1 (Setup)**:
- T003 and T004 can run in parallel (different template files)

**Phase 2 (Foundational)**:
- T006, T007, T008 can run in parallel (different database searches)
- T012 and T013 can run in parallel (different metric extractions)

**Phase 3 (US1)**:
- T017 and T018 can run in parallel (file structure + intro, different content)
- T019, T020, T021 CANNOT run in parallel (requires sequential evidence review per application)
- T024 runs after T019-T023 complete

**Phase 4 (US2)**:
- T026 and T027 can run in parallel (detailed example + sensitivity analysis, different subsections)

**Phase 5 (US3)**:
- T032 and T033 can run in parallel (checklist + change management, different subsections)

**Phase 6 (Polish)**:
- T036, T037, T038 can run in parallel (executive summary, limitations, conclusion - different sections)
- T040, T041, T042 can run in parallel (word count, readability, APA checks - different validation tools)
- T045 runs after T039-T042 complete

---

## Parallel Example: Phase 2 (Foundational)

```bash
# Launch database searches in parallel (different databases):
Task T006: "Google Scholar search for automated grading"
Task T007: "IEEE Xplore search for intelligent tutoring systems"
Task T008: "JSTOR search for AI writing/feedback tools"

# After evidence collected, extract metrics in parallel:
Task T012: "Extract teacher workload metrics"
Task T013: "Extract student outcome metrics"
```

---

## Parallel Example: User Story 1 (Evidence)

```bash
# These can start together (different content areas):
Task T017: "Initialize paper file structure"
Task T018: "Write Introduction section"

# Application sections written sequentially (each requires evidence review):
Task T019: "Write Automated Grading section" → then
Task T020: "Write Intelligent Tutoring Systems section" → then
Task T021: "Write AI Writing/Feedback Tools section"
```

---

## Parallel Example: Phase 6 (Polish)

```bash
# Launch writing tasks in parallel (different sections):
Task T036: "Write Executive Summary"
Task T037: "Write Limitations & Future Directions"
Task T038: "Write Conclusion"

# After writing complete, launch validation tasks in parallel:
Task T040: "Validate word count"
Task T041: "Run readability analysis"
Task T042: "APA compliance check"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (4 tasks, ~1 hour)
2. Complete Phase 2: Foundational (12 tasks, ~3-4 days for literature search)
3. Complete Phase 3: User Story 1 (8 tasks, ~2-3 days for writing application sections)
4. **STOP and VALIDATE**: Have education administrator read Sections 2-5, verify 3 applications with evidence and metrics
5. If validated, proceed to US2 (ROI) or stop with MVP (evidence-only version)

**MVP Deliverable**: Sections 1-5 (Executive Summary + Introduction + 3 AI Applications + Comparison Table) = ~2500 words with 15+ peer-reviewed sources. Administrator can understand what AI applications work and their impact, but lacks ROI justification and implementation guidance.

### Incremental Delivery

1. Complete Setup + Foundational (Phases 1-2) → Foundation ready
2. Add User Story 1 (Phase 3) → Test independently → **Delivery 1: Evidence-based applications MVP**
3. Add User Story 2 (Phase 4) → Test independently → **Delivery 2: Evidence + ROI justification**
4. Add User Story 3 (Phase 5) → Test independently → **Delivery 3: Full paper with implementation guidance**
5. Polish (Phase 6) → **Final Delivery: Publication-ready research paper**

### Sequential Writing Strategy (Recommended)

**Timeline**: 2 weeks (14 days)

- **Days 1-4**: Phases 1-2 (Setup + Foundational = literature search and evidence synthesis)
- **Days 5-7**: Phase 3 (User Story 1 = write application sections)
- **Days 8-9**: Phase 4 (User Story 2 = write ROI section)
- **Days 10-11**: Phase 5 (User Story 3 = write implementation section)
- **Days 12-14**: Phase 6 (Polish = executive summary, limitations, conclusion, validation)

**Effort**: 40-50 hours total (3-4 hours/day)

---

## Task Summary

**Total Tasks**: 46

**By Phase**:
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 12 tasks
- Phase 3 (US1 - Evidence): 8 tasks
- Phase 4 (US2 - ROI): 6 tasks
- Phase 5 (US3 - Implementation): 5 tasks
- Phase 6 (Polish): 11 tasks

**By User Story**:
- User Story 1 (Evidence-Based Applications): 8 tasks (T017-T024)
- User Story 2 (ROI Justification): 6 tasks (T025-T030)
- User Story 3 (Implementation Guidance): 5 tasks (T031-T035)

**Parallel Tasks**: 15 tasks marked [P] (33% of total)

**Independent Test Criteria**:
- **US1**: Administrator can identify 3 applications with workload/outcome metrics and peer-reviewed citations
- **US2**: Administrator can create budget proposal with cost estimates, benefits, and break-even timeline
- **US3**: Administrator can identify prerequisites, training needs, and deployment challenges

**MVP Scope**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1) = 24 tasks → Evidence-based applications paper

---

## Notes

- [P] tasks = different files/sections, no dependencies within phase
- [Story] label maps task to specific user story for traceability
- Each user story delivers independent value (can stop after any phase)
- No test tasks included - research paper validated by administrator review, not automated tests
- Commit after each completed section (e.g., after each application section)
- Stop at any checkpoint to validate independently
- Avoid: vague tasks, redundant evidence collection, unsupported claims without citations
- All file paths are absolute from repository root
- JSON schemas provide structure for evidence collection (contracts/ directory)
- Follow quickstart.md methodology for detailed workflow guidance
