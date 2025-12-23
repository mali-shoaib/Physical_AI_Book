# Quickstart: Research Methodology Summary

**Feature**: 005-ai-classroom-research
**Date**: 2025-12-22
**Purpose**: Concise overview of research approach for writing AI classroom efficiency paper

## Research Paper Overview

**Title**: AI in the K-12 Classroom: Evidence-Based Impact on Teacher Workload and Student Outcomes

**Target Audience**: Education administrators (principals, superintendents, curriculum directors)

**Word Count**: 3000-5000 words (target: 3500 words)

**Timeline**: 2 weeks from literature search to final draft

**Deliverable**: Markdown document (`docs/research-papers/ai-classroom-efficiency-k12.md`) with APA 7th edition citations

---

## Methodology Workflow

### Phase 0: Literature Search (Days 1-3)

**Objective**: Identify 15+ peer-reviewed sources from 2015-2025

**Databases**:
- ERIC (Education Resources Information Center)
- Google Scholar
- IEEE Xplore
- JSTOR

**Search Strategy**:
1. Run Boolean queries for each AI application (see research.md Decision 7)
2. Screen abstracts for inclusion criteria (peer-reviewed, K-12, empirical data, 2015-2025)
3. Full-text review of included studies
4. Extract data into structured format (contracts/application-evidence.schema.json)

**Deliverable**: Populated Research Evidence entities (15+ studies)

---

### Phase 1: Evidence Synthesis (Days 4-7)

**Objective**: Organize findings into structured data models

**Tasks**:
1. **Application Summaries**: Create AI Classroom Application entity for each of 3 applications
   - Automated Grading/Assessment
   - Intelligent Tutoring Systems
   - AI Writing/Feedback Tools
2. **Workload Metrics**: Calculate time savings from evidence (hours/week, % reduction)
3. **Outcome Metrics**: Synthesize student impact data (test scores, engagement, effect sizes)
4. **Cost Research**: Gather pricing data from vendor websites or published case studies
5. **ROI Calculations**: Apply formula (Benefits - Costs) / Costs × 100; compute break-even

**Deliverables**:
- 3 AI Classroom Application records with linked evidence
- Teacher Workload Metrics (1 per application)
- Student Outcome Metrics (1 per application)
- Cost Components (4 categories × 3 applications)
- ROI Calculations (1+ per application)
- Comparison Table (contracts/comparison-table.schema.json)

---

### Phase 2: Writing (Days 8-12)

**Objective**: Draft 3500-word paper following APA 7th edition style

**Section Allocation** (word count budget):

| Section | Words | Content Focus |
|---------|-------|---------------|
| Executive Summary | 300 | Key findings, ROI highlights, recommendations (\<10 min read) |
| Introduction | 400 | Context, purpose, significance of AI in K-12 |
| AI Application 1 | 600 | Automated grading evidence, metrics, examples |
| AI Application 2 | 600 | ITS evidence, metrics, examples |
| AI Application 3 | 600 | AI writing tools evidence, metrics, examples |
| ROI Analysis | 500 | Methodology, example calculations, sensitivity analysis |
| Implementation | 400 | Prerequisites, training, common pitfalls |
| Limitations | 300 | Research gaps, caveats, generalizability constraints |
| Conclusion | 200 | Summary, actionable recommendations |
| References | N/A | 15+ APA 7th edition citations |
| **TOTAL** | **3500** | |

**Embedded Tables/Figures**:
1. **Comparison Table** (after Application sections): Side-by-side metrics for all 3 applications
2. **ROI Example** (in ROI section): Detailed calculation for one application
3. **Implementation Checklist** (in Implementation section): Prerequisites and steps

**Writing Guidelines**:
- **Readability**: Target Flesch-Kincaid grade level 10-12 (accessible to non-technical audience)
- **APA Format**: Author-date in-text citations, alphabetical reference list, Level 1-3 headings
- **Jargon**: Define technical terms on first use (e.g., "RCT (randomized controlled trial)")
- **Tone**: Objective, evidence-based, avoid promotional language

---

### Phase 3: Review & Finalization (Days 13-14)

**Quality Checks**:
1. **Word Count**: Verify 3000-5000 range
2. **Readability**: Run Flesch-Kincaid analysis (tools: readabilityformulas.com, Hemingway App)
3. **APA Compliance**: Check citations against APA manual or automated checker (ProWritingAid)
4. **Evidence Backing**: Ensure all claims cite peer-reviewed sources (no unsupported assertions)
5. **Success Criteria**: Validate against spec.md SC-001 to SC-010
6. **Functional Requirements**: Confirm FR-001 to FR-015 met

**Revision Priorities**:
- Cut redundant content if over 5000 words
- Add evidence citations if any claims lack support
- Simplify language if Flesch-Kincaid exceeds grade 12
- Correct APA formatting errors

**Deliverable**: Final Markdown document ready for distribution to administrators

---

## Key Constraints & Assumptions

**Constraints** (from user input and spec.md):
- 2-week timeline (14 days)
- 3000-5000 word limit
- Markdown format (APA styling adapted)
- Minimum 8 peer-reviewed sources (user input), 15+ sources (spec.md FR-015) - use higher standard
- APA 7th edition mandatory

**Assumptions** (from spec.md):
- Target readers: Education administrators with budget authority
- Readers prioritize ROI justification and evidence-based decisions
- Teacher hourly rate: $35-50 (national averages)
- School context: 500-student schools with 25 teachers (medium size)
- Deployment: Reliable internet, 1:1 or shared device models available

---

## Data Collection Structures

**Use JSON schemas** (`contracts/*.schema.json`) to organize evidence during literature search:

1. **application-evidence.schema.json**: Document each AI application with linked research evidence
   - Minimum 2 evidence records per application
   - Include workload_metric and outcome_metric objects

2. **roi-calculation.schema.json**: Structure cost-benefit analysis
   - Costs: Software, implementation, training, maintenance
   - Benefits: Teacher time savings × hourly rate
   - Output: ROI %, break-even timeline

3. **comparison-table.schema.json**: Generate side-by-side comparison for paper
   - 3 applications × 5 metrics (workload, outcomes, cost, ROI, break-even)
   - Include markdown_format field for easy embedding

**Data Flow**: Literature search → JSON schemas → Markdown sections → Final paper

---

## Validation Before Writing

**Pre-Writing Checklist** (from data-model.md):
- [ ] 3+ AI Classroom Application entities defined
- [ ] 15+ Research Evidence entities documented (APA format)
- [ ] Each application has 2+ linked peer-reviewed evidence records
- [ ] Each application has quantified Teacher Workload Metric (hours/week or %)
- [ ] Each application has quantified Student Outcome Metric (test scores, engagement, etc.)
- [ ] Each application has 4 Cost Component categories (licensing, implementation, training, maintenance)
- [ ] Each application has at least 1 ROI Calculation with break-even timeline
- [ ] Comparison Table populated with all metrics
- [ ] All data sources cited (no unsupported claims)

**If any item unchecked**: Return to literature search or cost research to fill gaps before drafting paper.

---

## Markdown to APA Formatting Notes

**APA requires specific formatting** (designed for Word/LaTeX), adapted for Markdown:

| APA Element | Markdown Equivalent | Notes |
|-------------|---------------------|-------|
| Headings | `#` (Level 1), `##` (Level 2), `###` (Level 3) | Use bold for emphasis where needed |
| Italics | `*text*` or `_text_` | For book titles, emphasis |
| Bold | `**text**` | Not standard in APA but acceptable in Markdown |
| In-text citation | `(Author, Year)` or `Author (Year)` | Plain text, no special syntax |
| Block quote | `> Quoted text` | Use for quotes \>40 words |
| Table | Markdown table syntax | APA prefers numbered tables; approximate with captions |
| Hanging indent | Not native to Markdown | Acknowledge limitation; acceptable for digital distribution |
| Page numbers | N/A | Digital-only deliverable, no pagination |

**Limitation Disclosure**: Markdown does not perfectly replicate APA print format (no hanging indents, page numbers). For print distribution, convert to Word using Pandoc or similar tool.

---

## Tools & Resources

**Literature Search**:
- ERIC: https://eric.ed.gov
- Google Scholar: https://scholar.google.com
- IEEE Xplore: https://ieeexplore.ieee.org (institutional access may be required)
- JSTOR: https://www.jstor.org (institutional access may be required)

**Readability Analysis**:
- Readability Formulas: https://readabilityformulas.com/free-readability-formula-tests.php
- Hemingway Editor: https://hemingwayapp.com

**APA Citation Help**:
- Purdue OWL APA Guide: https://owl.purdue.edu/owl/research_and_citation/apa_style/apa_style_introduction.html
- APA Style Blog: https://apastyle.apa.org/blog
- Citation generators: Zotero, EasyBib, Citation Machine (verify output manually)

**Word Count**:
- Command line: `wc -w filename.md`
- Text editors: Most IDEs show word count (VS Code, Sublime Text)
- Online: wordcounter.net

**Markdown Editors** (recommended):
- VS Code with Markdown Preview
- Typora (WYSIWYG Markdown editor)
- Obsidian (knowledge management + Markdown)

---

## Next Steps After Planning

1. **Run `/sp.tasks`**: Generate implementation tasks from plan.md
2. **Execute Literature Search**: Follow research.md Decision 7 search strategy
3. **Populate JSON Schemas**: Structure evidence using contracts/*.schema.json
4. **Write Paper**: Follow quickstart.md word allocation and APA guidelines
5. **Validate Quality**: Check against success criteria (SC-001 to SC-010)
6. **Distribute**: Export Markdown to PDF or publish to docs/research-papers/

**Estimated Effort**: 40-50 hours over 2 weeks (3-4 hours/day for literature search, synthesis, writing, revision)
