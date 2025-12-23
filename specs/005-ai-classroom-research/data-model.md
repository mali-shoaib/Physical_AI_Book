# Data Model: AI Classroom Efficiency Paper

**Date**: 2025-12-22
**Feature**: 005-ai-classroom-research
**Purpose**: Define entity structure for organizing research evidence and paper content

## Core Entities

### 1. AI Classroom Application

Represents a specific category of AI tool used in K-12 classrooms.

**Attributes**:
- `application_id` (string): Unique identifier (e.g., "automated-grading", "intelligent-tutoring", "ai-writing-feedback")
- `name` (string): Display name (e.g., "Automated Grading and Assessment Systems")
- `description` (string): Plain-language explanation of what the application does (2-3 sentences, \<100 words)
- `target_grade_levels` (array[string]): Applicable grade ranges (e.g., ["K-5", "6-8", "9-12"])
- `primary_subject_areas` (array[string]): Main disciplines (e.g., ["Mathematics", "English Language Arts", "Science"])
- `deployment_model` (string): How implemented (e.g., "Cloud-based SaaS", "On-premise software", "Hybrid")
- `example_platforms` (array[string]): Named products for context (e.g., ["Gradescope", "Turnitin Feedback Studio"]) - not endorsements, per scope exclusions

**Relationships**:
- Has many `Research Evidence` records (minimum 2 per SC-001)
- Has one `Teacher Workload Metric` summary
- Has one `Student Outcome Metric` summary
- Has one `Cost Component` set
- Has one `ROI Calculation` result

**Validation Rules** (from spec.md FR-001, FR-002):
- Minimum 3 applications required in paper
- Each application must link to at least 2 peer-reviewed evidence records
- Description must be accessible to non-technical administrators (no unexplained jargon)

**State Transitions**: N/A (static descriptive entity, not a workflow)

---

### 2. Research Evidence

Represents a peer-reviewed study documenting impact of an AI application.

**Attributes**:
- `evidence_id` (string): Unique identifier (e.g., "steenbergen-hu-2013-meta-analysis")
- `citation_apa` (string): Full APA 7th edition reference list entry
- `authors` (array[string]): Author names for in-text citations
- `year` (integer): Publication year (2015-2025 per research.md Decision 4)
- `title` (string): Study title
- `source` (string): Journal name, conference proceedings, or publication venue
- `doi` (string, optional): Digital Object Identifier
- `study_type` (enum): "RCT", "Quasi-Experimental", "Meta-Analysis", "Large-Scale Observational"
- `sample_size` (integer): Number of participants (students, teachers, or classrooms)
- `duration` (string): Study length (e.g., "12 weeks", "1 academic year")
- `context_grade_levels` (array[string]): Grades studied (e.g., ["6", "7", "8"])
- `context_subject` (string): Subject area (e.g., "Algebra", "Writing")
- `context_demographics` (string): School/student characteristics (e.g., "Urban, 60% free/reduced lunch")
- `measured_outcomes` (array[string]): Specific metrics reported (e.g., ["Grading time reduction", "Test score gains"])
- `key_findings` (string): Summary of results relevant to teacher workload or student outcomes (2-3 sentences)

**Relationships**:
- Belongs to one or more `AI Classroom Application` (studies may compare multiple tools)
- Supports one or more `Teacher Workload Metric` or `Student Outcome Metric` claims

**Validation Rules** (from spec.md FR-002, FR-015):
- Minimum 15 evidence records required across all applications
- Publication year must be 2015-2025
- Must be peer-reviewed (journal article, conference proceedings, or systematic review)
- Citation must follow APA 7th edition format

---

### 3. Teacher Workload Metric

Represents quantified impact on teacher time or effort for a specific AI application.

**Attributes**:
- `metric_id` (string): Unique identifier
- `application_id` (string): Foreign key to AI Classroom Application
- `task_category` (enum): "Grading", "Lesson Planning", "Administrative", "Feedback Delivery"
- `baseline_time_hours_per_week` (float): Teacher time investment without AI
- `post_ai_time_hours_per_week` (float): Teacher time investment with AI
- `time_saved_hours_per_week` (float): Calculated difference (baseline - post_ai)
- `percentage_reduction` (float): Calculated (time_saved / baseline) × 100
- `calculation_methodology` (string): How metric was measured (e.g., "Teacher time logs over 8 weeks", "Survey self-report")
- `confidence_level` (string): Statistical confidence or certainty (e.g., "p \< 0.05", "95% CI [3.2, 5.8 hours]")
- `supporting_evidence_ids` (array[string]): References to Research Evidence records

**Relationships**:
- Belongs to one `AI Classroom Application`
- Supported by one or more `Research Evidence` records

**Validation Rules** (from spec.md FR-003, SC-002):
- All applications must have at least one quantified workload metric
- Metrics must be specific (not vague claims like "reduces workload")
- Must cite peer-reviewed evidence for calculation

**Calculation Example**:
```
Automated Grading:
- Baseline: 8 hours/week grading assignments
- Post-AI: 3 hours/week reviewing auto-graded work
- Time Saved: 5 hours/week
- Percentage: 62.5% reduction
```

---

### 4. Student Outcome Metric

Represents quantified impact on student learning or engagement for a specific AI application.

**Attributes**:
- `metric_id` (string): Unique identifier
- `application_id` (string): Foreign key to AI Classroom Application
- `metric_type` (enum): "Test Scores", "Engagement", "Learning Efficiency", "Completion Rates"
- `baseline_measurement` (string): Pre-intervention value (e.g., "Pre-test mean: 68%", "Homework completion: 65%")
- `post_ai_measurement` (string): Post-intervention value (e.g., "Post-test mean: 79%", "Homework completion: 85%")
- `improvement_quantified` (string): Calculated gain (e.g., "+11 percentage points", "+20% completion rate")
- `statistical_significance` (string): Effect size or p-value (e.g., "Cohen's d = 0.42", "p \< 0.01")
- `supporting_evidence_ids` (array[string]): References to Research Evidence records
- `equity_considerations` (string, optional): Differential impact by student subgroup (e.g., "Larger gains for struggling students, d = 0.58 vs. d = 0.31 for advanced")

**Relationships**:
- Belongs to one `AI Classroom Application`
- Supported by one or more `Research Evidence` records

**Validation Rules** (from spec.md FR-004, SC-003):
- All applications must have at least one quantified outcome metric
- Metrics must be specific (not vague claims like "improves learning")
- Must cite peer-reviewed evidence for measurement

**Calculation Example**:
```
Intelligent Tutoring System:
- Baseline: Pre-test mean 68% (SD = 12)
- Post-AI: Post-test mean 79% (SD = 10)
- Improvement: +11 percentage points
- Significance: Cohen's d = 0.98 (large effect), p < 0.001
```

---

### 5. Cost Component

Represents financial elements of adopting an AI application.

**Attributes**:
- `cost_id` (string): Unique identifier
- `application_id` (string): Foreign key to AI Classroom Application
- `category` (enum): "Software Licensing", "Implementation", "Training", "Maintenance"
- `cost_type` (enum): "One-Time", "Recurring Annual"
- `pricing_model` (enum): "Per-Student", "Per-Teacher", "Per-School", "Flat Rate"
- `cost_low_estimate` (float): Lower bound of cost range (in USD)
- `cost_high_estimate` (float): Upper bound of cost range (in USD)
- `cost_assumptions` (string): Context for estimate (e.g., "Based on 500-student school, 25 teachers, published vendor pricing 2024")
- `data_source` (string): Where cost data obtained (e.g., "Vendor website", "Published case study: Smith et al., 2023")

**Relationships**:
- Belongs to one `AI Classroom Application`
- Used in `ROI Calculation`

**Validation Rules** (from spec.md FR-006):
- All applications must have costs for all 4 categories (Licensing, Implementation, Training, Maintenance)
- Cost ranges acceptable due to context variability
- Data source must be cited (vendor pricing, published study, or estimate with justification)

**Example**:
```
Automated Grading System:
- Software Licensing: $5-12 per student per year (recurring)
- Implementation: $2,000-5,000 one-time (setup, data integration)
- Training: $1,500-3,000 one-time (2-day workshop for 25 teachers)
- Maintenance: $500-1,000 per year (IT support, updates)
```

---

### 6. ROI Calculation

Represents cost-benefit analysis for an AI application.

**Attributes**:
- `roi_id` (string): Unique identifier
- `application_id` (string): Foreign key to AI Classroom Application
- `school_context` (string): Scenario description (e.g., "Medium-sized urban elementary, 500 students, 25 teachers")
- `total_cost_year1` (float): First-year costs (one-time + recurring)
- `total_cost_annual_recurring` (float): Ongoing annual costs (years 2+)
- `annual_benefits_quantified` (float): Yearly value of time savings (hours × hourly rate)
- `teacher_hourly_rate_assumption` (float): Rate used for calculation (e.g., $40/hour)
- `roi_percentage` (float): Calculated (Benefits - Costs) / Costs × 100
- `break_even_months` (integer): Months until cumulative benefits exceed cumulative costs
- `payback_period_description` (string): Plain-language explanation (e.g., "Investment pays for itself within 18 months")
- `assumptions_and_caveats` (string): Limitations of ROI calculation (e.g., "Assumes full teacher adoption; excludes intangible benefits like teacher satisfaction")

**Relationships**:
- Belongs to one `AI Classroom Application`
- Derived from `Cost Component` set and `Teacher Workload Metric`

**Validation Rules** (from spec.md FR-007, SC-004):
- All applications must have at least one ROI calculation example
- Break-even timeline required
- Assumptions must be documented (transparent methodology)

**Calculation Example**:
```
Automated Grading - Medium School (500 students, 25 teachers):
- Year 1 Costs: $8,500 (licensing) + $3,500 (implementation) + $2,000 (training) + $750 (maintenance) = $14,750
- Annual Recurring: $8,500 (licensing) + $750 (maintenance) = $9,250
- Annual Benefits: 25 teachers × 5 hours/week × 36 weeks × $40/hour = $180,000
- ROI Year 1: ($180,000 - $14,750) / $14,750 × 100 = 1,120%
- Break-Even: ~1 month (benefits far exceed costs)
- Caveat: Assumes all 25 teachers adopt tool and achieve 5 hours/week savings
```

---

## Entity Relationships Diagram

```
AI Classroom Application (3 required)
├── has many Research Evidence (2+ per application)
├── has one Teacher Workload Metric (summary across evidence)
├── has one Student Outcome Metric (summary across evidence)
├── has many Cost Components (4 categories: licensing, implementation, training, maintenance)
└── has one ROI Calculation (derived from costs + workload metric)

Research Evidence (15+ total)
├── supports one or more AI Classroom Application
├── informs Teacher Workload Metric calculations
└── informs Student Outcome Metric calculations

Comparison Table (deliverable artifact)
├── rows: 3 AI Classroom Applications
└── columns: Workload Impact | Student Outcomes | Total Cost | ROI | Break-Even
```

## Data Flow for Paper Writing

**Phase 2 (Tasks Generation)**:
1. Literature search → populate Research Evidence entities (15+)
2. Evidence synthesis → populate Teacher Workload Metrics (1 per application)
3. Evidence synthesis → populate Student Outcome Metrics (1 per application)
4. Cost research → populate Cost Components (4 per application)
5. Calculation → populate ROI Calculations (1+ per application)
6. Aggregation → generate Comparison Table (3 applications × 5 metrics)

**Phase 3 (Writing)**:
1. Executive Summary: Synthesize key findings from all entities
2. Introduction: Context for paper, no entity data required
3. Application Sections (3): One section per AI Classroom Application entity, embedding Research Evidence, Workload Metrics, Outcome Metrics
4. ROI Section: Present ROI Calculation methodology with examples from 2-3 applications
5. Implementation Section: Extract common prerequisites from Research Evidence contexts
6. Limitations Section: Aggregate caveats from ROI assumptions and evidence limitations
7. Conclusion: Synthesize actionable recommendations
8. References: APA-formatted list from all Research Evidence records

## Validation Checklist

Before writing phase:
- [ ] 3+ AI Classroom Application entities defined
- [ ] 15+ Research Evidence entities documented
- [ ] Each application has 2+ linked evidence records
- [ ] Each application has quantified Teacher Workload Metric
- [ ] Each application has quantified Student Outcome Metric
- [ ] Each application has 4 Cost Component categories
- [ ] Each application has at least 1 ROI Calculation
- [ ] All citations in APA 7th edition format
- [ ] All data sources cited (no unsupported claims)

**Next Steps**: Create JSON schemas in `/contracts/` to structure data collection during literature search and evidence synthesis.
