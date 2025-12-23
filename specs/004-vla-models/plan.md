# Implementation Plan: Module 4 - Vision-Language-Action (VLA) Models

**Branch**: `004-vla-models` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-vla-models/spec.md`

**Note**: This plan follows the Spec-Driven Development workflow and documents the architectural approach for creating Module 4 educational content.

## Summary

Module 4 teaches students how vision, language, and action modalities converge in modern robotics to enable voice-controlled autonomous systems. The module covers Whisper ASR for speech-to-text, LLM cognitive planning for translating natural language to robot actions, ROS 2 action execution, and culminates in an end-to-end capstone where a humanoid robot in Isaac Sim responds to voice commands.

**Primary Requirement**: Create comprehensive educational content (chapters, code examples, exercises) that teaches VLA convergence through hands-on implementation, building on Module 3's Isaac Sim and Nav2 foundation.

**Technical Approach**: Docusaurus-based documentation with markdown chapters, embedded Python/Bash code examples, Mermaid diagrams for architecture visualization, and contract schemas for validation. Content follows research-concurrent workflow: gather best practices → design chapter structure → create examples → validate against acceptance criteria.

## Technical Context

**Language/Version**:
- Markdown (MDX) for chapter content
- Python 3.10+ for code examples
- Bash for shell scripts
- JavaScript/React for Docusaurus 3.x

**Primary Dependencies**:
- **Documentation**: Docusaurus 3.x, MDX, Mermaid (diagrams)
- **Code Examples**: openai-whisper, openai (API client), rclpy (ROS 2 Python), sounddevice, numpy, pydantic
- **ROS 2**: Humble, nav2_msgs, action_msgs, geometry_msgs
- **Simulation**: Isaac Sim 2024.1+ (from Module 3)

**Storage**:
- Static files (markdown chapters, code examples, diagrams, screenshots)
- Git repository for version control

**Testing**:
- Manual validation of code examples against acceptance criteria
- Contract schemas for structured data validation (JSON Schema)
- Docusaurus build verification (MDX syntax, link checking)

**Target Platform**:
- Web (Docusaurus static site)
- Development environment: Ubuntu 20.04/22.04 or Windows 10/11 with NVIDIA GPU (8GB+ VRAM) for running examples

**Project Type**: Documentation/Educational Content (Docusaurus static site)

**Performance Goals**:
- Chapter load time <2 seconds
- Code examples execute successfully on student hardware (see spec assumptions)
- Whisper transcription: real-time or near-real-time (<2x audio duration)
- LLM response latency: <5 seconds for action planning

**Constraints**:
- Must integrate seamlessly with existing Module 3 content (Nav2, Isaac Sim)
- APA citation style required (per user request and constitution reference)
- Code examples must be self-contained and independently testable
- No deep ML theory (keep accessible to beginner/intermediate students)
- Simulation-only (no physical hardware requirements)

**Scale/Scope**:
- 4-5 chapters (estimated 15-20 pages each)
- 20-25 Python code examples (Whisper integration, LLM clients, ROS 2 nodes, action validation)
- 10-12 Mermaid diagrams (VLA pipeline, data flow, system architecture)
- 1 comprehensive capstone project
- 3-4 contract schemas for validation
- Estimated student completion time: 12-15 hours

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Since the project constitution is not yet fully defined, this check evaluates against general educational content development principles and the user's explicit requirements:

### Educational Content Principles

- [x] **APA Citation Style**: Per user request, all references and citations must follow APA format
- [x] **Research-Concurrent Approach**: Plan follows Research → Foundation → Analysis → Synthesis phases
- [x] **Self-Contained Examples**: All code examples must be independently testable (aligns with spec FR-016, FR-017)
- [x] **Beginner-Accessible**: No deep ML/LLM theory, explanations kept simple and accurate (per spec constraints)
- [x] **Integration with Existing Content**: Must seamlessly integrate with Module 3 Nav2 and Isaac Sim (spec dependency)

### Quality Gates

- [x] **Testable Acceptance Criteria**: Each user story has clear Given-When-Then scenarios that can be validated
- [x] **No Implementation Lock-In**: Spec describes capabilities, not specific tech choices (e.g., "LLM" not "GPT-4 only")
- [x] **Measurable Success**: 14 success criteria defined with quantitative metrics (SC-001 through SC-014)
- [x] **Risk Mitigation Documented**: 8 risks identified with mitigation strategies (RISK-001 through RISK-008)
- [x] **Clear Scope Boundaries**: In-scope and out-of-scope explicitly defined to prevent scope creep

### Architecture Quality

- [x] **Docusaurus Layout Defined**: Chapter structure aligns with sidebar navigation pattern from Module 3
- [x] **Validation Strategy**: Contract schemas for structured data (LLM outputs, action sequences)
- [x] **Modular Design**: Each chapter is independently testable and deployable (aligns with P1-P5 user story priorities)

**Status**: ✅ PASSED - All gates satisfied. Proceeding to Phase 0 research.

**Notes**:
- APA style requirement captured in Technical Context constraints
- Research-concurrent approach documented in Technical Approach summary
- No constitution violations requiring complexity justification

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-models/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0: Technology decisions and best practices
├── data-model.md        # Phase 1: Key entities and data structures
├── quickstart.md        # Phase 1: Quick start guide for Module 4
├── contracts/           # Phase 1: JSON schemas for validation
│   ├── voice-command.schema.json
│   ├── llm-plan.schema.json
│   └── action-sequence.schema.json
├── checklists/
│   └── requirements.md  # Specification quality checklist (completed)
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks command)
```

### Content Structure (Docusaurus)

This is a **documentation/educational content project** using Docusaurus 3.x static site generator. The structure follows the existing pattern from Module 3.

```text
docs/
├── intro.md                     # Project introduction (existing)
├── module-1-ros2/               # Module 1 content (existing)
├── module-3-isaac/              # Module 3 content (existing, completed)
│   ├── ch1-isaac-sim-basics.md
│   ├── ch2-synthetic-data-generation.md
│   ├── ch3-vslam-isaac-ros.md
│   ├── ch4-depth-perception-mapping.md
│   ├── ch5-nav2-integration.md
│   ├── ch6-end-to-end-capstone.md
│   └── assets/
│       ├── code/
│       ├── models/
│       └── diagrams/
└── module-4-vla/                # NEW: Module 4 content (this feature)
    ├── ch1-vla-fundamentals.md          # VLA convergence concepts
    ├── ch2-voice-to-action.md           # Whisper ASR integration
    ├── ch3-llm-cognitive-planning.md    # LLM for robotics task planning
    ├── ch4-ros2-action-execution.md     # Executing LLM-generated plans
    ├── ch5-end-to-end-capstone.md       # Complete voice-controlled humanoid
    └── assets/
        ├── code/                         # Python/Bash code examples
        │   ├── whisper/
        │   │   ├── install_whisper.sh
        │   │   ├── whisper_ros2_node.py
        │   │   └── test_transcription.py
        │   ├── llm/
        │   │   ├── llm_planner.py
        │   │   ├── openai_client.py
        │   │   ├── local_llama_client.py
        │   │   ├── prompt_templates.py
        │   │   └── action_validator.py
        │   ├── integration/
        │   │   ├── vla_pipeline.py
        │   │   ├── action_executor.py
        │   │   └── safety_constraints.py
        │   └── capstone/
        │       ├── voice_controlled_humanoid.py
        │       ├── launch_vla_system.py
        │       └── performance_metrics.py
        ├── diagrams/                     # Mermaid diagram source files
        │   ├── vla-pipeline.mmd
        │   ├── whisper-integration.mmd
        │   ├── llm-planning-flow.mmd
        │   └── system-architecture.mmd
        └── images/                       # Screenshots and illustrations
            ├── whisper-setup.png
            ├── llm-planning-example.png
            └── capstone-demo.png

sidebars.js                      # Navigation configuration (update required)
docusaurus.config.js             # Site configuration (existing)
```

**Structure Decision**: Using Docusaurus documentation structure (Option: Educational Content). This aligns with existing Module 1 and Module 3 patterns, ensuring consistent navigation and user experience. The `docs/module-4-vla/` directory will contain 5 markdown chapters with embedded code examples, Mermaid diagrams, and screenshots. Validation schemas live in `specs/004-vla-models/contracts/` for development reference.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations detected. All design decisions align with educational content best practices and specified constraints.

---

## Phase 0: Research & Technology Decisions

This phase resolves all "NEEDS CLARIFICATION" items from Technical Context and documents technology choices with rationale.

### Deliverables

- [x] **research.md** - Technology decisions with rationale, alternatives considered, best practices
  - Documentation framework: Docusaurus 3.x
  - Speech recognition: OpenAI Whisper (dual approach with model size options)
  - LLMs: Cloud (OpenAI API) + Local (Ollama + Llama) dual approach
  - ROS 2: Humble with Nav2 integration from Module 3
  - Validation: JSON Schema + Pydantic for contract testing
  - Diagrams: Mermaid (text-based, version-controllable)
  - Code style: Python 3.10+ with type hints
  - Citation format: APA 7th edition
  - Risk mitigation strategies for RISK-001, RISK-002, RISK-003

### Key Decisions Made

1. **Dual LLM Approach**: Provide both cloud (OpenAI) and local (Llama) options to maximize student accessibility
   - **Rationale**: Addresses cost concerns (RISK-001) while maintaining quality
   - **Trade-off**: More complexity in teaching, but better student outcomes

2. **Whisper Model Flexibility**: Support tiny, base, small, medium, large models
   - **Rationale**: Different students have different hardware capabilities (RISK-003)
   - **Trade-off**: Must test examples on multiple model sizes

3. **JSON Schema Validation**: Use schemas for all data contracts
   - **Rationale**: Catch LLM hallucinations before robot execution (RISK-002)
   - **Trade-off**: Additional validation code required

4. **APA Citation Style**: Per user request, follow APA 7th edition
   - **Rationale**: Educational standard for social sciences/education research
   - **Trade-off**: None - clear requirement from user

5. **Module 3 Integration**: Reuse Nav2 setup, Isaac Sim environment, humanoid models
   - **Rationale**: Reduces setup time, maintains learning continuity
   - **Trade-off**: Module 4 cannot be standalone (requires Module 3 completion)

**Status**: ✅ COMPLETE - All technology decisions documented in research.md

---

## Phase 1: Design & Contracts

This phase defines data models, generates API contracts, creates quickstart guide, and updates agent context.

### Deliverables

- [x] **data-model.md** - Key entities with attributes, relationships, validation rules
  - Voice Command: Audio → text transcription entity
  - LLM Plan: Structured action sequences from language model
  - ROS 2 Action: Executable robot behaviors
  - Task Execution Log: End-to-end pipeline record
  - Entity relationship diagram (Mermaid)
  - State transition diagrams for each entity
  - Cross-entity validation rules

- [x] **contracts/** - JSON schemas for validation
  - `voice-command.schema.json` - Voice Command structure validation
  - `llm-plan.schema.json` - LLM Plan structure validation
  - `action-sequence.schema.json` - ROS 2 Action structure validation
  - All schemas follow JSON Schema Draft 07 specification

- [x] **quickstart.md** - Getting started guide for Module 4
  - Prerequisites (knowledge, system, software)
  - Chapter overview with time estimates
  - Learning path through all 5 chapters
  - Common workflows (test transcription, generate plans, execute actions)
  - Troubleshooting guide (4 common issues)
  - Code examples index
  - Success criteria checklist

- [x] **Agent context updated** - Claude Code context file updated with Module 4 technology
  - Project type: Documentation/Educational Content (Docusaurus)
  - Technology stack added to context
  - Agent-specific guidance for Module 4 implementation

### Design Highlights

**Entity Architecture**:
- 4 core entities model complete VLA pipeline
- Clear separation of concerns (input → processing → execution → logging)
- Referential integrity enforced via UUID foreign keys
- State machines for tracking execution progress

**Validation Strategy**:
- **Pre-execution**: Validate LLM output against JSON schemas before robot actions
- **Safety layer**: Check workspace bounds, action existence, collision constraints
- **Runtime**: Monitor action feedback and status
- **Post-execution**: Record outcomes in Task Execution Log for analysis

**Integration Points**:
- **Module 3 Ch5 Nav2**: `navigate_to_pose`, `follow_path` actions (line 215)
- **Module 3 Ch1 Isaac Sim**: Humanoid robot models, ROS 2 bridge, simulation environment
- **Module 1 ROS 2**: Node creation, topic/action patterns, launch files

**Chapter Structure Template**:
Each of 5 chapters follows consistent pattern:
1. Introduction (5%) - Objectives, prerequisites, time estimate
2. Conceptual Foundation (25%) - Theory, diagrams, "why this matters"
3. Hands-On Implementation (50%) - Code walkthrough, working examples
4. Exercises (15%) - 3-5 exercises, simple to challenging
5. Summary & Next Steps (5%) - Takeaways, preview of next chapter

**Status**: ✅ COMPLETE - All Phase 1 artifacts generated

---

## Phase 2: Task Breakdown

**Note**: Phase 2 (task generation) is handled by `/sp.tasks` command, NOT `/sp.plan`.

The `/sp.tasks` command will:
1. Read this plan and spec.md
2. Generate testable, ordered tasks in tasks.md
3. Include acceptance criteria for each task
4. Map tasks to user stories (P1-P5)
5. Estimate task complexity and dependencies

**Next Command**: Run `/sp.tasks` to generate tasks.md

---

## Summary

### Planning Complete

**Artifacts Generated**:
1. ✅ plan.md (this file) - Implementation plan with architecture
2. ✅ research.md - Technology decisions and best practices
3. ✅ data-model.md - Entity definitions with validation rules
4. ✅ contracts/ - JSON schemas (3 files)
5. ✅ quickstart.md - Getting started guide
6. ✅ Agent context updated - CLAUDE.md includes Module 4 technology

**Architecture Summary**:
- **Content Type**: Docusaurus 3.x static site
- **Structure**: 5 chapters, 20-25 code examples, 10-12 diagrams
- **Learning Flow**: VLA Fundamentals → Voice Input → LLM Planning → Action Execution → Capstone
- **Integration**: Builds on Module 3 (Isaac Sim, Nav2) and Module 1 (ROS 2 basics)
- **Data Pipeline**: Voice Command → LLM Plan → ROS 2 Actions → Task Execution Log
- **Validation**: JSON Schema + Pydantic for safety before robot execution

**Key Decisions**:
1. Dual LLM approach (cloud + local) for accessibility
2. Flexible Whisper model sizes for hardware compatibility
3. JSON Schema validation to catch hallucinations (safety layer)
4. APA citation style per user requirement
5. Reuse Module 3 infrastructure for seamless integration

**Risk Mitigation**:
- RISK-001 (LLM costs): Local LLM alternative, rate limiting examples
- RISK-002 (Hallucinations): Validation layer, ground truth datasets
- RISK-003 (Whisper hardware): Multiple model sizes, CPU fallback, Colab notebooks

**Ready for Implementation**: Run `/sp.tasks` to generate implementation tasks.

---

**Branch**: `004-vla-models`
**Spec**: `specs/004-vla-models/spec.md`
**Plan**: `specs/004-vla-models/plan.md` (this file)
**Status**: Phase 0 & 1 COMPLETE ✅ | Phase 2 pending (`/sp.tasks` command)
