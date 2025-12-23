# Quick Start Guide: Module 4 VLA Models

**Feature**: Module 4 - Vision-Language-Action (VLA) Models
**Audience**: Content creators, developers, and students
**Purpose**: Get up and running with Module 4 content quickly

---

## Overview

Module 4 teaches Vision-Language-Action (VLA) convergence in robotics: how voice commands, language understanding, and robot actions combine to create autonomous voice-controlled systems. This guide helps you navigate the module structure and understand the learning flow.

---

## Prerequisites

Before starting Module 4, ensure you have:

### Knowledge Prerequisites
- ✅ Completed **Module 1** (ROS 2 Basics): Understanding of nodes, topics, actions, launch files
- ✅ Completed **Module 3** (Isaac Sim & Nav2): Working Isaac Sim environment, Nav2 navigation configured
- ✅ Basic Python programming (functions, classes, error handling)
- ✅ Familiarity with command line interfaces (bash/PowerShell)

### System Requirements
- **OS**: Ubuntu 20.04/22.04 OR Windows 10/11
- **GPU**: NVIDIA GPU with 8GB+ VRAM (for Isaac Sim from Module 3)
- **RAM**: 16GB+ recommended
- **Disk**: 10GB free space (for Whisper models and code examples)
- **Internet**: Required for LLM API access or model downloads

### Software Prerequisites
- **Isaac Sim 2024.1+**: Installed in Module 3
- **ROS 2 Humble**: Installed in Module 1
- **Nav2**: Configured in Module 3 Ch5
- **Python 3.10+**: With pip package manager

---

## Module 4 Structure

### Chapter Overview

| Chapter | Title | Focus | Estimated Time | Dependencies |
|---------|-------|-------|----------------|--------------|
| **Ch1** | VLA Fundamentals | Conceptual foundation of VLA convergence | 2-3 hours | Modules 1, 3 |
| **Ch2** | Voice-to-Action with Whisper | Speech recognition and ROS 2 integration | 3-4 hours | Ch1 |
| **Ch3** | LLM Cognitive Planning | Language models for robotics task planning | 3-4 hours | Ch1, Ch2 |
| **Ch4** | ROS 2 Action Execution | Executing LLM-generated action sequences | 2-3 hours | Ch2, Ch3, Module 3 Ch5 |
| **Ch5** | End-to-End Capstone | Complete voice-controlled humanoid system | 3-4 hours | Ch1-Ch4 |

**Total Estimated Time**: 12-15 hours

---

## Learning Path

### Phase 1: Understand VLA Convergence (Ch1)

**Goal**: Grasp how vision, language, and action modalities work together

**You'll Learn**:
- Why LLMs are useful for robotics (flexibility, natural language interface)
- VLA pipeline architecture (speech → language → action → robot)
- When to use VLA vs hard-coded behaviors
- System architecture and data flow

**Key Deliverables**:
- Draw VLA pipeline diagram from memory
- Explain each stage: Whisper → LLM → ROS 2 → Robot
- Identify 3 appropriate use cases for VLA systems

---

### Phase 2: Implement Voice Input (Ch2)

**Goal**: Set up Whisper ASR and integrate with ROS 2

**You'll Learn**:
- Whisper model sizes (tiny, base, small, medium, large) and trade-offs
- Installing Whisper and dependencies (`pip install openai-whisper`)
- Capturing audio with `sounddevice` library
- Creating ROS 2 publisher node for transcriptions
- Multi-language support (English + one other language)

**Key Deliverables**:
- Working Whisper ROS 2 node publishing to `/voice_command` topic
- Test transcription with command: "Navigate to the kitchen"
- Measure transcription accuracy (>95% for clear speech)

**Quick Start Command**:
```bash
# Install Whisper
pip install openai-whisper sounddevice

# Test transcription (from Ch2 code examples)
python whisper_ros2_node.py --model base
```

---

### Phase 3: Add LLM Planning (Ch3)

**Goal**: Translate natural language to ROS 2 action sequences

**You'll Learn**:
- Prompt engineering for robotics (system prompts, few-shot examples)
- OpenAI API integration (with cost controls)
- Local LLM alternative (Ollama + Llama 7B)
- Structured output with Pydantic models
- Safety constraint validation

**Key Deliverables**:
- LLM generates `navigate_to_pose` action with correct parameters
- Complex command decomposition: "Find the red ball and bring it to me"
- Validation layer catches hallucinated actions

**Quick Start Command**:
```bash
# Set up OpenAI API (if using cloud LLM)
export OPENAI_API_KEY="your-api-key-here"

# OR set up local LLM (free alternative)
curl -fsSL https://ollama.ai/install.sh | sh
ollama pull llama2:7b

# Test LLM planner (from Ch3 code examples)
python llm_planner.py --command "Navigate to coordinates (2, 3)"
```

---

### Phase 4: Execute Actions (Ch4)

**Goal**: Connect LLM planning to physical robot execution

**You'll Learn**:
- ROS 2 action client implementation
- Integration with Nav2 (from Module 3 Ch5)
- Status monitoring and error handling
- Replanning on failure (LLM re-invocation)

**Key Deliverables**:
- Execute LLM-generated `navigate_to_pose` action in Isaac Sim
- Monitor action feedback on `/task_status` topic
- Handle action failure and trigger replanning

**Quick Start Command**:
```bash
# Launch Isaac Sim + Nav2 (from Module 3)
ros2 launch nav2_bringup bringup_launch.py

# Run action executor (from Ch4 code examples)
python action_executor.py --plan-file example_plan.json
```

---

### Phase 5: Build End-to-End System (Ch5)

**Goal**: Integrate all components into voice-controlled humanoid capstone

**You'll Learn**:
- System integration: Whisper + LLM + ROS 2 + Isaac Sim
- Performance monitoring (success rate, latency, task completion time)
- Debugging strategies (isolating failures in speech/LLM/execution layers)
- Optional: Object detection integration for full VLA loop

**Key Deliverables**:
- Complete system where voice command executes full task
- Success rate >80% over 10 trials
- Trace data flow through entire pipeline with logs

**Quick Start Command**:
```bash
# Launch complete VLA system (from Ch5 capstone)
python launch_vla_system.py

# Then speak command: "Navigate to the door"
# System will: transcribe → plan → execute → report status
```

---

## Common Workflows

### Workflow 1: Test Voice Transcription

```bash
# 1. Start Whisper ROS 2 node
python whisper_ros2_node.py --model base

# 2. In another terminal, monitor topic
ros2 topic echo /voice_command

# 3. Speak into microphone: "Move forward 2 meters"
# Expected: Transcription appears on topic with confidence >0.95
```

### Workflow 2: Generate Action Plan from Text

```bash
# 1. Test LLM planner with text input (bypass voice for debugging)
python llm_planner.py --command "Go to the table and pick up the cup"

# 2. Inspect generated plan
cat output/plan_12345.json

# 3. Validate plan against schema
python validate_plan.py --plan output/plan_12345.json --schema contracts/llm-plan.schema.json
```

### Workflow 3: Execute Action Sequence

```bash
# 1. Ensure Isaac Sim and Nav2 are running
ros2 topic list | grep /navigate_to_pose

# 2. Execute validated plan
python action_executor.py --plan output/plan_12345.json

# 3. Monitor execution status
ros2 topic echo /task_status
```

---

## Troubleshooting

### Issue 1: Whisper Out of Memory

**Symptom**: `RuntimeError: CUDA out of memory` or system freeze

**Solutions**:
1. Use smaller Whisper model: `--model tiny` or `--model base`
2. Run on CPU instead of GPU: `--device cpu`
3. Close other GPU applications (Isaac Sim, browser)

**Reference**: Ch2 Section 2.3 "Whisper Model Selection"

---

### Issue 2: LLM API Rate Limits

**Symptom**: `openai.error.RateLimitError: You exceeded your current quota`

**Solutions**:
1. Switch to local LLM: Install Ollama and use `llama2:7b` model
2. Implement rate limiting: See Ch3 code example `rate_limiter.py`
3. Add request delays: `time.sleep(2)` between API calls

**Reference**: Ch3 Section 3.4 "Cost Control and Rate Limiting"

---

### Issue 3: Action Validation Failures

**Symptom**: `ValidationError: action_type 'navigat_to_pose' does not exist` (typo in LLM output)

**Solutions**:
1. Check action exists: `ros2 action list`
2. Validate plan before execution: Use `validate_plan.py` script
3. Review LLM prompt: Ensure action names in few-shot examples are correct

**Reference**: Ch3 Section 3.6 "Safety Validation Layer"

---

### Issue 4: Nav2 Integration Not Working

**Symptom**: `Action server /navigate_to_pose not available`

**Solutions**:
1. Verify Nav2 running: `ros2 node list | grep nav2`
2. Re-launch Nav2: `ros2 launch nav2_bringup bringup_launch.py`
3. Check Module 3 Ch5 setup: Ensure costmaps and controllers configured

**Reference**: Module 3 Ch5 "Nav2 Integration"

---

## Code Examples Index

All code examples are located in `docs/module-4-vla/assets/code/`:

### Whisper Examples (Ch2)
- `whisper/install_whisper.sh` - Installation script
- `whisper/whisper_ros2_node.py` - ROS 2 publisher for transcriptions
- `whisper/test_transcription.py` - Standalone testing script

### LLM Examples (Ch3)
- `llm/llm_planner.py` - Main LLM planner class
- `llm/openai_client.py` - OpenAI API wrapper with rate limiting
- `llm/local_llama_client.py` - Local Llama integration via Ollama
- `llm/prompt_templates.py` - System prompts and few-shot examples
- `llm/action_validator.py` - Safety validation layer

### Integration Examples (Ch4)
- `integration/vla_pipeline.py` - Complete VLA pipeline orchestration
- `integration/action_executor.py` - ROS 2 action client for executing plans
- `integration/safety_constraints.py` - Workspace bounds and collision checking

### Capstone Examples (Ch5)
- `capstone/voice_controlled_humanoid.py` - End-to-end demonstration
- `capstone/launch_vla_system.py` - System launcher with all components
- `capstone/performance_metrics.py` - Success rate and latency tracking

---

## Additional Resources

### Documentation
- **Whisper**: https://github.com/openai/whisper
- **OpenAI API**: https://platform.openai.com/docs
- **Ollama**: https://ollama.ai/docs
- **Nav2**: https://navigation.ros.org/
- **ROS 2 Actions**: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/

### Research Papers (APA Format)
- Radford, A., Kim, J. W., Xu, T., Brockman, G., McLeavey, C., & Sutskever, I. (2022). Robust speech recognition via large-scale weak supervision. *arXiv preprint arXiv:2212.04356*.
- Brown, T. B., Mann, B., Ryder, N., et al. (2020). Language models are few-shot learners. *Advances in Neural Information Processing Systems*, 33, 1877-1901.

### Community Support
- **Textbook Issues**: GitHub Issues (link in textbook intro)
- **ROS Answers**: https://answers.ros.org/
- **OpenAI Forum**: https://community.openai.com/

---

## Success Criteria

By the end of Module 4, you should be able to:

- [x] Explain the VLA pipeline in under 5 minutes with a diagram
- [x] Configure Whisper for speech-to-text with >95% accuracy
- [x] Prompt LLM to generate valid ROS 2 action sequences
- [x] Execute LLM-generated plans in Isaac Sim
- [x] Debug failures at each stage (speech/LLM/execution)
- [x] Complete capstone: voice-controlled humanoid with >80% success rate

**Self-Assessment**: Use checklists at the end of each chapter to verify understanding before proceeding.

---

## Next Steps

1. **Start with Ch1**: Read "VLA Fundamentals" to build conceptual foundation
2. **Set Up Environment**: Install Whisper (`pip install openai-whisper`)
3. **Work Through Exercises**: Each chapter has 3-5 hands-on exercises
4. **Build Capstone**: Ch5 synthesizes all concepts into working demo
5. **Explore Extensions**: Add object detection (optional) for full VLA loop

**Estimated Completion**: 12-15 hours for thorough understanding with all exercises

---

**Document Version**: 1.0
**Last Updated**: 2025-12-22
**Maintained By**: Module 4 Content Team
