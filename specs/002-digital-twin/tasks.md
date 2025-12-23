---
description: "Task list for Module 2 - Digital Twin (Gazebo & Unity) implementation"
---

# Tasks: Module 2 - Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: No test tasks included (not specified in feature requirements - content validation only)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus project**: `docs/module-2-digital-twin/` for chapters
- **Code assets**: `docs/module-2-digital-twin/assets/code/` for examples
- **Unity project**: `docs/module-2-digital-twin/assets/unity/`
- **Contracts**: `specs/002-digital-twin/contracts/` for schemas

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project structure and directory creation for Module 2

- [X] T001 Create module directory structure docs/module-2-digital-twin/
- [X] T002 Create assets subdirectories: assets/code/{worlds,models,launch,scripts,config}/
- [X] T003 [P] Create Unity project directory: assets/unity/PhysicalAI_Module2/
- [X] T004 [P] Copy contract templates from specs/002-digital-twin/contracts/ to reference location
- [X] T005 Update sidebars.js to include Module 2 category and chapter entries

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY chapter can be implemented

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [X] T006 Create humanoid robot URDF model in docs/module-2-digital-twin/assets/code/models/simple_biped/model.sdf
- [X] T007 [P] Create humanoid robot model.config in docs/module-2-digital-twin/assets/code/models/simple_biped/model.config
- [X] T008 [P] Create sensor platform URDF model in docs/module-2-digital-twin/assets/code/models/sensor_platform/model.sdf
- [X] T009 [P] Create sensor platform model.config in docs/module-2-digital-twin/assets/code/models/sensor_platform/model.config
- [X] T010 Create base RViz2 configuration file in docs/module-2-digital-twin/assets/code/config/rviz_base.rviz
- [X] T011 Extend validate_urdf.py script to support SDF validation using gz sdf --check
- [X] T012 Create validation checklist based on specs/002-digital-twin/checklists/requirements.md

**Checkpoint**: ✅ Foundation ready - chapter implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create Physics-Based Humanoid Simulation in Gazebo (Priority: P1) 🎯 MVP

**Goal**: Enable students to create functional Gazebo simulations with custom physics parameters, demonstrating gravity, collisions, and joint dynamics

**Independent Test**: Launch Gazebo world files and verify robot stability, physics behavior, and simulation runs at >= 1.0 RTF without errors

### Chapter Content for User Story 1

- [ ] T013 [P] [US1] Create chapter file docs/module-2-digital-twin/ch1-gazebo-physics.md with frontmatter per specs/002-digital-twin/contracts/chapter-frontmatter-schema.yaml
- [ ] T014 [US1] Write Section 1: Introduction to Gazebo Fortress (architecture overview, SDF format basics) in ch1-gazebo-physics.md
- [ ] T015 [US1] Write Section 2: Physics Engine Configuration (gravity, friction, restitution, solver parameters) in ch1-gazebo-physics.md
- [ ] T016 [US1] Write Section 3: World File Creation (template walkthrough, model inclusion, plugin loading) in ch1-gazebo-physics.md
- [ ] T017 [US1] Write Section 4: Collision Detection (geometries, contact forces, friction cones) in ch1-gazebo-physics.md
- [ ] T018 [US1] Write Section 5: Joint Dynamics (revolute, prismatic, fixed joints, limits, damping) in ch1-gazebo-physics.md
- [ ] T019 [US1] Write Section 6: Troubleshooting (physics instabilities, performance optimization) in ch1-gazebo-physics.md
- [ ] T020 [US1] Add Learning Objectives, Prerequisites, Summary, and Exercises sections per FR-015 to ch1-gazebo-physics.md

### Code Examples for User Story 1

- [ ] T021 [P] [US1] Create empty_world.sdf (minimal Gazebo template) in docs/module-2-digital-twin/assets/code/worlds/empty_world.sdf
- [ ] T022 [P] [US1] Create physics_demo.sdf (gravity + friction tests) in docs/module-2-digital-twin/assets/code/worlds/physics_demo.sdf
- [ ] T023 [P] [US1] Create collision_test.sdf (various geometries: box, sphere, mesh) in docs/module-2-digital-twin/assets/code/worlds/collision_test.sdf
- [ ] T024 [US1] Create humanoid_world.sdf (full robot simulation with joints and physics) in docs/module-2-digital-twin/assets/code/worlds/humanoid_world.sdf
- [ ] T025 [P] [US1] Create gazebo_physics.launch.py (ROS 2 launch file for world loading) in docs/module-2-digital-twin/assets/code/launch/gazebo_physics.launch.py
- [ ] T026 [P] [US1] Create spawn_robot.py (Python helper script) in docs/module-2-digital-twin/assets/code/scripts/spawn_robot.py

### Validation for User Story 1

- [ ] T027 [US1] Validate all SDF files with gz sdf --check <file> for empty_world.sdf, physics_demo.sdf, collision_test.sdf, humanoid_world.sdf
- [ ] T028 [US1] Validate Python launch files with python validate_examples.py for gazebo_physics.launch.py and spawn_robot.py
- [ ] T029 [US1] Test Gazebo world loading: Launch each world file in headless mode for 10 seconds, verify no errors
- [ ] T030 [US1] Verify chapter frontmatter against JSON schema in specs/002-digital-twin/contracts/chapter-frontmatter-schema.yaml
- [ ] T031 [US1] Test acceptance scenario: Launch physics_demo.sdf and verify RTF >= 1.0 on 8GB RAM system
- [ ] T032 [US1] Embed APA citations to Gazebo Fortress documentation (gazebosim.org/docs) per FR-010

**Checkpoint**: At this point, User Story 1 (Chapter 1: Gazebo Physics) should be fully functional, validated, and independently testable

---

## Phase 4: User Story 2 - Build Interactive 3D Environments in Unity for Robot Visualization (Priority: P2)

**Goal**: Enable students to create visually rich Unity environments for robot visualization with ROS 2 integration

**Independent Test**: Open Unity scenes, verify robot model rendering, establish ROS 2 TCP connection, and observe real-time joint state updates

### Chapter Content for User Story 2

- [ ] T033 [P] [US2] Create chapter file docs/module-2-digital-twin/ch2-unity-rendering.md with frontmatter per chapter-frontmatter-schema.yaml
- [ ] T034 [US2] Write Section 1: Unity Project Setup (creating project, installing ROS-TCP-Connector) in ch2-unity-rendering.md
- [ ] T035 [US2] Write Section 2: ROS 2 Integration Architecture (TCP Endpoint configuration, message serialization) in ch2-unity-rendering.md
- [ ] T036 [US2] Write Section 3: Importing Robot Models (URDF Importer package, material and texture setup) in ch2-unity-rendering.md
- [ ] T037 [US2] Write Section 4: Environment Design (lighting: directional/point/area, navigation meshes) in ch2-unity-rendering.md
- [ ] T038 [US2] Write Section 5: Real-Time Synchronization (joint state subscribers, transform updates) in ch2-unity-rendering.md
- [ ] T039 [US2] Write Section 6: Troubleshooting (connection failures, performance bottlenecks) in ch2-unity-rendering.md
- [ ] T040 [US2] Add Learning Objectives, Prerequisites, Summary, and Exercises sections per FR-015 to ch2-unity-rendering.md

### Unity Project Setup for User Story 2

- [ ] T041 [US2] Create Unity 2022.3 LTS project in docs/module-2-digital-twin/assets/unity/PhysicalAI_Module2/
- [ ] T042 [US2] Install ROS-TCP-Connector package in Unity project via Package Manager
- [ ] T043 [P] [US2] Install URDF Importer package in Unity project
- [ ] T044 [P] [US2] Install Unity Perception package (preparation for US3 sensors)
- [ ] T045 [US2] Configure Unity project settings (quality, physics, player settings) per performance requirements (>=30 FPS)

### Unity Scenes for User Story 2

- [ ] T046 [P] [US2] Create OfficeEnvironment.unity scene in Assets/Scenes/ with furniture, lighting, and navigation mesh
- [ ] T047 [P] [US2] Create FactoryFloor.unity scene in Assets/Scenes/ with industrial props and robot workspace
- [ ] T048 [US2] Import humanoid URDF model into Unity scenes using URDF Importer
- [ ] T049 [US2] Configure materials and textures for humanoid robot in Unity (realistic rendering per acceptance criteria)
- [ ] T050 [US2] Set up lighting in both scenes (directional sun, point lights for realism)

### Unity Scripts for User Story 2

- [ ] T051 [P] [US2] Create ROSConnection.cs (Unity C# script for TCP connection) in Assets/Scripts/ROSConnection.cs
- [ ] T052 [P] [US2] Create JointStatePublisher.cs (Unity C# script for joint updates) in Assets/Scripts/JointStatePublisher.cs
- [ ] T053 [US2] Integrate ROSConnection.cs into Unity scenes as GameObject component
- [ ] T054 [US2] Configure TCP endpoint settings (IP: 127.0.0.1, Port: 10000) in Unity Inspector

### ROS 2 Integration for User Story 2

- [ ] T055 [P] [US2] Create robot_visualization.launch.py (ROS 2 launch file for Unity integration) in docs/module-2-digital-twin/assets/code/launch/robot_visualization.launch.py
- [ ] T056 [US2] Configure ros_tcp_endpoint node in robot_visualization.launch.py
- [ ] T057 [US2] Test ROS 2 → Unity communication: Publish joint states, verify Unity robot updates

### Validation for User Story 2

- [ ] T058 [US2] Validate Python launch files with validate_examples.py for robot_visualization.launch.py
- [ ] T059 [US2] Validate Unity C# scripts syntax (basic compilation check in Unity Editor)
- [ ] T060 [US2] Test Unity scene loading: Open OfficeEnvironment.unity and FactoryFloor.unity, verify no errors
- [ ] T061 [US2] Test acceptance scenario: Launch ros_tcp_endpoint + Unity scene, verify real-time joint synchronization
- [ ] T062 [US2] Verify Unity scene file sizes <500MB per constraint in plan.md
- [ ] T063 [US2] Embed APA citations to Unity Robotics Hub documentation (github.com/Unity-Technologies/Unity-Robotics-Hub) per FR-010

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - Gazebo physics simulation + Unity visualization

---

## Phase 5: User Story 3 - Simulate Virtual Sensors for Perception Testing (Priority: P3)

**Goal**: Enable students to integrate virtual LiDAR, depth cameras, and IMUs into simulations with realistic noise models

**Independent Test**: Attach sensors to robot models, launch simulations, subscribe to sensor topics via ROS 2, and visualize data in RViz2

### Chapter Content for User Story 3

- [ ] T064 [P] [US3] Create chapter file docs/module-2-digital-twin/ch3-sensor-simulation.md with frontmatter per chapter-frontmatter-schema.yaml
- [ ] T065 [US3] Write Section 1: Sensor Plugin Architecture (Gazebo plugin system, Unity Perception package) in ch3-sensor-simulation.md
- [ ] T066 [US3] Write Section 2: LiDAR Simulation (ray-casting parameters, point cloud generation) in ch3-sensor-simulation.md
- [ ] T067 [US3] Write Section 3: Depth Camera Configuration (projection models, depth image encoding) in ch3-sensor-simulation.md
- [ ] T068 [US3] Write Section 4: IMU Sensor Setup (orientation quaternions, angular velocity, linear acceleration) in ch3-sensor-simulation.md
- [ ] T069 [US3] Write Section 5: Noise Models (Gaussian noise, bias drift, update rate) in ch3-sensor-simulation.md
- [ ] T070 [US3] Write Section 6: Sensor Fusion Basics (multi-sensor integration, timestamp synchronization) in ch3-sensor-simulation.md
- [ ] T071 [US3] Add Learning Objectives, Prerequisites, Summary, and Exercises sections per FR-015 to ch3-sensor-simulation.md

### Gazebo Sensor Examples for User Story 3

- [ ] T072 [P] [US3] Create sensor_playground.sdf (Gazebo world with sensor platform) in docs/module-2-digital-twin/assets/code/worlds/sensor_playground.sdf
- [ ] T073 [P] [US3] Create lidar_sensor.sdf (LiDAR plugin configuration with range and resolution) in docs/module-2-digital-twin/assets/code/models/sensor_platform/lidar_sensor.sdf
- [ ] T074 [P] [US3] Create depth_camera_sensor.sdf (Depth camera plugin configuration) in docs/module-2-digital-twin/assets/code/models/sensor_platform/depth_camera_sensor.sdf
- [ ] T075 [P] [US3] Create imu_sensor.sdf (IMU plugin configuration) in docs/module-2-digital-twin/assets/code/models/sensor_platform/imu_sensor.sdf
- [ ] T076 [US3] Integrate sensor plugins into sensor_platform model with proper mounting frames

### Unity Sensor Examples for User Story 3

- [ ] T077 [P] [US3] Create DepthCameraUnity.cs (Unity C# script for depth camera using Perception package) in Assets/Scripts/DepthCameraUnity.cs
- [ ] T078 [US3] Configure Unity Perception package depth camera in OfficeEnvironment.unity scene
- [ ] T079 [US3] Set up depth camera resolution (640x480) and FOV per Intel RealSense D435 specs

### ROS 2 Sensor Configuration for User Story 3

- [ ] T080 [P] [US3] Create imu_params.yaml (ROS 2 parameter file for IMU noise configuration) in docs/module-2-digital-twin/assets/code/config/imu_params.yaml
- [ ] T081 [P] [US3] Create sensor_params.yaml (ROS 2 parameter file for LiDAR/depth camera settings) in docs/module-2-digital-twin/assets/code/config/sensor_params.yaml
- [ ] T082 [US3] Create sensor_demo.launch.py (ROS 2 launch file for sensor simulation) in docs/module-2-digital-twin/assets/code/launch/sensor_demo.launch.py
- [ ] T083 [US3] Configure noise models in sensor_params.yaml: Gaussian noise (mean=0, stddev per datasheets), bias drift

### RViz2 Visualization for User Story 3

- [ ] T084 [US3] Create sensor_viz.rviz (RViz2 config for sensor data visualization) in docs/module-2-digital-twin/assets/code/config/sensor_viz.rviz
- [ ] T085 [US3] Add PointCloud2 display plugin for LiDAR in sensor_viz.rviz
- [ ] T086 [P] [US3] Add DepthCloud display plugin for depth camera in sensor_viz.rviz
- [ ] T087 [P] [US3] Add IMU display plugin for orientation visualization in sensor_viz.rviz
- [ ] T088 [US3] Configure fixed frame and topic mappings in sensor_viz.rviz

### Helper Scripts for User Story 3

- [ ] T089 [P] [US3] Create test_sensors.py (Python script to subscribe and validate sensor data) in docs/module-2-digital-twin/assets/code/scripts/test_sensors.py
- [ ] T090 [US3] Implement sensor data validation logic: Check update rates, data format, range limits

### Validation for User Story 3

- [ ] T091 [US3] Validate all sensor SDF files with gz sdf --check for sensor_playground.sdf and sensor plugin configs
- [ ] T092 [US3] Validate YAML files with yamllint for imu_params.yaml and sensor_params.yaml
- [ ] T093 [US3] Validate Python scripts with validate_examples.py for sensor_demo.launch.py and test_sensors.py
- [ ] T094 [US3] Test Gazebo sensor simulation: Launch sensor_playground.sdf, verify sensor topics publish data
- [ ] T095 [US3] Test Unity depth camera: Launch Unity scene with Perception, verify depth image output
- [ ] T096 [US3] Test RViz2 visualization: Open sensor_viz.rviz, verify PointCloud2, depth, and IMU displays
- [ ] T097 [US3] Test acceptance scenario: Launch sensor_demo.launch.py with multiple sensors, verify >= 30 FPS simulation speed
- [ ] T098 [US3] Verify sensor data accuracy: Compare LiDAR range to configured values (within 10% per SC-006)
- [ ] T099 [US3] Embed APA citations to sensor datasheets (Velodyne VLP-16, Intel RealSense D435, Bosch BMI088) per FR-010

**Checkpoint**: All user stories should now be independently functional - complete Module 2 with physics, rendering, and sensors

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple chapters and final integration

### Documentation Updates

- [ ] T100 [P] Update docs/intro.md to reference Module 2 chapters
- [ ] T101 [P] Create Module 2 overview/landing page if needed
- [ ] T102 [P] Add cross-references between chapters (e.g., Ch1 references Ch2 for Unity integration)

### Code Quality and Validation

- [ ] T103 Run complete validation suite: npm run build for Docusaurus
- [ ] T104 [P] Verify all markdown files with markdownlint
- [ ] T105 [P] Run validate_all.py script to check all code examples across 3 chapters
- [ ] T106 Check for broken links within Module 2 documentation
- [ ] T107 Verify all code examples include inline comments per FR-009

### Performance and Optimization

- [ ] T108 Test all Gazebo worlds on reference hardware (8GB RAM, integrated GPU), verify RTF >= 1.0
- [ ] T109 [P] Test Unity scenes on reference hardware, verify >= 30 FPS in Editor
- [ ] T110 [P] Optimize any assets exceeding size limits (Unity scenes <500MB)
- [ ] T111 Add performance optimization guidelines to troubleshooting sections per FR-014

### Educational Quality

- [ ] T112 Verify all chapters have Learning Objectives and Prerequisites sections per FR-015
- [ ] T113 [P] Verify all chapters include troubleshooting sections per FR-009
- [ ] T114 [P] Verify all chapters include exercises aligned with learning objectives
- [ ] T115 Check that content is accessible to students who completed Module 1 (no gaps in prerequisites)

### Final Validation

- [ ] T116 Run complete end-to-end test: Follow quickstart.md for Module 2, verify all steps work
- [ ] T117 Verify constitution compliance: Technical accuracy, reproducibility, educational clarity, APA citations
- [ ] T118 Update sidebars.js if any chapter titles or IDs changed during implementation
- [ ] T119 Create deployment checklist based on specs/002-digital-twin/checklists/requirements.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Independent of US1 (but references humanoid model from Foundational)
  - User Story 3 (P3): Can start after Foundational - Independent of US1/US2 (but uses models from Foundational)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1) - Gazebo Physics**: Can start after Foundational (Phase 2) - Fully independent
- **User Story 2 (P2) - Unity Rendering**: Can start after Foundational (Phase 2) - Uses humanoid model but otherwise independent
- **User Story 3 (P3) - Virtual Sensors**: Can start after Foundational (Phase 2) - Uses sensor platform but otherwise independent

### Within Each User Story

- Chapter content before code examples (understanding before implementation)
- Code examples before validation (create before test)
- All SDF/Python files before integration testing
- Validation before checkpoint

### Parallel Opportunities

**Phase 1 - Setup**:
- T003 (Unity directory) || T004 (Contract templates) - different directories

**Phase 2 - Foundational**:
- T007 (model.config simple_biped) || T009 (model.config sensor_platform) - different files
- T008 (simple_biped SDF) || T010 (RViz config) - different files

**Phase 3 - User Story 1**:
- T021 (empty_world.sdf) || T022 (physics_demo.sdf) || T023 (collision_test.sdf) - different world files
- T025 (launch file) || T026 (spawn script) - different files

**Phase 4 - User Story 2**:
- T043 (URDF Importer) || T044 (Perception package) - different Unity packages
- T046 (OfficeEnvironment) || T047 (FactoryFloor) - different Unity scenes
- T051 (ROSConnection.cs) || T052 (JointStatePublisher.cs) - different scripts

**Phase 5 - User Story 3**:
- T073 (lidar_sensor.sdf) || T074 (depth_camera_sensor.sdf) || T075 (imu_sensor.sdf) - different sensor files
- T080 (imu_params.yaml) || T081 (sensor_params.yaml) - different config files
- T086 (DepthCloud display) || T087 (IMU display) - different RViz plugins

**Phase 6 - Polish**:
- T100 (intro.md) || T101 (overview) || T102 (cross-refs) - different docs
- T104 (markdownlint) || T105 (validate_all.py) || T106 (broken links) - different validation tools
- T109 (Unity FPS) || T110 (asset optimization) - different platforms

---

## Parallel Example: User Story 1 (Gazebo Physics)

```bash
# Launch all Gazebo world file creation tasks together:
Task T021: "Create empty_world.sdf in docs/module-2-digital-twin/assets/code/worlds/empty_world.sdf"
Task T022: "Create physics_demo.sdf in docs/module-2-digital-twin/assets/code/worlds/physics_demo.sdf"
Task T023: "Create collision_test.sdf in docs/module-2-digital-twin/assets/code/worlds/collision_test.sdf"

# Launch Python support scripts together:
Task T025: "Create gazebo_physics.launch.py in docs/module-2-digital-twin/assets/code/launch/gazebo_physics.launch.py"
Task T026: "Create spawn_robot.py in docs/module-2-digital-twin/assets/code/scripts/spawn_robot.py"
```

## Parallel Example: User Story 2 (Unity Rendering)

```bash
# Launch Unity scene creation tasks together:
Task T046: "Create OfficeEnvironment.unity scene in Assets/Scenes/"
Task T047: "Create FactoryFloor.unity scene in Assets/Scenes/"

# Launch Unity C# scripts together:
Task T051: "Create ROSConnection.cs in Assets/Scripts/ROSConnection.cs"
Task T052: "Create JointStatePublisher.cs in Assets/Scripts/JointStatePublisher.cs"
```

## Parallel Example: User Story 3 (Virtual Sensors)

```bash
# Launch all sensor configuration files together:
Task T073: "Create lidar_sensor.sdf"
Task T074: "Create depth_camera_sensor.sdf"
Task T075: "Create imu_sensor.sdf"

# Launch RViz display plugins together:
Task T086: "Add DepthCloud display plugin in sensor_viz.rviz"
Task T087: "Add IMU display plugin in sensor_viz.rviz"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T012) - CRITICAL
3. Complete Phase 3: User Story 1 (T013-T032)
4. **STOP and VALIDATE**: Test Chapter 1 independently with Gazebo simulations
5. Deploy/demo Module 2 with Chapter 1 only

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (Chapter 1: Gazebo Physics) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (Chapter 2: Unity Rendering) → Test independently → Deploy/Demo
4. Add User Story 3 (Chapter 3: Virtual Sensors) → Test independently → Deploy/Demo
5. Each chapter adds value without breaking previous chapters

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: User Story 1 (Gazebo Physics chapter + examples)
   - Author B: User Story 2 (Unity Rendering chapter + scenes)
   - Author C: User Story 3 (Virtual Sensors chapter + configs)
3. Chapters complete and integrate independently

---

## Task Summary

**Total Tasks**: 119 tasks

**By Phase**:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 7 tasks
- Phase 3 (User Story 1 - Gazebo): 20 tasks
- Phase 4 (User Story 2 - Unity): 31 tasks
- Phase 5 (User Story 3 - Sensors): 36 tasks
- Phase 6 (Polish): 20 tasks

**By User Story**:
- User Story 1 (P1 - Gazebo Physics): 20 tasks
- User Story 2 (P2 - Unity Rendering): 31 tasks
- User Story 3 (P3 - Virtual Sensors): 36 tasks
- Infrastructure (Setup + Foundational + Polish): 32 tasks

**Parallel Opportunities**: 47 tasks marked [P] - can be executed concurrently with other tasks in same phase

**MVP Scope** (User Story 1 only): 32 tasks (Setup + Foundational + US1)

**Independent Test Criteria**:
- US1: Launch all 4 Gazebo worlds, verify RTF >= 1.0, no errors, physics behaves correctly
- US2: Open Unity scenes, establish ROS 2 connection, verify real-time robot rendering at >= 30 FPS
- US3: Launch sensor simulation, visualize in RViz2, verify sensor data accuracy within 10%

---

## Notes

- [P] tasks = different files, no dependencies within same phase
- [Story] label (US1, US2, US3) maps task to specific chapter for traceability
- Each user story (chapter) should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate chapter independently
- All SDF files validated with `gz sdf --check`
- All Python files validated with `validate_examples.py`
- All YAML files validated with `yamllint`
- All markdown validated with `markdownlint`
- Constitution compliance verified in Phase 6
- APA citations required for all external references (Gazebo docs, Unity docs, sensor datasheets)
