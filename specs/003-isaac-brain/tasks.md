# Tasks: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Input**: Design documents from `/specs/003-isaac-brain/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅

**Tests**: Tests are NOT explicitly requested in the specification. This task list focuses on educational content creation and code example development.

**Organization**: Tasks are grouped by user story (P1-P5) to enable independent implementation and testing of each chapter. Each phase represents a complete, independently deliverable chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5)
- Include exact file paths in descriptions

## Path Conventions

This is a **Docusaurus educational content project**:
- **Content**: `docs/module-3-isaac/` (Markdown chapters)
- **Code Examples**: `docs/module-3-isaac/assets/code/` (Python scripts, launch files, config files)
- **Robot Models**: `docs/module-3-isaac/assets/models/` (USD humanoid models)
- **Images**: `static/img/module-3/` (screenshots, diagrams)
- **Planning**: `specs/003-isaac-brain/` (not published)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create directory structure and foundational assets for Module 3

- [ ] T001 Create `docs/module-3-isaac/` directory structure
- [ ] T002 [P] Create `docs/module-3-isaac/assets/code/isaac_sim/` directory
- [ ] T003 [P] Create `docs/module-3-isaac/assets/code/isaac_ros/` directory
- [ ] T004 [P] Create `docs/module-3-isaac/assets/code/nav2/` directory
- [ ] T005 [P] Create `docs/module-3-isaac/assets/code/examples/` directory
- [ ] T006 [P] Create `docs/module-3-isaac/assets/models/` directory for USD robot models
- [ ] T007 [P] Create `docs/module-3-isaac/assets/diagrams/` directory for Mermaid source files
- [ ] T008 [P] Create `static/img/module-3/` directory for screenshots and exported diagrams
- [ ] T009 Create Python validation script `scripts/validate_module3_examples.py` for code quality checks
- [ ] T010 Update `sidebars.js` to include Module 3 navigation structure with 6 chapters

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create USD robot models and validation infrastructure that ALL chapters depend on

**⚠️ CRITICAL**: No chapter writing can begin until these foundational assets are complete

- [ ] T011 Create base USD humanoid model in `docs/module-3-isaac/assets/models/humanoid_base.usda` (torso, head, arms, legs with basic physics)
- [ ] T012 Create stereo camera USD humanoid in `docs/module-3-isaac/assets/models/humanoid_stereo_cameras.usda` (base + stereo camera pair on head)
- [ ] T013 Create full sensor USD humanoid in `docs/module-3-isaac/assets/models/humanoid_full_sensors.usda` (base + stereo cameras + IMU + odometry)
- [ ] T014 Validate all USD models with Isaac Sim USD format checker
- [ ] T015 [P] Create validation rules in `specs/003-isaac-brain/contracts/chapter-schema.json` (frontmatter fields: title, description, learning_objectives, prerequisites)
- [ ] T016 [P] Create code example schema in `specs/003-isaac-brain/contracts/code-example-schema.json` (structure: filename, language, description, hardware_requirements, expected_output)
- [ ] T017 [P] Create Mermaid diagram standards in `specs/003-isaac-brain/contracts/mermaid-diagram-schema.json` (required elements, style guide)
- [ ] T018 Configure Python syntax validator in `scripts/validate_module3_examples.py` to check all `.py` files in `docs/module-3-isaac/assets/code/`
- [ ] T019 Configure YAML schema validator in `scripts/validate_module3_examples.py` to check ROS 2 config files
- [ ] T020 Test validation script on sample code example to confirm all checks work

**Checkpoint**: Foundation ready - chapter writing and code examples can now proceed in parallel

---

## Phase 3: User Story 1 - Understanding Isaac Sim Basics (Priority: P1) 🎯 MVP

**Goal**: Students can set up Isaac Sim, load humanoid robots, run simulations, and capture synthetic sensor data

**Independent Test**: Student launches Isaac Sim, loads `humanoid_stereo_cameras.usda`, runs 10-second simulation, exports 100+ RGB/depth frames

**Success Criteria** (from spec.md):
- SC-001: Launch Isaac Sim and run basic simulation within 30 minutes
- SC-002: 90% of students generate 100+ frame synthetic dataset

### Implementation for User Story 1 (Chapter 1)

#### Chapter Content

- [ ] T021 [P] [US1] Write Chapter 1 introduction in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (overview, learning objectives, prerequisites from Modules 1-2)
- [ ] T022 [P] [US1] Write Section 1.1: Isaac Sim Installation in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (Ubuntu 22.04, GPU requirements, CUDA/cuDNN, version pinning to 2024.1)
- [ ] T023 [P] [US1] Write Section 1.2: Isaac Sim UI Tour in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (viewport, scene hierarchy, property panel, simulation controls)
- [ ] T024 [P] [US1] Write Section 1.3: Loading USD Robot Models in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (File → Import workflow, physics properties verification, sensor configuration)
- [ ] T025 [P] [US1] Write Section 1.4: Running Basic Simulation in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (play/pause/stop controls, timestep configuration, real-time factor)
- [ ] T026 [P] [US1] Write Section 1.5: ROS 2 Bridge Setup in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (enable ROS 2 extension, configure topic publishing, clock synchronization)
- [ ] T027 [P] [US1] Write Section 1.6: Capturing Synthetic Sensor Data in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (RGB images, depth maps, export workflow, data formats)
- [ ] T028 [P] [US1] Write Section 1.7: Troubleshooting Common Issues in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (GPU driver errors, CUDA out of memory, physics instability, from research.md Section 5.1)
- [ ] T029 [P] [US1] Write Chapter 1 exercises in `docs/module-3-isaac/ch1-isaac-sim-basics.md` (3-5 hands-on challenges with solution hints)
- [ ] T030 [P] [US1] Write Chapter 1 summary and next steps in `docs/module-3-isaac/ch1-isaac-sim-basics.md`

#### Code Examples for User Story 1

- [ ] T031 [P] [US1] Create `launch_isaac_sim.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (Python script to programmatically launch Isaac Sim)
- [ ] T032 [P] [US1] Create `load_humanoid_model.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (load USD model, configure physics, verify sensors)
- [ ] T033 [P] [US1] Create `ros2_bridge_setup.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (enable ROS 2 bridge, configure topic publishing, test connectivity)
- [ ] T034 [P] [US1] Create `capture_rgb_depth.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (capture RGB and depth from stereo cameras, save to disk as PNG/NumPy)
- [ ] T035 [P] [US1] Create `synthetic_data_pipeline.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (automated synthetic data generation: 100 frames with RGB, depth, segmentation)
- [ ] T036 [P] [US1] Create `verify_sensor_topics.sh` in `docs/module-3-isaac/assets/code/isaac_sim/` (Bash script to verify ROS 2 topics are publishing: `ros2 topic list`, `ros2 topic hz`)
- [ ] T037 [P] [US1] Create `gpu_memory_monitor.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (monitor VRAM usage during simulation, warn if approaching limit)

#### Diagrams for User Story 1

- [ ] T038 [P] [US1] Create Mermaid diagram in `docs/module-3-isaac/ch1-isaac-sim-basics.md` showing Isaac Sim architecture (Omniverse, PhysX, RTX, ROS 2 Bridge, USD)
- [ ] T039 [P] [US1] Create Mermaid diagram in `docs/module-3-isaac/ch1-isaac-sim-basics.md` showing simulation setup workflow (Environment Prep → ROS 2 Integration → Testing, from research.md Section 1.3)
- [ ] T040 [P] [US1] Create Mermaid diagram in `docs/module-3-isaac/ch1-isaac-sim-basics.md` showing ROS 2 data flow (Isaac Sim Sensors → ROS 2 Topics → External Nodes)

#### Screenshots for User Story 1

- [ ] T041 [P] [US1] Capture screenshot of Isaac Sim default scene in `static/img/module-3/isaac-sim-default-scene.png`
- [ ] T042 [P] [US1] Capture screenshot of humanoid model loaded in viewport in `static/img/module-3/humanoid-loaded.png`
- [ ] T043 [P] [US1] Capture screenshot of ROS 2 Bridge configuration panel in `static/img/module-3/ros2-bridge-config.png`
- [ ] T044 [P] [US1] Capture screenshot of synthetic RGB and depth images side-by-side in `static/img/module-3/rgb-depth-sample.png`

#### Validation for User Story 1

- [ ] T045 [US1] Run validation script on all Chapter 1 code examples: `python scripts/validate_module3_examples.py --chapter 1`
- [ ] T046 [US1] Test all Chapter 1 code examples on Docker container (Ubuntu 22.04 + ROS 2 Humble + Isaac Sim)
- [ ] T047 [US1] Verify Chapter 1 frontmatter matches schema in `contracts/chapter-schema.json`
- [ ] T048 [US1] Build Docusaurus site and verify Chapter 1 renders correctly: `npm run build && npm run serve`
- [ ] T049 [US1] Manual review: Complete Chapter 1 as a student would, generate 100 frame dataset within 30 minutes (SC-001, SC-002 validation)

**Checkpoint**: Chapter 1 complete and independently testable. Students can now set up Isaac Sim and generate synthetic data.

---

## Phase 4: User Story 2 - Implementing VSLAM with Isaac ROS (Priority: P2)

**Goal**: Students implement Visual SLAM using Isaac ROS GEMs on simulated humanoid robot to build maps and localize

**Independent Test**: Student configures Isaac ROS VSLAM, teleoperates robot through environment, generates occupancy grid map with trajectory

**Success Criteria** (from spec.md):
- SC-003: Configure Isaac ROS VSLAM and produce valid map within 1 hour

### Implementation for User Story 2 (Chapter 3)

**Note**: Chapter 2 (Synthetic Data Generation) will be developed after Chapter 1. Chapter 3 (VSLAM) is prioritized here as P2 user story.

#### Chapter Content

- [ ] T050 [P] [US2] Write Chapter 3 introduction in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (VSLAM overview, GPU acceleration benefits, chapter goals)
- [ ] T051 [P] [US2] Write Section 3.1: VSLAM Fundamentals in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (feature extraction, tracking, mapping, loop closure from research.md FR-005)
- [ ] T052 [P] [US2] Write Section 3.2: Isaac ROS Visual SLAM Package in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (isaac_ros_visual_slam overview, GPU acceleration architecture, inputs/outputs)
- [ ] T053 [P] [US2] Write Section 3.3: Configuring Stereo Cameras in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (stereo camera calibration, topic publishing, frame synchronization)
- [ ] T054 [P] [US2] Write Section 3.4: Running Isaac ROS VSLAM in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (launch file configuration, parameter tuning, GPU memory management)
- [ ] T055 [P] [US2] Write Section 3.5: Teleoperation and Mapping in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (keyboard teleop, path planning, map building workflow)
- [ ] T056 [P] [US2] Write Section 3.6: Visualizing VSLAM Output in RViz2 in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (odometry, trajectory, occupancy grid, TF tree from research.md FR-006)
- [ ] T057 [P] [US2] Write Section 3.7: Saving and Reloading Maps in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (map serialization, localization in known maps)
- [ ] T058 [P] [US2] Write Section 3.8: Performance Analysis in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (GPU vs CPU benchmarks, frame rate analysis, from research.md FR-012)
- [ ] T059 [P] [US2] Write Section 3.9: Troubleshooting VSLAM in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` (featureless environments, node crashes, map artifacts from research.md Section 5.3)
- [ ] T060 [P] [US2] Write Chapter 3 exercises and summary in `docs/module-3-isaac/ch3-vslam-isaac-ros.md`

#### Code Examples for User Story 2

- [ ] T061 [P] [US2] Create `isaac_ros_vslam_setup.sh` in `docs/module-3-isaac/assets/code/isaac_ros/` (install Isaac ROS VSLAM packages via apt or Docker)
- [ ] T062 [P] [US2] Create `stereo_camera_config.yaml` in `docs/module-3-isaac/assets/code/isaac_ros/` (camera calibration parameters, topic remapping, frame IDs)
- [ ] T063 [P] [US2] Create `vslam_launch.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (ROS 2 launch file for Isaac ROS VSLAM with stereo camera inputs)
- [ ] T064 [P] [US2] Create `vslam_params.yaml` in `docs/module-3-isaac/assets/code/isaac_ros/` (VSLAM parameters: feature detector, tracking thresholds, loop closure)
- [ ] T065 [P] [US2] Create `teleop_humanoid.py` in `docs/module-3-isaac/assets/code/examples/` (keyboard teleoperation script for humanoid robot in Isaac Sim)
- [ ] T066 [P] [US2] Create `visualize_vslam_rviz.rviz` in `docs/module-3-isaac/assets/code/isaac_ros/` (RViz2 configuration file for VSLAM visualization: odometry, trajectory, map)
- [ ] T067 [P] [US2] Create `save_vslam_map.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (save occupancy grid map to .pgm/.yaml format)
- [ ] T068 [P] [US2] Create `reload_map_localize.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (reload saved map, localize robot within it)
- [ ] T069 [P] [US2] Create `benchmark_vslam_performance.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (measure frame rate, GPU memory usage, compare to CPU baseline)

#### Diagrams for User Story 2

- [ ] T070 [P] [US2] Create Mermaid diagram in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` showing VSLAM pipeline architecture (Feature Extraction → Tracking → Mapping → Loop Closure)
- [ ] T071 [P] [US2] Create Mermaid diagram in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` showing Isaac ROS VSLAM data flow (Stereo Cameras → VSLAM Node → Odometry/Map Topics → RViz2)
- [ ] T072 [P] [US2] Create Mermaid diagram in `docs/module-3-isaac/ch3-vslam-isaac-ros.md` showing TF tree for VSLAM (map → odom → base_link → camera_left/camera_right)

#### Screenshots for User Story 2

- [ ] T073 [P] [US2] Capture screenshot of RViz2 showing VSLAM trajectory and map in `static/img/module-3/vslam-trajectory-map.png`
- [ ] T074 [P] [US2] Capture screenshot of VSLAM node output showing GPU acceleration in `static/img/module-3/vslam-gpu-stats.png`
- [ ] T075 [P] [US2] Capture screenshot of feature tracking visualization in `static/img/module-3/vslam-feature-tracking.png`

#### Validation for User Story 2

- [ ] T076 [US2] Run validation script on all Chapter 3 code examples: `python scripts/validate_module3_examples.py --chapter 3`
- [ ] T077 [US2] Test all Chapter 3 code examples on Docker container with Isaac ROS installed
- [ ] T078 [US2] Verify Chapter 3 frontmatter matches schema
- [ ] T079 [US2] Build Docusaurus site and verify Chapter 3 renders correctly
- [ ] T080 [US2] Manual review: Complete Chapter 3 as student, generate valid occupancy map within 1 hour (SC-003 validation)

**Checkpoint**: Chapter 3 complete. Students can now implement VSLAM and build maps independently.

---

## Phase 5: User Story 3 - Depth Perception and 3D Mapping (Priority: P3)

**Goal**: Students process depth sensor data to create 3D point cloud maps for complex 3D environments

**Independent Test**: Student configures depth sensors, runs stereo disparity node, generates 3D point cloud, visualizes in RViz2

### Implementation for User Story 3 (Chapter 4)

#### Chapter Content

- [ ] T081 [P] [US3] Write Chapter 4 introduction in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (depth perception overview, 3D navigation importance for bipeds)
- [ ] T082 [P] [US3] Write Section 4.1: Depth Sensing Technologies in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (stereo cameras, time-of-flight, LiDAR comparison from research.md FR-008)
- [ ] T083 [P] [US3] Write Section 4.2: Stereo Disparity Fundamentals in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (disparity map calculation, depth computation, calibration importance)
- [ ] T084 [P] [US3] Write Section 4.3: Isaac ROS Stereo Image Proc in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (isaac_ros_stereo_image_proc package, GPU acceleration, inputs/outputs)
- [ ] T085 [P] [US3] Write Section 4.4: Generating 3D Point Clouds in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (depth → 3D conversion, point cloud messages, PCL integration)
- [ ] T086 [P] [US3] Write Section 4.5: Point Cloud Filtering and Downsampling in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (voxel grid filter, statistical outlier removal, real-time optimization)
- [ ] T087 [P] [US3] Write Section 4.6: Visualizing Point Clouds in RViz2 in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (PointCloud2 display, color mapping, 3D navigation)
- [ ] T088 [P] [US3] Write Section 4.7: 3D Mapping for Stairs and Uneven Terrain in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (height-aware navigation, multi-level environments)
- [ ] T089 [P] [US3] Write Section 4.8: Troubleshooting Depth Perception in `docs/module-3-isaac/ch4-depth-perception-mapping.md` (sensor synchronization errors, depth accuracy issues from research.md)
- [ ] T090 [P] [US3] Write Chapter 4 exercises and summary in `docs/module-3-isaac/ch4-depth-perception-mapping.md`

#### Code Examples for User Story 3

- [ ] T091 [P] [US3] Create `stereo_disparity_launch.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (launch Isaac ROS stereo disparity node with stereo camera inputs)
- [ ] T092 [P] [US3] Create `stereo_params.yaml` in `docs/module-3-isaac/assets/code/isaac_ros/` (stereo matching parameters, disparity range, uniqueness ratio)
- [ ] T093 [P] [US3] Create `depth_to_pointcloud.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (convert depth image to PointCloud2 message, publish to ROS 2 topic)
- [ ] T094 [P] [US3] Create `pointcloud_filter.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (apply voxel grid filter and statistical outlier removal to point cloud)
- [ ] T095 [P] [US3] Create `visualize_pointcloud_rviz.rviz` in `docs/module-3-isaac/assets/code/isaac_ros/` (RViz2 config for 3D point cloud visualization)
- [ ] T096 [P] [US3] Create `save_pointcloud.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (save point cloud to PCD file format for offline analysis)
- [ ] T097 [P] [US3] Create `benchmark_depth_performance.py` in `docs/module-3-isaac/assets/code/isaac_ros/` (measure stereo disparity frame rate, GPU memory usage)

#### Diagrams for User Story 3

- [ ] T098 [P] [US3] Create Mermaid diagram in `docs/module-3-isaac/ch4-depth-perception-mapping.md` showing stereo depth pipeline (Left/Right Images → Stereo Disparity → Depth Image → Point Cloud)
- [ ] T099 [P] [US3] Create Mermaid diagram in `docs/module-3-isaac/ch4-depth-perception-mapping.md` showing depth sensor comparison table (stereo vs ToF vs LiDAR)
- [ ] T100 [P] [US3] Create Mermaid diagram in `docs/module-3-isaac/ch4-depth-perception-mapping.md` showing point cloud processing pipeline (Raw Cloud → Filtering → Downsampling → Optimized Cloud)

#### Screenshots for User Story 3

- [ ] T101 [P] [US3] Capture screenshot of depth image visualization in `static/img/module-3/depth-image-sample.png`
- [ ] T102 [P] [US3] Capture screenshot of 3D point cloud in RViz2 showing stairs/terrain in `static/img/module-3/pointcloud-3d-environment.png`
- [ ] T103 [P] [US3] Capture screenshot of point cloud before/after filtering in `static/img/module-3/pointcloud-filtering.png`

#### Validation for User Story 3

- [ ] T104 [US3] Run validation script on all Chapter 4 code examples: `python scripts/validate_module3_examples.py --chapter 4`
- [ ] T105 [US3] Test all Chapter 4 code examples on Docker container
- [ ] T106 [US3] Verify Chapter 4 frontmatter matches schema
- [ ] T107 [US3] Build Docusaurus site and verify Chapter 4 renders correctly
- [ ] T108 [US3] Manual review: Complete Chapter 4 as student, generate 3D point cloud and visualize in RViz2

**Checkpoint**: Chapter 4 complete. Students can now process depth data and create 3D maps independently.

---

## Phase 6: User Story 4 - Nav2 Integration for Bipedal Navigation (Priority: P4)

**Goal**: Students integrate Isaac ROS perception with Nav2 path planning for autonomous bipedal humanoid navigation

**Independent Test**: Student configures Nav2 with Isaac ROS inputs, sends navigation goals, robot reaches goal in 8/10 trials (±0.2m tolerance)

**Success Criteria** (from spec.md):
- SC-004: 85% success rate for Nav2 autonomous navigation with Isaac ROS perception

### Implementation for User Story 4 (Chapter 5)

#### Chapter Content

- [ ] T109 [P] [US4] Write Chapter 5 introduction in `docs/module-3-isaac/ch5-nav2-integration.md` (Nav2 overview, bipedal navigation challenges)
- [ ] T110 [P] [US4] Write Section 5.1: Nav2 Architecture in `docs/module-3-isaac/ch5-nav2-integration.md` (planners, controllers, costmaps, behavior trees from research.md Section 1)
- [ ] T111 [P] [US4] Write Section 5.2: Costmap Configuration in `docs/module-3-isaac/ch5-nav2-integration.md` (static layer, obstacle layer, inflation layer from research.md Section 3)
- [ ] T112 [P] [US4] Write Section 5.3: Integrating VSLAM and Depth Perception in `docs/module-3-isaac/ch5-nav2-integration.md` (perception → costmap data flow, sensor fusion)
- [ ] T113 [P] [US4] Write Section 5.4: Humanoid-Specific Parameters in `docs/module-3-isaac/ch5-nav2-integration.md` (velocity limits 0.3-0.5 m/s, footprint, TEB controller from research.md Section 2)
- [ ] T114 [P] [US4] Write Section 5.5: Launching Nav2 Stack in `docs/module-3-isaac/ch5-nav2-integration.md` (launch file structure, parameter configuration, initialization checks)
- [ ] T115 [P] [US4] Write Section 5.6: Sending Navigation Goals in `docs/module-3-isaac/ch5-nav2-integration.md` (RViz2 2D Nav Goal tool, programmatic goal sending, monitoring status from research.md FR-010)
- [ ] T116 [P] [US4] Write Section 5.7: Behavior Trees and Recovery in `docs/module-3-isaac/ch5-nav2-integration.md` (behavior tree structure, bipedal recovery behaviors from research.md FR-011)
- [ ] T117 [P] [US4] Write Section 5.8: Tuning Nav2 for Humanoids in `docs/module-3-isaac/ch5-nav2-integration.md` (parameter optimization, cost function weights, balance considerations)
- [ ] T118 [P] [US4] Write Section 5.9: Troubleshooting Navigation in `docs/module-3-isaac/ch5-nav2-integration.md` (TF errors, costmap freezing, oscillation from research.md Section 5.3)
- [ ] T119 [P] [US4] Write Chapter 5 exercises and summary in `docs/module-3-isaac/ch5-nav2-integration.md`

#### Code Examples for User Story 4

- [ ] T120 [P] [US4] Create `nav2_setup.sh` in `docs/module-3-isaac/assets/code/nav2/` (install Nav2 packages, dependencies)
- [ ] T121 [P] [US4] Create `costmap_common_params.yaml` in `docs/module-3-isaac/assets/code/nav2/` (global costmap parameters: resolution, dimensions, plugins)
- [ ] T122 [P] [US4] Create `global_costmap_params.yaml` in `docs/module-3-isaac/assets/code/nav2/` (static layer, obstacle layer config from research.md Section 3.1)
- [ ] T123 [P] [US4] Create `local_costmap_params.yaml` in `docs/module-3-isaac/assets/code/nav2/` (local costmap for dynamic obstacles)
- [ ] T124 [P] [US4] Create `planner_params.yaml` in `docs/module-3-isaac/assets/code/nav2/` (Smac or TEB planner configuration)
- [ ] T125 [P] [US4] Create `controller_params.yaml` in `docs/module-3-isaac/assets/code/nav2/` (DWA or TEB controller with humanoid velocity limits from research.md Section 2.2)
- [ ] T126 [P] [US4] Create `behavior_tree.xml` in `docs/module-3-isaac/assets/code/nav2/` (custom behavior tree for bipedal recovery)
- [ ] T127 [P] [US4] Create `nav2_launch.py` in `docs/module-3-isaac/assets/code/nav2/` (launch Nav2 stack with Isaac ROS perception inputs)
- [ ] T128 [P] [US4] Create `send_navigation_goal.py` in `docs/module-3-isaac/assets/code/nav2/` (programmatically send 2D navigation goal, monitor status)
- [ ] T129 [P] [US4] Create `monitor_nav_performance.py` in `docs/module-3-isaac/assets/code/nav2/` (track success rate, goal tolerance, time to goal)

#### Diagrams for User Story 4

- [ ] T130 [P] [US4] Create Mermaid diagram in `docs/module-3-isaac/ch5-nav2-integration.md` showing Nav2 architecture (Costmap → Global Planner → Local Controller → Behavior Tree)
- [ ] T131 [P] [US4] Create Mermaid diagram in `docs/module-3-isaac/ch5-nav2-integration.md` showing perception → Nav2 data flow (VSLAM/Depth → Costmap → Planner → cmd_vel)
- [ ] T132 [P] [US4] Create Mermaid diagram in `docs/module-3-isaac/ch5-nav2-integration.md` showing behavior tree for bipedal navigation (NavigateToPose → ComputePath → FollowPath → Recovery)

#### Screenshots for User Story 4

- [ ] T133 [P] [US4] Capture screenshot of Nav2 costmap in RViz2 showing obstacles from perception in `static/img/module-3/nav2-costmap-obstacles.png`
- [ ] T134 [P] [US4] Capture screenshot of planned path visualization in `static/img/module-3/nav2-planned-path.png`
- [ ] T135 [P] [US4] Capture screenshot of humanoid robot navigating to goal in `static/img/module-3/nav2-autonomous-navigation.png`

#### Validation for User Story 4

- [ ] T136 [US4] Run validation script on all Chapter 5 code examples: `python scripts/validate_module3_examples.py --chapter 5`
- [ ] T137 [US4] Test all Chapter 5 code examples on Docker container
- [ ] T138 [US4] Verify Chapter 5 frontmatter matches schema
- [ ] T139 [US4] Build Docusaurus site and verify Chapter 5 renders correctly
- [ ] T140 [US4] Manual review: Complete Chapter 5 as student, achieve 8/10 navigation goal success (SC-004 validation)

**Checkpoint**: Chapter 5 complete. Students can now deploy autonomous navigation with Nav2 independently.

---

## Phase 7: User Story 5 - End-to-End AI-Robot Brain Workflow (Priority: P5)

**Goal**: Students understand and demonstrate complete perception-to-action pipeline, explain architecture, analyze system performance

**Independent Test**: Student draws architecture diagram, explains data flow, runs end-to-end system, captures logs/metrics, compares GPU vs CPU

**Success Criteria** (from spec.md):
- SC-005: Students can explain end-to-end pipeline with diagram and verbal explanation
- SC-008: 80%+ agree "Module clearly explained GPU acceleration benefits"

### Implementation for User Story 5 (Chapter 6)

#### Chapter Content

- [ ] T141 [P] [US5] Write Chapter 6 introduction in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (capstone overview, synthesis of all concepts)
- [ ] T142 [P] [US5] Write Section 6.1: AI-Robot Brain Architecture in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (complete system diagram: Isaac Sim → Isaac ROS → Nav2 from research.md Section 8.2)
- [ ] T143 [P] [US5] Write Section 6.2: Data Flow Analysis in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (sensors → GEMs → topics → Nav2 → motion commands, TF tree management from research.md FR-016)
- [ ] T144 [P] [US5] Write Section 6.3: Running Complete System in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (launch all components, verify connectivity, troubleshoot integration)
- [ ] T145 [P] [US5] Write Section 6.4: Logging and Metrics Collection in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (capture logs at each stage, verify data transformations)
- [ ] T146 [P] [US5] Write Section 6.5: Performance Analysis in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (GPU vs CPU comparison, frame rates, latency analysis from research.md Section 6.1)
- [ ] T147 [P] [US5] Write Section 6.6: Failure Mode Analysis in `docs/module-3-isaac/ch6-end-to-end-capstone.md` ("what if" scenarios: sensor failure, GPU exhaustion, SLAM drift from spec.md Edge Cases)
- [ ] T148 [P] [US5] Write Section 6.7: Real-World Deployment Considerations in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (sim-to-real transfer, production robotics best practices)
- [ ] T149 [P] [US5] Write Section 6.8: Module 3 Summary in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (key learnings, connections to Module 4-5)
- [ ] T150 [P] [US5] Write Chapter 6 capstone project in `docs/module-3-isaac/ch6-end-to-end-capstone.md` (design custom scenario, implement, document, present)

#### Code Examples for User Story 5

- [ ] T151 [P] [US5] Create `launch_full_pipeline.sh` in `docs/module-3-isaac/assets/code/examples/` (Bash script to launch Isaac Sim + Isaac ROS + Nav2 in correct order)
- [ ] T152 [P] [US5] Create `end_to_end_launch.py` in `docs/module-3-isaac/assets/code/examples/` (ROS 2 launch file for complete system)
- [ ] T153 [P] [US5] Create `verify_pipeline_connectivity.py` in `docs/module-3-isaac/assets/code/examples/` (check all topics, transforms, nodes are running)
- [ ] T154 [P] [US5] Create `capture_system_logs.py` in `docs/module-3-isaac/assets/code/examples/` (log data at each pipeline stage: sensors → perception → navigation)
- [ ] T155 [P] [US5] Create `analyze_performance_metrics.py` in `docs/module-3-isaac/assets/code/examples/` (compute frame rates, latencies, GPU memory usage across pipeline)
- [ ] T156 [P] [US5] Create `autonomous_navigation_demo.py` in `docs/module-3-isaac/assets/code/examples/` (complete demo: load environment, navigate to multiple goals, report metrics from research.md FR-020)
- [ ] T157 [P] [US5] Create `gpu_vs_cpu_comparison.py` in `docs/module-3-isaac/assets/code/examples/` (benchmark Isaac ROS (GPU) vs CPU-based perception for VSLAM and depth from research.md Section 6.1)

#### Diagrams for User Story 5

- [ ] T158 [P] [US5] Create Mermaid diagram in `docs/module-3-isaac/ch6-end-to-end-capstone.md` showing complete AI-Robot Brain architecture (all components, data flows, transforms)
- [ ] T159 [P] [US5] Create Mermaid diagram in `docs/module-3-isaac/ch6-end-to-end-capstone.md` showing failure mode decision tree (sensor fail → recovery, GPU exhausted → fallback)
- [ ] T160 [P] [US5] Create Mermaid diagram in `docs/module-3-isaac/ch6-end-to-end-capstone.md` showing performance comparison table (GPU vs CPU for VSLAM, depth, overall pipeline)

#### Screenshots for User Story 5

- [ ] T161 [P] [US5] Capture screenshot of full system running in Isaac Sim + RViz2 in `static/img/module-3/end-to-end-system.png`
- [ ] T162 [P] [US5] Capture screenshot of system logs showing data flow through pipeline in `static/img/module-3/system-logs-dataflow.png`
- [ ] T163 [P] [US5] Capture screenshot of performance metrics dashboard in `static/img/module-3/performance-metrics.png`

#### Validation for User Story 5

- [ ] T164 [US5] Run validation script on all Chapter 6 code examples: `python scripts/validate_module3_examples.py --chapter 6`
- [ ] T165 [US5] Test all Chapter 6 code examples on Docker container
- [ ] T166 [US5] Verify Chapter 6 frontmatter matches schema
- [ ] T167 [US5] Build Docusaurus site and verify Chapter 6 renders correctly
- [ ] T168 [US5] Manual review: Complete Chapter 6 as student, produce architecture diagram, explain verbally, run end-to-end system (SC-005, SC-008 validation)

**Checkpoint**: Chapter 6 complete. Students can now explain and demonstrate the complete AI-Robot Brain workflow.

---

## Phase 8: User Story 2 Extension - Synthetic Data Generation (Priority: P2 Extension)

**Goal**: Create Chapter 2 content on synthetic data generation (deferred to after Chapters 1, 3-6)

**Independent Test**: Student uses Isaac Sim SDG pipeline to generate labeled dataset (RGB, depth, segmentation) of 1000+ frames

**Note**: This was originally part of US1 but expanded into separate chapter per plan.md Decision 1

### Implementation for Chapter 2

#### Chapter Content

- [ ] T169 [P] [US2 Ext] Write Chapter 2 introduction in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (why synthetic data, applications in ML/training)
- [ ] T170 [P] [US2 Ext] Write Section 2.1: Synthetic Data Types in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (RGB, depth, semantic/instance segmentation, bounding boxes from research.md Section 4.1)
- [ ] T171 [P] [US2 Ext] Write Section 2.2: Isaac Sim SDG Pipeline in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (synthetic data recorder extension, programmable collection)
- [ ] T172 [P] [US2 Ext] Write Section 2.3: Configuring Data Collection in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (sensor types, data formats, randomization strategies)
- [ ] T173 [P] [US2 Ext] Write Section 2.4: Batch Data Generation in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (automated generation of thousands of frames)
- [ ] T174 [P] [US2 Ext] Write Section 2.5: Dataset Formatting in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (COCO, Pascal VOC, Cityscapes formats from research.md Section 4.2.4)
- [ ] T175 [P] [US2 Ext] Write Section 2.6: Exporting and Reusing Datasets in `docs/module-3-isaac/ch2-synthetic-data-generation.md` (export workflow, use outside simulation from research.md FR-017)
- [ ] T176 [P] [US2 Ext] Write Chapter 2 exercises and summary in `docs/module-3-isaac/ch2-synthetic-data-generation.md`

#### Code Examples for Chapter 2

- [ ] T177 [P] [US2 Ext] Create `configure_sdg_pipeline.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (configure synthetic data recorder: RGB, depth, segmentation)
- [ ] T178 [P] [US2 Ext] Create `generate_dataset_batch.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (generate 1000 frames with randomization: lighting, poses, viewpoints from research.md Section 4.2.2)
- [ ] T179 [P] [US2 Ext] Create `export_coco_format.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (export dataset to COCO JSON format with annotations)
- [ ] T180 [P] [US2 Ext] Create `export_pascalvoc_format.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (export dataset to Pascal VOC XML format)
- [ ] T181 [P] [US2 Ext] Create `visualize_dataset.py` in `docs/module-3-isaac/assets/code/isaac_sim/` (visualize synthetic dataset: images, masks, bounding boxes)

#### Diagrams for Chapter 2

- [ ] T182 [P] [US2 Ext] Create Mermaid diagram in `docs/module-3-isaac/ch2-synthetic-data-generation.md` showing SDG pipeline (Scene Setup → Randomization → Capture → Labeling → Export)
- [ ] T183 [P] [US2 Ext] Create Mermaid diagram in `docs/module-3-isaac/ch2-synthetic-data-generation.md` showing data type comparison table (RGB, depth, semantic, instance, bounding boxes)

#### Screenshots for Chapter 2

- [ ] T184 [P] [US2 Ext] Capture screenshot of synthetic data recorder UI in `static/img/module-3/sdg-recorder-ui.png`
- [ ] T185 [P] [US2 Ext] Capture screenshot showing RGB, depth, segmentation side-by-side in `static/img/module-3/synthetic-data-types.png`

#### Validation for Chapter 2

- [ ] T186 [US2 Ext] Run validation script on all Chapter 2 code examples: `python scripts/validate_module3_examples.py --chapter 2`
- [ ] T187 [US2 Ext] Test all Chapter 2 code examples on Docker container
- [ ] T188 [US2 Ext] Verify Chapter 2 frontmatter matches schema
- [ ] T189 [US2 Ext] Build Docusaurus site and verify Chapter 2 renders correctly
- [ ] T190 [US2 Ext] Manual review: Complete Chapter 2 as student, generate 1000+ frame labeled dataset

**Checkpoint**: Chapter 2 complete. All 6 chapters now finished.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Module-wide improvements, validation, and deployment preparation

- [ ] T191 [P] Update `sidebars.js` to reflect final chapter order: Ch1 → Ch2 → Ch3 → Ch4 → Ch5 → Ch6
- [ ] T192 [P] Create Module 3 introduction page in `docs/module-3-isaac/index.md` (module overview, learning path, prerequisites)
- [ ] T193 [P] Update `README.md` with Module 3 completion status and code example count
- [ ] T194 [P] Create `docs/module-3-isaac/troubleshooting.md` aggregating all troubleshooting sections from chapters
- [ ] T195 [P] Create `docs/module-3-isaac/resources.md` with links to official docs (NVIDIA Isaac, Isaac ROS, Nav2 from research.md Section 5.4)
- [ ] T196 Run full validation suite on all 50+ code examples: `python scripts/validate_module3_examples.py --all`
- [ ] T197 Run Docusaurus build and verify no broken links: `npm run build && npm run serve`
- [ ] T198 Test all 6 chapters sequentially as a student would (15-20 hour completion time validation)
- [ ] T199 [P] Create ADR for key architectural decisions if not already documented (chapter structure, USD models, Isaac ROS exclusivity, Nav2 tuning, validation strategy from plan.md Section "Key Design Decisions")
- [ ] T200 Verify all success criteria from spec.md:
  - SC-001: Isaac Sim launch within 30 minutes ✅
  - SC-002: 90% generate 100+ frame dataset ✅
  - SC-003: VSLAM map within 1 hour ✅
  - SC-004: 85% Nav2 success rate (8/10 trials) ✅
  - SC-005: Explain pipeline with diagram ✅
  - SC-006: 50+ runnable code examples ✅
  - SC-007: All examples execute on specified hardware ✅
  - SC-008: 80%+ agree GPU acceleration explained ✅
  - SC-009: 5+ Mermaid diagrams ✅
  - SC-010: Troubleshoot 3+ common issues ✅
  - SC-011: 75% complete capstone on first try ✅
  - SC-012: Content accessible to beginners ✅

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Chapter 1 (Isaac Sim Basics)
- **User Story 2 (Phase 4)**: Depends on Foundational - Chapter 3 (VSLAM) - Can start after Foundational, does NOT depend on US1
- **User Story 3 (Phase 5)**: Depends on Foundational - Chapter 4 (Depth) - Can start after Foundational, does NOT depend on US1/US2
- **User Story 4 (Phase 6)**: Depends on Foundational + US2 + US3 - Chapter 5 (Nav2) integrates VSLAM and depth perception
- **User Story 5 (Phase 7)**: Depends on ALL previous user stories - Chapter 6 (Capstone) synthesizes all concepts
- **User Story 2 Ext (Phase 8)**: Depends on Foundational - Chapter 2 (Synthetic Data) - Can be done anytime after US1, but recommended after US5 for better context
- **Polish (Phase 9)**: Depends on all chapters being complete

### User Story Dependencies

```
Foundational (Phase 2) ✅
    ↓
    ├─→ US1 (Chapter 1) ─────────────────────┐
    ├─→ US2 (Chapter 3 - VSLAM) ─────────────┼─→ US4 (Chapter 5 - Nav2) ──→ US5 (Chapter 6 - Capstone)
    └─→ US3 (Chapter 4 - Depth) ─────────────┘
         ↓
    US2 Ext (Chapter 2 - Synthetic Data) - Optional order, recommended after US5
```

**Parallel Opportunities**:
- After Foundational: US1, US2, US3 can all start in parallel (different chapters)
- US4 (Nav2) must wait for US2 (VSLAM) and US3 (Depth) to complete
- US5 (Capstone) must wait for all previous user stories
- Chapter 2 (Synthetic Data) is independent and can be worked on anytime after Chapter 1

### Within Each User Story (Chapter)

1. Chapter content tasks (all [P] tasks) can run in parallel
2. Code examples (all [P] tasks) can run in parallel
3. Diagrams (all [P] tasks) can run in parallel
4. Screenshots (all [P] tasks) can run in parallel
5. Validation tasks must run sequentially after all content/code is complete

### Recommended Execution Strategy

**MVP First** (Maximum value, minimum effort):
1. Complete Phase 1 (Setup) + Phase 2 (Foundational)
2. Complete Phase 3 (US1 - Chapter 1: Isaac Sim Basics)
3. **STOP and VALIDATE**: Test Chapter 1 independently
4. If successful, this is a valid MVP: Students can set up Isaac Sim and generate synthetic data

**Incremental Delivery** (Add value progressively):
1. MVP (Chapter 1) ✅
2. Add Chapter 3 (VSLAM) → Test independently → Release
3. Add Chapter 4 (Depth) → Test independently → Release
4. Add Chapter 5 (Nav2) → Test independently → Release (requires Ch3 + Ch4)
5. Add Chapter 6 (Capstone) → Test independently → Release (requires all)
6. Add Chapter 2 (Synthetic Data) → Test independently → Release (enhances Ch1)

**Parallel Team Strategy**:
- **Week 1**: Everyone on Setup + Foundational
- **Week 2-3**:
  - Developer A: Chapter 1 content + code examples
  - Developer B: Chapter 3 content + code examples
  - Developer C: Chapter 4 content + code examples
- **Week 4**:
  - Developer A: Chapter 2 content + code examples
  - Developer B+C: Chapter 5 content + code examples (requires Ch3+Ch4)
- **Week 5**: Everyone on Chapter 6 (Capstone)
- **Week 6**: Polish phase (validation, ADRs, module-wide improvements)

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# Launch all Chapter 1 content tasks together (all [P] marked):
Task T021: "Write Chapter 1 introduction"
Task T022: "Write Section 1.1: Isaac Sim Installation"
Task T023: "Write Section 1.2: Isaac Sim UI Tour"
Task T024: "Write Section 1.3: Loading USD Robot Models"
Task T025: "Write Section 1.4: Running Basic Simulation"
Task T026: "Write Section 1.5: ROS 2 Bridge Setup"
Task T027: "Write Section 1.6: Capturing Synthetic Sensor Data"
Task T028: "Write Section 1.7: Troubleshooting Common Issues"
Task T029: "Write Chapter 1 exercises"
Task T030: "Write Chapter 1 summary"

# Launch all Chapter 1 code examples together (all [P] marked):
Task T031: "Create launch_isaac_sim.py"
Task T032: "Create load_humanoid_model.py"
Task T033: "Create ros2_bridge_setup.py"
Task T034: "Create capture_rgb_depth.py"
Task T035: "Create synthetic_data_pipeline.py"
Task T036: "Create verify_sensor_topics.sh"
Task T037: "Create gpu_memory_monitor.py"

# Launch all Chapter 1 diagrams together (all [P] marked):
Task T038: "Create Isaac Sim architecture diagram"
Task T039: "Create simulation setup workflow diagram"
Task T040: "Create ROS 2 data flow diagram"

# Launch all Chapter 1 screenshots together (all [P] marked):
Task T041: "Capture Isaac Sim default scene screenshot"
Task T042: "Capture humanoid loaded screenshot"
Task T043: "Capture ROS 2 Bridge config screenshot"
Task T044: "Capture RGB/depth sample screenshot"

# Then run validation tasks sequentially:
Task T045 → T046 → T047 → T048 → T049
```

---

## Implementation Strategy

### MVP Scope (Chapter 1 Only)

**Estimated Effort**: 3-4 weeks (1 developer)

**Deliverables**:
- Chapter 1: Isaac Sim Basics (complete)
- 7 Python code examples
- 3 Mermaid diagrams
- 4 screenshots
- Validation suite for Chapter 1
- Students can set up Isaac Sim and generate synthetic data

**Value**: Immediate educational value - students learn photorealistic simulation and synthetic data generation (foundation for all advanced perception)

### Full Module 3 Scope

**Estimated Effort**: 10-12 weeks (1 developer) OR 5-6 weeks (3 developers in parallel)

**Deliverables**:
- 6 complete chapters (12,000+ words)
- 50+ Python code examples
- 3 USD humanoid robot models
- 15+ Mermaid diagrams
- 15+ screenshots
- Comprehensive validation suite
- Module-wide troubleshooting guide
- Official documentation references

**Value**: Complete AI-Robot Brain module - students master NVIDIA Isaac Sim, Isaac ROS, and Nav2 for autonomous humanoid navigation

---

## Task Count Summary

- **Total Tasks**: 200
- **Setup Phase**: 10 tasks (T001-T010)
- **Foundational Phase**: 10 tasks (T011-T020)
- **User Story 1 (Chapter 1)**: 29 tasks (T021-T049)
- **User Story 2 (Chapter 3 - VSLAM)**: 31 tasks (T050-T080)
- **User Story 3 (Chapter 4 - Depth)**: 28 tasks (T081-T108)
- **User Story 4 (Chapter 5 - Nav2)**: 32 tasks (T109-T140)
- **User Story 5 (Chapter 6 - Capstone)**: 28 tasks (T141-T168)
- **User Story 2 Ext (Chapter 2 - Synthetic Data)**: 22 tasks (T169-T190)
- **Polish Phase**: 10 tasks (T191-T200)

**Parallel Opportunities Identified**:
- 120+ tasks marked [P] (can run in parallel within their phase)
- All 5 user stories (chapters 1, 3, 4) can start in parallel after Foundational phase
- All content, code, diagram, screenshot tasks within a chapter can run in parallel

**Independent Test Criteria**:
- **US1**: Generate 100+ frame synthetic dataset
- **US2**: Produce valid VSLAM occupancy grid map
- **US3**: Generate 3D point cloud and visualize in RViz2
- **US4**: Navigate to goal 8/10 trials (±0.2m)
- **US5**: Explain pipeline with diagram, run end-to-end system

**Suggested MVP**: User Story 1 (Chapter 1) - Isaac Sim Basics (29 tasks after Setup + Foundational)

---

## Validation Checkpoints

After each chapter (user story) phase, run these validations:

1. **Code Validation**: `python scripts/validate_module3_examples.py --chapter X`
2. **Schema Validation**: Verify frontmatter matches `contracts/chapter-schema.json`
3. **Build Test**: `npm run build && npm run serve`
4. **Manual Walkthrough**: Complete chapter as student, verify independent test criteria
5. **Success Criteria Check**: Validate relevant SC-XXX metrics from spec.md

**Final Module Validation** (before release):
1. Run full validation suite on all 50+ code examples
2. Test complete 15-20 hour learning path sequentially
3. Verify all 12 success criteria (SC-001 through SC-012) are met
4. Ensure all functional requirements (FR-001 through FR-020) are satisfied
5. Check code example count: 50+ ✅
6. Check Mermaid diagram count: 5+ ✅
7. Confirm hardware requirements documented for each tutorial

---

**Tasks Generation Complete**
**Total Tasks**: 200
**Ready for Implementation**: Phase 1 (Setup) can start immediately
**Estimated Timeline**:
- MVP (Chapter 1): 3-4 weeks (1 developer)
- Full Module: 10-12 weeks (1 developer) OR 5-6 weeks (3 parallel developers)
**Next Step**: Begin Phase 1 - Setup (T001-T010)
