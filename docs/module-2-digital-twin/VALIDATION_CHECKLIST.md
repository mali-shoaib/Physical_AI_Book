# Module 2 Validation Checklist - Digital Twin (Gazebo & Unity)

**Purpose**: Validate Module 2 content before publication
**Feature**: specs/002-digital-twin/spec.md
**Created**: 2025-12-19

## Code Validation

- [ ] All SDF world files validated with `gz sdf --check`
- [ ] All Python launch files validated with `python validate_examples.py`
- [ ] All YAML config files validated with `yamllint`
- [ ] All SDF code blocks in markdown validated with `validate_urdf.py`
- [ ] All inline code examples \<20 lines for readability

## Chapter Frontmatter

- [ ] ch1-gazebo-physics.md has valid frontmatter per chapter-frontmatter-schema.yaml
- [ ] ch2-unity-rendering.md has valid frontmatter per chapter-frontmatter-schema.yaml
- [ ] ch3-sensor-simulation.md has valid frontmatter per chapter-frontmatter-schema.yaml
- [ ] All chapter IDs match pattern `ch[0-9]+-[a-z0-9-]+`
- [ ] All sidebar labels \<= 25 characters

## Educational Quality (per FR-015)

- [ ] Chapter 1 has Learning Objectives section
- [ ] Chapter 1 has Prerequisites section referencing Module 1
- [ ] Chapter 1 has Summary section
- [ ] Chapter 1 has Exercises section
- [ ] Chapter 2 has Learning Objectives section
- [ ] Chapter 2 has Prerequisites section
- [ ] Chapter 2 has Summary section
- [ ] Chapter 2 has Exercises section
- [ ] Chapter 3 has Learning Objectives section
- [ ] Chapter 3 has Prerequisites section
- [ ] Chapter 3 has Summary section
- [ ] Chapter 3 has Exercises section

## Functional Requirements

- [ ] FR-001: All config files (SDF, Unity scenes, sensor configs) are runnable without modification
- [ ] FR-002: Chapter 1 explains gravity, collision, contact forces, joint dynamics with math
- [ ] FR-003: Chapter 1 includes >= 3 Gazebo world examples (static, dynamic, multi-robot)
- [ ] FR-004: Chapter 2 provides Unity setup steps with ROS-TCP-Connector
- [ ] FR-005: Chapter 2 includes >= 2 Unity scenes (indoor, outdoor) with lighting/materials
- [ ] FR-006: Chapter 3 explains sensor plugins (gazebo_ros_ray_sensor, _camera, _imu)
- [ ] FR-007: Chapter 3 provides noise models (Gaussian, bias drift, update rates)
- [ ] FR-008: All code validated using validation scripts
- [ ] FR-009: All chapters include troubleshooting sections
- [ ] FR-010: All chapters reference official docs (gazebosim.org, docs.unity3d.com, docs.ros.org)
- [ ] FR-011: Sensor examples demonstrate RViz2 visualization
- [ ] FR-012: Unity integration supports both simulation-only and physics-enabled modes
- [ ] FR-013: All URDF models include inertial properties, collision meshes, visual meshes
- [ ] FR-014: Documentation includes performance optimization guidelines
- [ ] FR-015: All chapters align with textbook constitution

## Success Criteria

- [ ] SC-001: Students can create Gazebo simulation in \<15 minutes after reading Chapter 1
- [ ] SC-002: Students can set up Unity-ROS 2 integration in \<30 minutes after Chapter 2
- [ ] SC-003: Students can add sensors and visualize in RViz2 in \<20 minutes after Chapter 3
- [ ] SC-004: All simulations run at RTF >= 1.0 on 8GB RAM + integrated GPU
- [ ] SC-006: Sensor data accuracy within 10% of specifications
- [ ] SC-009: All code executes without errors on fresh ROS 2 Humble on Ubuntu 22.04

## Citations (APA Format per Constitution)

- [ ] Gazebo Fortress documentation cited (gazebosim.org/docs)
- [ ] Unity Robotics Hub cited (github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ ] ROS 2 Humble documentation cited (docs.ros.org/en/humble/)
- [ ] Sensor datasheets cited (Velodyne VLP-16, Intel RealSense D435, Bosch BMI088)
- [ ] All citations follow APA 7th edition format

## Performance Testing

- [ ] empty_world.sdf loads in Gazebo without errors
- [ ] physics_demo.sdf runs at RTF >= 1.0
- [ ] collision_test.sdf demonstrates collision detection correctly
- [ ] humanoid_world.sdf loads humanoid model with stable physics
- [ ] Unity scenes load in \<10 seconds
- [ ] Unity-ROS 2 TCP connection establishes successfully
- [ ] All sensor topics publish data at configured rates
- [ ] RViz2 displays sensor data without performance degradation

## Build & Deployment

- [ ] `npm run build` completes successfully for Docusaurus
- [ ] No broken links in Module 2 documentation
- [ ] All images and assets load correctly
- [ ] Sidebar navigation includes all 3 Module 2 chapters
- [ ] Module 2 accessible from main navigation

## Final Checks

- [ ] All tasks in tasks.md marked as completed
- [ ] No unresolved TODO comments in code
- [ ] All acceptance scenarios from spec.md verified
- [ ] No validation warnings or errors
- [ ] Constitution compliance verified (technical accuracy, reproducibility, APA citations)

---

**Status**: PENDING
**Last Updated**: 2025-12-19
**Validated By**: [Name]
**Date**: [Date]
