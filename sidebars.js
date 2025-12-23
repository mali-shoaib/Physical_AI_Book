/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // Manual sidebar configuration for better control
  mainSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-1-ros2/ch1-ros2-basics',
          label: 'Ch1: ROS 2 Basics',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/ch2-rclpy-control',
          label: 'Ch2: rclpy Control',
        },
        {
          type: 'doc',
          id: 'module-1-ros2/ch3-urdf-humanoids',
          label: 'Ch3: URDF for Humanoids',
        },
      ],
    },
    // Module 2 - TODO: Add chapters when content is ready
    // {
    //   type: 'category',
    //   label: 'Module 2: The Digital Twin (Gazebo & Unity)',
    //   collapsed: false,
    //   items: [],
    // },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-3-isaac/ch1-isaac-sim-basics',
          label: 'Ch1: Isaac Sim Basics',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/ch2-synthetic-data-generation',
          label: 'Ch2: Synthetic Data Generation',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/ch3-vslam-isaac-ros',
          label: 'Ch3: VSLAM with Isaac ROS',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/ch4-depth-perception-mapping',
          label: 'Ch4: Depth Perception & Mapping',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/ch5-nav2-integration',
          label: 'Ch5: Nav2 Integration',
        },
        {
          type: 'doc',
          id: 'module-3-isaac/ch6-end-to-end-capstone',
          label: 'Ch6: End-to-End Capstone',
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Models',
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'module-4-vla/ch1-vla-fundamentals',
          label: 'Ch1: VLA Fundamentals',
        },
        // Chapters 2-5 to be added when implemented
        // {
        //   type: 'doc',
        //   id: 'module-4-vla/ch2-voice-to-action',
        //   label: 'Ch2: Voice-to-Action with Whisper',
        // },
        // {
        //   type: 'doc',
        //   id: 'module-4-vla/ch3-llm-cognitive-planning',
        //   label: 'Ch3: LLM Cognitive Planning',
        // },
        // {
        //   type: 'doc',
        //   id: 'module-4-vla/ch4-ros2-action-execution',
        //   label: 'Ch4: ROS 2 Action Execution',
        // },
        // {
        //   type: 'doc',
        //   id: 'module-4-vla/ch5-end-to-end-capstone',
        //   label: 'Ch5: End-to-End Capstone',
        // },
      ],
    },
  ],
};

module.exports = sidebars;
