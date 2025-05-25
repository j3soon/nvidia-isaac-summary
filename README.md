# NVIDIA Isaac Summary

An unofficial summary of NVIDIA Isaac.

The following is a list of NVIDIA Isaac components. [[link](https://developer.nvidia.com/isaac)]

## Component Descriptions

### Overview

- Omniverse [[link](https://www.nvidia.com/en-us/omniverse/)][[docs](https://docs.omniverse.nvidia.com/)][[github](https://github.com/NVIDIA-Omniverse)][[youtube](https://www.youtube.com/@NVIDIAOmniverse)]
  > The platform for connecting and developing OpenUSD applications.
  >
  > -- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)

  It is worth noting that Omniverse is a platform, and is not a standalone application. Therefore, the term `Omniverse`, should not be used to refer to a specific component, but to the entire platform. Technically speaking, there is no such thing as an app/component named Omniverse in the entire software stack. For an example, [Omniverse USD Composer](https://docs.omniverse.nvidia.com/composer/latest/index.html) (formerly _Omniverse Create_) is a template app built upon [Omniverse Kit](https://docs.omniverse.nvidia.com/dev-guide/latest/kit-architecture.html). It is a piece of software that run on the Omniverse platform. Before using any Omniverse-based applications, make sure your system meets the hardware and software [technical requirements](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/common/technical-requirements.html).

- Isaac [[link](https://developer.nvidia.com/isaac)]
  > The NVIDIA Isaac robotics platform includes a full suite of GPU-accelerated innovations in AI perception, manipulation, simulation, and software.
  >
  > -- [NVIDIA Isaac](https://developer.nvidia.com/isaac)

  Isaac stands as NVIDIA's GPU-accelerated solution for robotics, which refers to the robotics platform instead of a specific software component. Alongside essential tools for general AI applications (such as TensorRT, TAO Toolkit, and Triton Inference Server), NVIDIA Isaac components can be categorized into two main branches: (1) Isaac Sim (including Isaac Sim, Isaac Lab, etc.) for simulation, which currently [requires x86 CPUs and RTX GPUs](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html) for acceleration. (2) Isaac ROS for deployment, which can run on both PCs (x86 CPUs) and on Jetson hardware (ARM CPUs).

### (Omniverse) Isaac Sim

- (Omniverse) Isaac Sim [[link](https://developer.nvidia.com/isaac-sim)][[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+T-OV-01+V1/)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+S-OV-03+V1/)][[youtube](https://youtu.be/pxPFr58gHmQ?list=PL3jK4xNnlCVf1SzxjCm7ZxDBNl9QYyV8X)]  
  a robotics simulation toolkit based on Omniverse.
  > a scalable robotics simulation application and synthetic data-generation tool that powers photorealistic, physically accurate virtual environments.
  >
  > -- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

  Before starting, please make sure your hardware and software meet the [system requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements).

  Technically, Isaac Sim is an app built upon Omniverse Kit, which is a SDK for building apps upon the Omniverse platform. The simulation is accelerated by PhysX, while the scene is rendered through RTX rendering.  

  To install Isaac Sim, follow one of the following methods:

  1. Binary Installation:
     - [Workstation Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)
     - [Download Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/download.html)
  2. Pre-built Docker Container:  
     Docker containers are especially useful for quickly testing the software without the need to install it on the host system. However, Ubuntu is required for this. (I haven't tested it on WSL 2)
     - [Container Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html)
     - [Launch with GUI Enabled](https://github.com/j3soon/docker-isaac-sim)
  3. Install with PIP (Experimental):
     - [Python Environment Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
  4. [Deprecated on October 1, 2025] [Omniverse Launcher](https://tutorial.j3soon.com/robotics/deprecated-components/#omniverse-launcher):
     - Download [Omniverse Launcher (Linux)](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage)
     - Download [Omniverse Launcher (Windows)](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-win.exe)

  - [Deprecated on October 1, 2025] [Omniverse Nucleus](https://tutorial.j3soon.com/robotics/deprecated-components/#omniverse-launcher) and [Omniverse Cache](https://tutorial.j3soon.com/robotics/deprecated-components/#omniverse-launcher).

  - [Deprecated] [Isaac Sim Unity3D](https://tutorial.j3soon.com/robotics/deprecated-components/#isaac-sim-unity3d).

  - ROS & ROS 2 Bridges [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)][[tutorials](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/ros2_landing_page.html)]
    > tools to facilitate integration with ROS systems.
    >
    > -- [NVIDIA Isaac Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/ros_tutorials/tutorial_ros_turtlebot.html)

  - Isaac Cortex [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/cortex_tutorials/tutorial_cortex_1_overview.html)]  
    A behavior programming tool.
    > enables programming task awareness and adaptive decision making into robots, and easily switching between simulation and reality.
    >
    > -- [NVIDIA Isaac Cortex](https://www.nvidia.com/en-us/on-demand/session/gtcspring22-s42693/) (slightly rephrased)

  - Isaac Core API [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/index.html)]  
    A Python abstraction API (for Pixar USD API).
    > a set of APIs that are designed to be used in robotics applications, APIs that abstract away the complexity of USD APIs and merge multiple steps into one for frequently performed tasks.
    >
    > -- [NVIDIA Isaac Core API](https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_world.html)

  - Isaac Sim Assets [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_overview.html)]  
    A collection of USD assets including environments, robots, sensors, props, and other featured assets.

  - other features such as [OmniGraph](https://docs.isaacsim.omniverse.nvidia.com/latest/omnigraph/index.html), [Importers](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/importers_exporters.html), etc.

- (Omniverse) Replicator [[docs](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)][[blog](https://developer.nvidia.com/blog/build-custom-synthetic-data-generation-pipelines-with-omniverse-replicator/)]  
  a synthetic data generation (SDG) toolkit based on Omniverse.
  > an advanced, extensible SDK to generate physically accurate 3D synthetic data, and easily build custom synthetic data generation (SDG) tools to accelerate the training and accuracy of perception networks.
  >
  > -- [NVIDIA Replicator](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)

  - Isaac Sim Replicator [[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)]
    > a collection of extensions, python APIs, workflows, and tools such as Replicator Composer that enable a variety of synthetic data generation tasks.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_replicator.html)

  - [Deprecated] [(Omniverse) Replicator Insight](https://tutorial.j3soon.com/robotics/deprecated-components/#omniverse-replicator-insight)

### Isaac Lab

- Isaac Lab [[link](https://developer.nvidia.com/isaac/lab)][[docs](https://isaac-sim.github.io/IsaacLab/index.html)][[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/index.html)][[github](https://github.com/isaac-sim/IsaacLab)]  
  an open-source platform based on Isaac Sim, currently the de facto framework for robot learning in Omniverse.  
  Formerly _Isaac Orbit_.
  > a unified and modular framework for robot learning that aims to simplify common workflows in robotics research (such as RL, learning from demonstrations, and motion planning). It is built upon NVIDIA Isaac Sim to leverage the latest simulation capabilities for photo-realistic scenes, and fast and efficient simulation.
  >
  > -- [NVIDIA Isaac Lab](https://isaac-sim.github.io/IsaacLab/index.html)

  For more details on the position of Isaac Lab in the Isaac ecosystem, see the [Isaac Lab Ecosystem](https://isaac-sim.github.io/IsaacLab/main/source/setup/ecosystem.html).

- [Deprecated] [Isaac Orbit](https://tutorial.j3soon.com/robotics/deprecated-components/#isaac-orbit), [(Omniverse) Isaac Gym](https://tutorial.j3soon.com/robotics/deprecated-components/#omniverse-isaac-gym), [Isaac Gym (Preview Release)](https://tutorial.j3soon.com/robotics/deprecated-components/#isaac-gym-preview-release).

### Isaac ROS

- Isaac Robot Operating System (ROS) [[link](https://developer.nvidia.com/isaac-ros)][[github](https://github.com/NVIDIA-ISAAC-ROS)][[docs](https://nvidia-isaac-ros.github.io/index.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/ros)]  
  a collection of GPU-accelerated ROS2 packages (i.e., Isaac GEMs) and pipelines.  
  > a collection of hardware-accelerated packages that make it easier for ROS developers to build high-performance solutions on NVIDIA hardware.
  >
  > -- [Isaac ROS](https://developer.nvidia.com/isaac-ros)

  In legacy robotics deployment systems, ROS1 is commonly used but [lacks support for key use cases](https://design.ros2.org/articles/why_ros2.html) such as multi-robot systems, real-time systems, and production-ready environments. Therefore, ROS2 was developed to address these issues. However, using ROS2 can still present challenges, such as reproducibility across different systems and efficient GPU communication between ROS nodes. Isaac ROS, a collection of GPU-accelerated ROS2 packages and pipelines, addresses these challenges by (1) adopting a workflow based on the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit) (previously NVIDIA Docker) to ensure reproducibility across systems, allowing near-identical deployment experiences across systems with x86 and ARM CPUs, and (2) enabling efficient GPU communication between ROS nodes by reducing memory copies between GPUs and CPUs through [NITROS](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros).

  Mainstream robotics applications can be roughly categorized into manipulation and navigation tasks. To address these tasks, Isaac Manipulator and Isaac Perceptor were developed as reference workflows. These workflows will be described in detail below.

  The term `Isaac ROS` refer to the packages for ROS 2, instead of Isaac SDK. Isaac ROS should not be confused with the `ROS & ROS 2 Bridges` in Isaac Sim, or the `ROS Bridge` in Isaac SDK.  
  The packages (i.e., Isaac GEMs) are named as `Isaac ROS <Package_Name>`. Unfortunately, ambiguous terms such as `Isaac Elbrus` still exist ([source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)). Since the Elbrus package exist in both Isaac ROS and Isaac SDK, Elbrus should be refered to as `Isaac ROS Elbrus` for preciseness.  

  To get started quickly, follow these steps:

  1. Ensure your PC/Jetson hardware and operating system meet the [system requirements](https://nvidia-isaac-ros.github.io/getting_started/index.html#system-requirements). For setting up your Jetson environment, please refer to [the Jetson section](#jetson).
  2. Set up your [system environment](https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html).
  3. Configure your [development environment](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html).
  4. (Optional) To feel the power of Isaac ROS, simply follow the [Nvblox tutorial](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart) for a quick introduction.

  Some of the Isaac ROS packages are listed below:

  - (Isaac ROS) Nvblox [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)][[github](https://github.com/nvidia-isaac/nvblox)]
    > processes depth and pose to reconstruct a 3D scene in real-time and outputs a 2D costmap for Nav2. The costmap is used in planning during navigation as a vision-based solution to avoid obstacles.
    >
    > -- [Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)

    This packages includes the use of the [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) package, which uses [cuVSLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html) as the underlying algorithm.

  - (Isaac ROS) NVIDIA Isaac for Transport for ROS (NITROS) [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros)]
    > the NVIDIA implementation of type adaption and negotiation for ROS2 that eliminates software/CPU overhead and improves performance of hardware acceleration.
    >
    > -- [Isaac ROS](https://developer.nvidia.com/isaac-ros) (slightly rephrased)

  - [etc.](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)

- Isaac Manipulator [[link](https://developer.nvidia.com/isaac/manipulator)][[docs](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_manipulator/index.html)][[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_manipulator)]  
  a collection of Isaac ROS packages for manipulation tasks.  
  > a reference workflow of NVIDIA-accelerated libraries and AI models that enables developers to build AI-enabled robot arms, or manipulators, that can perceive, understand, and interact with their environments.
  >
  > -- [NVIDIA Isaac Manipulator](https://developer.nvidia.com/isaac/manipulator)

  - Extended Robot Description Format (XRDF) [[docs](https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html)]
    > designed to supplement the URDF by adding specification for: (1) Semantic labeling of configuration space (i.e., c-space) and tool frame(s) to control the robot, (2) Acceleration and jerk limits required to generate smooth motion, (3) Collision spheres to efficiently represent robot geometry, and (4) Masking to regulate self-collision avoidance.
    >
    > -- [Extended Robot Description Format (XRDF)](https://nvidia-isaac-ros.github.io/concepts/manipulation/xrdf.html)
    - Lula Robot Description and XRDF Editor [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_robot_description_editor.html)]
      > UI tool to generate a configuration file that supplements the information available about a robot in its URDF.
      >
      > -- [Lula Robot Description and XRDF Editor](https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_robot_description_editor.html)

  - (Isaac ROS) cuMotion [[docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)][[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion)]
    > provides CUDA-accelerated manipulation capabilities for robots in ROS 2. It provides two main capabilities: (1) Motion generation for robot arms via integration of cuMotion into MoveIt 2, and (2) Segmentation of robots from depth streams using cuMotion's kinematics and geometry processing functions to accurately identify and filter out parts of the robot. This allows reconstruction of obstacles in the environment without spurious contributions from the robot itself.
    >
    > -- [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/index.html)

    - cuRobo [[docs](https://curobo.org/)][[github](https://github.com/NVlabs/curobo)]
      > a CUDA accelerated library containing a suite of robotics algorithms that run significantly faster than existing implementations leveraging parallel compute.
      >
      > -- [NVIDIA cuRobo](https://github.com/NVlabs/curobo)

      cuRobo is shipped as Isaac ROS cuMotion for ROS2 integration. ([source](https://curobo.org/))

      - cuRobo with Isaac Sim [[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_curobo.html)]

    - (Isaac ROS) Pose Estimation [[docs](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/index.html)][[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)]
      > Deep learned, NVIDIA-accelerated 3D object pose estimation
      >
      > -- [NVIDIA Isaac ROS Pose Estimation](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)

      - Foundation Pose [[github](https://github.com/NVlabs/FoundationPose)]
        > a unified foundation model for 6D object pose estimation and tracking, supporting both model-based and model-free setups.
        >
        > -- [FoundationPose](https://github.com/NVlabs/FoundationPose)

- Isaac Perceptor [[link](https://developer.nvidia.com/isaac/perceptor)][[docs](https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/index.html)][[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_perceptor)]  
  a collection of Isaac ROS packages for autonomous mobile robots (AMRs).  
  Successor of _Isaac for AMRs_ and _Isaac AMR_.
  > a reference workflow of NVIDIA-accelerated libraries and AI models that helps you quickly build robust autonomous mobile robots (AMRs) to perceive, localize, and operate in unstructured environments like warehouses or factories.
  >
  > -- [NVIDIA Isaac Perceptor](https://developer.nvidia.com/isaac/perceptor)

- [Deprecated] [Isaac AMR](https://tutorial.j3soon.com/robotics/deprecated-components/#isaac-amr), [Isaac SDK](https://tutorial.j3soon.com/robotics/deprecated-components/#isaac-sdk).

### Mission Dispatch and Client

- Isaac Mission Dispatch [[github](https://github.com/nvidia-isaac/isaac_mission_dispatch)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/mission-dispatch)]
  > a cloud service that enables the communication between edge robots and other cloud services responsible for managing a fleet of robots. The communication between Mission Dispatch and robots is designed per VDA5050 protocol and uses MQTT, as MQTT is the industry standard for a highly efficient, scalable protocol for connecting devices over the internet. VDA 5050 is an open standard for communication between fleets of AGVs/AMRs and a central fleet service.
  >
  > -- [Isaac Mission Dispatch](https://github.com/nvidia-isaac/isaac_mission_dispatch)
- Isaac ROS Mission Client [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client)]
  > the ROS 2 packages for Mission Client, which communicates to a robot fleet management service. Mission Client receives tasks and actions from the fleet management service and updates its progress, state, and errors. Mission Client performs navigation actions with Nav2 and can be integrated with other ROS actions.
  >
  > -- [Isaac ROS Mission Client](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mission_client)

### (NVIDIA) Warp

- (NVIDIA) Warp [[link](https://developer.nvidia.com/warp-python)][[docs](https://nvidia.github.io/warp/)][[github](https://github.com/NVIDIA/warp)][[blog](https://developer.nvidia.com/blog/creating-differentiable-graphics-and-physics-simulation-in-python-with-nvidia-warp/)][[gtc24](https://www.nvidia.com/en-us/on-demand/session/gtc24-s63345/)]
  > an open-source developer framework for building and accelerating data generation and spatial computing in Python. Warp gives coders an easy way to write GPU-accelerated, kernel-based programs for simulation AI, robotics, and machine learning (ML).
  >
  > -- [NVIDIA Warp](https://developer.nvidia.com/warp-python)

  Not to be confused with CUDA warps (groups of 32 parallel threads) or [warp-level primitives](https://developer.nvidia.com/blog/using-cuda-warp-level-primitives/) in CUDA programming. Personally, I prefer to refer to it as _NVIDIA Warp_ (or _Warp Python_ as in the official URL) for preciseness. In addition, for a detailed comparison with other Python GPU libraries, see the [FAQ](https://nvidia.github.io/warp/faq.html).

- MuJoCo-Warp (or _MJWarp_) [[github](https://github.com/google-deepmind/mujoco_warp)]

  a GPU-optimized version of [MuJoCo](https://github.com/google-deepmind/mujoco) implemented in NVIDIA Warp. It provides GPU-accelerated physics simulation capabilities and will be integrated into Newton and [MJX](https://github.com/google-deepmind/mujoco/tree/main/mjx).

  > a GPU-optimized version of the MuJoCo physics simulator, designed for NVIDIA hardware.
  >
  > -- [MuJoCo Warp (MJWarp)](https://github.com/google-deepmind/mujoco_warp)

- Newton [[blog](https://developer.nvidia.com/blog/announcing-newton-an-open-source-physics-engine-for-robotics-simulation/)][[gtc25](https://www.nvidia.com/en-us/on-demand/session/gtc25-s72709/)]
  > an open-source, extensible physics engine being developed by NVIDIA, Google DeepMind, and Disney Research to advance robot learning and development.
  >
  > -- [NVIDIA Newton](https://developer.nvidia.com/blog/announcing-newton-an-open-source-physics-engine-for-robotics-simulation/)

### GR00T (Generalist Robot 00 Technology)

> Pronounced as _groot_.

- GR00T (Generalist Robot 00 Technology) [[link](https://developer.nvidia.com/isaac/gr00t)] [[blog](https://developer.nvidia.com/blog/advancing-humanoid-robot-sight-and-skill-development-with-nvidia-project-gr00t)]
  > a research initiative and development platform for developing general-purpose robot foundation models and data pipelines to accelerate humanoid robotics research and development.
  >
  > -- [NVIDIA Isaac GR00T](https://developer.nvidia.com/isaac/gr00t)

  - GR00T N1 [[github](https://github.com/NVIDIA/Isaac-GR00T)][[paper](https://arxiv.org/abs/2503.14734)][[huggingface](https://huggingface.co/nvidia/GR00T-N1-2B)]

    > the world's first open foundation model for generalized humanoid robot reasoning and skills. This cross-embodiment model takes multimodal input, including language and images, to perform manipulation tasks in diverse environments.
    >
    > -- [NVIDIA Isaac GR00T N1](https://github.com/NVIDIA/Isaac-GR00T)

  - GR00T Workflows [[link](https://developer.nvidia.com/isaac/gr00t#section-key-workflows)]

    The GR00T workflows include GR00T-Teleop, GR00T-Mimic, GR00T-Gen, GR00T-Dexterity, GR00T-Mobility, GR00T-Control, and GR00T-Perception.

- Open Physical AI Dataset [[blog](https://blogs.nvidia.com/blog/open-physical-ai-dataset/)][[huggingface](https://huggingface.co/collections/nvidia/physical-ai-67c643edbb024053dcbcd6d8)]

### Cosmos

- Cosmos [[link](https://www.nvidia.com/en-us/ai/cosmos/)][[docs](https://developer.nvidia.com/cosmos)][[github](https://github.com/NVIDIA/Cosmos)][[paper](https://arxiv.org/abs/2501.03575)][[blog](https://blogs.nvidia.com/blog/cosmos-world-foundation-models)][[youtube](https://youtu.be/9Uch931cDx8)][[huggingface](https://huggingface.co/collections/nvidia/cosmos-6751e884dc10e013a0a0d8e6)]
  > a platform of state-of-the-art generative world foundation models (WFM), advanced tokenizers, guardrails, and an accelerated data processing and curation pipeline built to accelerate the development of physical AI systems such as autonomous vehicles (AVs) and robots.
  >
  > -- [NVIDIA Cosmos](https://www.nvidia.com/en-us/ai/cosmos/)
  - Cosmos World Foundation Models [[link](https://developer.nvidia.com/cosmos#section-cosmos-models)][[github](https://github.com/nvidia-cosmos)]
    - Cosmos-Predict1 [[github](https://github.com/nvidia-cosmos/cosmos-predict1)][[website](https://research.nvidia.com/labs/dir/cosmos-predict1/)][[huggingface](https://huggingface.co/collections/nvidia/cosmos-predict1-67c9d1b97678dbf7669c89a7)]
      > a collection of general-purpose world foundation models for Physical AI that can be fine-tuned into customized world models for downstream applications.
      >
      > -- [NVIDIA Cosmos](https://github.com/NVIDIA/Cosmos)

      - Cosmos-Tokenize1 [[link](https://github.com/nvidia-cosmos/cosmos-predict1?tab=readme-ov-file#cosmos-predict1-models)]
    - Cosmos-Transfer1 [[github](https://github.com/nvidia-cosmos/cosmos-transfer1)][[website](https://research.nvidia.com/labs/dir/cosmos-transfer1/)][[huggingface](https://huggingface.co/collections/nvidia/cosmos-transfer1-67c9d328196453be6e568d3e)]
      > a world-to-world transfer model designed to bridge the perceptual divide between simulated and real-world environments.
      >
      > -- [NVIDIA Cosmos](https://github.com/NVIDIA/Cosmos)
    - Cosmos-Reason1 [[github](https://github.com/nvidia-cosmos/cosmos-reason1)][[website](https://research.nvidia.com/labs/dir/cosmos-reason1/)]
      > understand the physical common sense and generate appropriate embodied decisions in natural language through long chain-of-thought reasoning processes.
      >
      > -- [NVIDIA Cosmos](https://github.com/NVIDIA/Cosmos)
  - [Archived] Cosmos Models [[github](https://github.com/NVIDIA/Cosmos)]
    - Cosmos Diffusion Model [[github](https://github.com/NVIDIA/Cosmos/blob/main/cosmos1/models/diffusion/README.md)][[nim](https://build.nvidia.com/nvidia/cosmos-1_0-diffusion-7b)]
      > Pre-trained Diffusion-based world foundation models for Text2World and Video2World generation
    - Cosmos Autoregressive Model [[github](https://github.com/NVIDIA/Cosmos/blob/main/cosmos1/models/autoregressive/README.md)][[nim](https://build.nvidia.com/nvidia/cosmos-1_0-autoregressive-5b)]
      > Pre-trained Autoregressive-based world foundation models for Video2World generation
  - [Archived] Cosmos Tokenizer [[github](https://github.com/NVIDIA/Cosmos-Tokenizer)][[youtube](https://youtu.be/Soy_myOfWIU)]
    > a suite of image and video tokenizers that advances the state-of-the-art in visual tokenization, paving the way for scalable, robust and efficient development of large auto-regressive transformers (such as LLMs) or diffusion generators.
    >
    > -- [Cosmos Tokenizer](https://github.com/NVIDIA/Cosmos-Tokenizer)
  - [etc.](https://developer.nvidia.com/cosmos)

### Miscellaneous

- Isaac Nova Orin [[link](https://developer.nvidia.com/isaac/nova-orin)]  
  a reference architecture for AMRs based on NVIDIA Jetson AGX Orin.
  > a state-of-the-art compute and sensor reference architecture to accelerate AMR development and deployment. It features up to two Jetson AGX Orin computers and a full sensor suite for next-gen AMRs that enable surround vision-based perception with lidar-based solutions.
  >
  > -- [NVIDIA Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin)

- Nova Carter [[link](https://robotics.segway.com/nova-carter/)][[spec](https://docs.nvidia.com/isaac/doc/novacarter.html)]  
  a reference design robot based on the Isaac Nova Orin architecture.
  > a reference design robot that uses the Nova Orin compute and sensor architecture. It's a complete robotics development platform that accelerates the development and deployment of next-generation Autonomous Mobile Robots (AMRs). You can learn more about it from our partner, Segway Robotics
  >
  > -- [NVIDIA Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin)

- NVIDIA IGX Orin [[link](https://www.nvidia.com/en-us/edge-computing/products/igx/)]
  > an industrial-grade, edge AI platform that combines enterprise-level hardware, software, and support. It's purpose-built for industrial and medical environments, delivering powerful AI compute, high-bandwidth sensor processing, enterprise security, and functional safety. The platform also comes with NVIDIA AI Enterprise and up to 10 years of support, so you can confidently deliver AI safely and securely to support human and machine collaboration.
  >
  > -- [NVIDIA IGX Orin](https://www.nvidia.com/en-us/edge-computing/products/igx/)

- OSMO [[link](https://developer.nvidia.com/osmo)]
  > a cloud-native workflow orchestration platform that lets you easily scale your workloads across distributed environments—from on-premises to private and public cloud. It provides a single pane of glass for scheduling complex multi-stage and multi-container heterogeneous computing workflows.
  >
  > -- [NVIDIA OSMO](https://developer.nvidia.com/osmo)

  OSMO refers to an orchestration platform, which can take a workflow spec and run the specified workloads on Omniverse Cloud or DGX Cloud.

- cuOpt [[link](https://developer.nvidia.com/cuopt-logistics-optimization)][[docs](https://docs.nvidia.com/cuopt/index.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/cuopt/containers/cuopt)][[nim](https://build.nvidia.com/nvidia/nvidia-cuopt)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)][[github](https://github.com/NVIDIA/cuOpt-Resources)]  
  a GPU-accelerated solver for [vehicle routing problem](https://en.wikipedia.org/wiki/Vehicle_routing_problem).  
  Formerly _ReOpt_.
  > a GPU-accelerated logistics solver that uses heuristics and optimizations to calculate complex vehicle routing problem variants with a wide range of constraints.
  >
  > -- [NVIDIA cuOpt](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)
  - cuOpt for Isaac Sim [[docs](https://docs.isaacsim.omniverse.nvidia.com/latest/digital_twin/warehouse_logistics/logistics_tutorial_cuopt.html)]
    > a reference for the use of NVIDIA cuOpt to solve routing optimization problems in simulation.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/logistics_tutorial_cuopt.html)

- Omniverse Cloud [[link](https://www.nvidia.com/en-us/omniverse/cloud/)]
  > a platform of APIs and microservices enabling developers to easily integrate Universal Scene Description (OpenUSD) and RTX rendering and sensor simulation technologies into industrial digitalization and perception AI workflows and applications.
  >
  > -- [NVIDIA Omniverse Cloud](https://www.nvidia.com/en-us/omniverse/cloud/)

  Including Omniverse Cloud Sensor RTX APIs and others.

- Omniverse Farm [[docs](https://docs.omniverse.nvidia.com/farm/latest/index.html)]
  > Omniverse Farm Queue and Omniverse Farm Agent allow you to run tasks in the background, and to run automated jobs defined by you or others.
  >
  > -- [NVIDIA Omniverse Farm](https://docs.omniverse.nvidia.com/farm/latest/index.html)

  Omniverse Farm is actually flexible enough to run arbitrary tasks, not just rendering tasks. See the [j3soon/omni-farm-isaac](https://github.com/j3soon/omni-farm-isaac) repository for more information.

- Mega [[blog](https://blogs.nvidia.com/blog/mega-omniverse-blueprint/)][[youtube](https://youtu.be/S65_0clKQTE)]
  > an Omniverse Blueprint for developing, testing and optimizing physical AI and robot fleets at scale in a digital twin before deployment into real-world facilities.
  >
  > -- [NVIDIA Blog](https://blogs.nvidia.com/blog/mega-omniverse-blueprint/)

- NVIDIA NIMs and Blueprints for Simulation [[link](https://build.nvidia.com/explore/simulation)][[github](https://github.com/NVIDIA-Omniverse-blueprints)]
  - 3D Conditioning for Precise Visual Generative AI [[github](https://github.com/NVIDIA-Omniverse-blueprints/3d-conditioning)][[link](https://build.nvidia.com/nvidia/conditioning-for-precise-visual-generative-ai)]
    > Enhance and modify high-quality compositions using real-time rendering and generative AI output without affecting a hero product asset.
    >
    > -- [3D Conditioning for Precise Visual Generative AI](https://github.com/NVIDIA-Omniverse-blueprints/3d-conditioning)
  - Synthetic Manipulation Motion Generation for Robotics [[github](https://github.com/NVIDIA-Omniverse-blueprints/synthetic-manipulation-motion-generation)][[link](https://build.nvidia.com/nvidia/isaac-gr00t-synthetic-manipulation)]
    > Reference workflow for generating large amounts of synthetic motion trajectories for robot manipulation from a few human demonstrations.
    >
    > -- [Synthetic Manipulation Motion Generation for Robotics](https://github.com/NVIDIA-Omniverse-blueprints/synthetic-manipulation-motion-generation)
  - AI Weather Analytics with Earth-2 [[github](https://github.com/NVIDIA-Omniverse-blueprints/earth2-weather-analytics)][[link](https://build.nvidia.com/nvidia/earth2-weather-analytics)]
    > Reference implementation of the Omniverse Blueprint for Earth-2 Weather Analytics.
    >
    > -- [Earth-2 Weather Analytics Blueprint](https://github.com/NVIDIA-Omniverse-blueprints/earth2-weather-analytics)
  - Build a Digital Twin for Interactive Fluid Simulation [[github](https://github.com/NVIDIA-Omniverse-blueprints/digital-twins-for-fluid-simulation)][[link](https://build.nvidia.com/nvidia/digital-twins-for-fluid-simulation)]
    > a reference workflow for building real-time digital twins for external aerodynamic Computational Fluid Dynamics (CFD) workflows combining CUDA-X accelerated solvers, PhysicsNeMo for physics AI, and Omniverse for high quality rendering.
    >
    > -- [Omniverse Blueprint for Real-time Computer-aided Engineering Digital Twins](https://github.com/NVIDIA-Omniverse-blueprints/digital-twins-for-fluid-simulation)
  - and more...

- IsaacSimZMQ [[github](https://github.com/isaac-sim/IsaacSimZMQ)]
  > A reference bridge implementation for bidirectional communication between NVIDIA Isaac Sim and external applications using ZeroMQ and Protobuf.
  >
  > -- [IsaacSimZMQ](https://github.com/isaac-sim/IsaacSimZMQ)

### NVIDIA Research Projects

- R2D2 (or $R^2D^2$, Robotics Research and Development Digest) [[blogs](https://developer.nvidia.com/blog/tag/robotics-research-development-digest-r2d2/)]
  - 1st Edition [[blog](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)]
    - MobilityGen [[github](https://github.com/NVlabs/MobilityGen)][[dli](https://learn.nvidia.com/courses/course-detail?course_id=course-v1:DLI+S-OV-37+V1)]
      > A simulation-based workflow that uses Isaac Sim to rapidly generate large synthetic motion datasets for building models for robots across different embodiments and environments, as well as testing robots to navigate new environments, reducing costs and time compared to real-world data collection.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)
    - COMPASS (Cross-embOdiment Mobility Policy via ResiduAl RL and Skill Synthesis) [[github](https://github.com/NVlabs/COMPASS)][[site](https://nvlabs.github.io/COMPASS/)][[paper](https://arxiv.org/abs/2502.16372)]
      > A workflow for developing cross-embodiment mobility policies, facilitating fine-tuning using Isaac Lab, and zero-shot sim-to-real deployment.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)

      - X-Mobility [[github](https://github.com/NVlabs/X-MOBILITY)][[site](https://nvlabs.github.io/X-MOBILITY/)][[paper](https://arxiv.org/abs/2410.17491)]
        > A generalizable navigation model using auto-regressive world modeling, multi-head decoders, and decoupled policy learning for robust, zero-shot Sim2Real and cross-embodiment transfer.
        >
        > -- [X-Mobility](https://github.com/NVlabs/X-MOBILITY)
    - HOVER (Humanoid Versatile Controller) [[github](https://github.com/NVlabs/HOVER/)][[site](https://hover-versatile-humanoid.github.io/)][[paper](https://arxiv.org/abs/2410.21229)]
      > A workflow and a unified whole-body control generalist policy for diverse control modes in humanoid robots in Isaac Lab.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)
    - ReMEmbR (a Retrieval-augmented Memory for Embodied Robots) [[github](https://github.com/NVIDIA-AI-IOT/remembr)][[site](https://nvidia-ai-iot.github.io/remembr/)][[blog](https://developer.nvidia.com/blog/using-generative-ai-to-enable-robots-to-reason-and-act-with-remembr/)][[paper](https://arxiv.org/abs/2409.13682)]
      > A workflow that enables robots to reason and take mobility action, using LLMs, VLMs, and RAG (Retrieval-Augmented Generation).
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-advancing-robot-mobility-whole-body-control-with-ai-from-nvidia-research/)
  - 2nd Edition [[blog](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)]
    - Factory [[link](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html#contact-rich-manipulation)][[paper](https://arxiv.org/abs/2205.03532)]
      > A fast, physics-based simulation and learning toolkit for real-time contact-rich interactions.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - IndustReal [[paper](https://arxiv.org/abs/2305.17110)][[site](https://sites.google.com/nvidia.com/industreal)][[blog](https://developer.nvidia.com/blog/transferring-industrial-robot-assembly-tasks-from-simulation-to-reality/)]
      > A toolkit of algorithms and systems enabling robots to learn assembly tasks in simulation using reinforcement learning and transfer them to the real world.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - AutoMate [[paper](https://arxiv.org/abs/2407.08028)][[site](https://bingjietang718.github.io/automate/)][[blog](https://developer.nvidia.com/blog/training-sim-to-real-transferable-robotic-assembly-skills-over-diverse-geometries/)]
      > A novel policy-learning framework for training specialist and generalist robotic assembly policies across diverse geometries.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - MatchMaker [[paper](shttps://arxiv.org/abs/2503.05887)][[site](https://wangyian-me.github.io/MatchMaker/)]
      > A novel pipeline for auto-generating diverse, sim-ready assembly asset pairs using generative AI.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - SRSA [[paper](https://arxiv.org/abs/2503.04538)][[site](https://srsa2024.github.io/)]
      > A framework for retrieving preexisting skills for fine-tuning on a new robot assembly task.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - TacSL [[paper](https://arxiv.org/abs/2408.06506)][[site](https://iakinola23.github.io/tacsl/)]
      > A library for GPU-based visuotactile sensor simulation and learning.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
    - FORGE [[paper](https://arxiv.org/abs/2408.04587)][[site](https://noseworm.github.io/forge/)]
      > Zero-shot sim-to-real transfer of reinforcement-learning policies that use force measurements as input.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-unlocking-robotic-assembly-and-contact-rich-manipulation-with-nvidia-research/)
  - 3rd Edition [[blog](https://developer.nvidia.com/blog/r2d2-adapting-dexterous-robots-with-nvidia-research-workflows-and-models/)]
    - DextrAH-RGB [[paper](https://arxiv.org/abs/2412.01791)][[site](https://dextrah-rgb.github.io/)]
      > A workflow for dexterous grasping from stereo RGB input.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-adapting-dexterous-robots-with-nvidia-research-workflows-and-models/)
    - DexMimicGen [[github](https://github.com/NVlabs/dexmimicgen/)][[paper](https://arxiv.org/abs/2410.24185)][[site](https://dexmimicgen.github.io/)]
      > A data generation pipeline for bimanual dexterous manipulation using imitation learning (IL).
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-adapting-dexterous-robots-with-nvidia-research-workflows-and-models/)
    - GraspGen [[huggingface](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-GraspGen)]
      > A synthetic dataset of over 57 million grasps for different robots and grippers.
      >
      > -- [NVIDIA R2D2](https://developer.nvidia.com/blog/r2d2-adapting-dexterous-robots-with-nvidia-research-workflows-and-models/)

- Foundation Stereo [[github](https://github.com/NVlabs/FoundationStereo)][[site](https://nvlabs.github.io/FoundationStereo/)][[paper](https://arxiv.org/abs/2501.09898)]
  > a foundation model for stereo depth estimation designed to achieve strong zero-shot generalization.
  >
  > -- [FoundationStereo](https://github.com/NVlabs/FoundationStereo)

- RoboCasa [[github](https://github.com/robocasa/robocasa)][[docs](https://robocasa.ai/docs/introduction/overview.html)][[site](https://robocasa.ai/)][[paper](https://arxiv.org/abs/2406.02523)]
  > a large-scale simulation framework for training generally capable robots to perform everyday tasks.
  >
  > -- [RoboCasa](https://github.com/robocasa/robocasa)

- MimicGen [[github](https://github.com/NVlabs/mimicgen)][[docs](https://mimicgen.github.io/docs/introduction/overview.html)][[site](https://mimicgen.github.io/)][[paper](https://arxiv.org/abs/2310.17596)]
  > a system for automatically synthesizing large-scale, rich datasets from only a small number of human demonstrations by adapting them to new contexts.
  >
  > -- [MimicGen](https://arxiv.org/abs/2310.17596)

- MineDojo [[github](https://github.com/MineDojo/MineDojo)][[docs](https://docs.minedojo.org/)][[site](https://minedojo.org/)][[paper](https://arxiv.org/abs/2206.08853)]
  > a new AI research framework for building open-ended, generally capable embodied agents.
  >
  > -- [MineDojo](https://github.com/MineDojo/MineDojo)

- Other projects included in Isaac products such as [Curobo, FoundationPose](#isaac-ros), and [GR00T N1](#gr00t-generalist-robot-00-technology).

## Coding References

### OpenUSD Coding References

- [API Documentation](https://openusd.org/release/apiDocs.html)
- [USD Tutorials](https://openusd.org/release/tut_usd_tutorials.html)
- [OpenUSD Developer Tutorials](https://docs.omniverse.nvidia.com/workflows/latest/openusd-developer.html)
- [NVIDIA-Omniverse/OpenUSD-Code-Samples](https://github.com/NVIDIA-Omniverse/OpenUSD-Code-Samples)

### Omniverse Kit Coding References

- [Omniverse Kit Python Snippets](https://docs.omniverse.nvidia.com/dev-guide/latest/python-snippets.html)
- [Code Samples](https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref.html)
- [Python Scripting](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/python_scripting.html)
- [Set Up Your Development Environment](https://docs.omniverse.nvidia.com/workflows/latest/extensions/environment_configuration.html)
- [Getting Started with Extensions](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/extensions_basic.html)
- [NVIDIA-Omniverse/kit-extension-template](https://github.com/NVIDIA-Omniverse/kit-extension-template)
- [Omniverse Commands Tool](https://docs.omniverse.nvidia.com/isaacsim/latest/features/debugging/ext_omni_kit_commands.html)
- [OmniCLI](https://docs.omniverse.nvidia.com/connect/latest/connect-sample.html#omni-cli)
- [Omni.UI Documentation](https://docs.omniverse.nvidia.com/kit/docs/omni.ui/latest/Overview.html) (in UI through Extensions)
- [Physics Demos](https://docs.omniverse.nvidia.com/extensions/latest/ext_physics.html#explore-physics-demos) (in UI through Extensions)

### (Omniverse) Isaac Sim Coding References

- [Python Environment Installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html)
- [Python Environment](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html)
- [Debugging With Visual Studio Code](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_python_debugging.html)
- [Isaac Sim Workflows](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html)
- [Extension Workflow for RL](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/tutorial_gym_extension_workflow.html)
- [Isaac Sim: Extensions API](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)
- [Isaac Examples Menu](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_material/menu_examples.html) (in UI through Extensions)
- [Tips & Useful Links](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_material/reference_tips.html)

### OmniGraph Coding References

- `omni.graph.nodes`
  - [Standard Nodes \| Kit](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.nodes/latest/Overview.html)
  - [OmniGraph Node Library \| Omniverse Extensions](https://docs.omniverse.nvidia.com/extensions/latest/ext_omnigraph/node-library/node-library.html)
- `omni.graph.action_nodes`
  - [Action Graph Nodes \| Kit](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.action_nodes/latest/Overview.html)
- `omni.isaac.core_nodes`
  - [Core Omnigraph Nodes \| Isaac Sim](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core_nodes/docs/index.html)
- `omni.isaac.ros2_bridge`
  - [ROS2 Bridge \| Isaac Sim](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.ros2_bridge/docs/index.html)
- Custom Nodes
  - [Creating Python Nodes \| Kit](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.docs/latest/dev/CreatingPythonNodes.html)
  - [Script Node \| Kit](https://docs.omniverse.nvidia.com/kit/docs/omni.graph.scriptnode/latest/GeneratedNodeDocumentation/OgnScriptNode.html)
  - [Omnigraph: Custom Python Nodes \| Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_omnigraph_custom_python_nodes.html)

You can also hover your cursor on the OmniGraph node title to see its namespace and node name through the Isaac Sim GUI.

### Development Notes

- For VSCode Intellisense to work, you need to:
  - Omniverse Kit Extension
    - Correctly link the Omniverse app directories ([ref](https://github.com/NVIDIA-Omniverse/kit-extension-template?tab=readme-ov-file#linking-with-an-omniverse-app), [ref](https://github.com/j3soon/omni-nerf-extension?tab=readme-ov-file#development-notes))
    - Correctly set the VSCode settings `.vscode/settings.json` ([ref](https://github.com/NVIDIA-Omniverse/kit-extension-template/blob/main/.vscode/settings.json), [ref](https://github.com/j3soon/omni-nerf-extension/blob/master/extension/.vscode/settings.json))
    - Select the correct Python interpreter (e.g., the `isaac-sim` virtual environment if you are using [Isaac Sim with Anaconda](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html#advanced-running-with-anaconda))
  - Isaac Lab
    - Correctly link the Isaac Sim directories ([ref](https://isaac-sim.github.io/IsaacLab/main/source/overview/developer-guide/vs_code.html), [ref](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#creating-the-isaac-sim-symbolic-link))
    - Correctly set the VSCode settings `.vscode/settings.json` ([ref](https://github.com/isaac-sim/IsaacLab/blob/main/.vscode/tools/setup_vscode.py))
    - Select the correct Python interpreter (e.g., the `isaac-sim` virtual environment if you are using [Isaac Lab with Anaconda](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#setting-up-the-conda-environment-optional))
- Built-in Examples under `~/.local/share/ov/pkg/*`
  - Search under `~/.local/share/ov/pkg/isaac-sim-4.2.0` for example API usages.

## Epilogue

Please [open an issue](https://github.com/j3soon/nvidia-isaac-summary/issues) if you have spotted any errors or have questions regarding this document. For questions regarding the Isaac components, I recommend first going through the [Known Issues of Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/known_issues.html), [Known Issues of Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/refs/issues.html), [Troubleshooting Guide for Isaac ROS](https://nvidia-isaac-ros.github.io/troubleshooting/index.html), then considering asking in the [NVIDIA Developer Forums](https://forums.developer.nvidia.com/) under the [Isaac topic](https://forums.developer.nvidia.com/c/isaac-sdk/15).

I have documented some bug fixes and workarounds for Isaac in the [j3soon/isaac-extended](https://github.com/j3soon/isaac-extended) repository. I recommend also checking out that repository for reference.

Last updated on 2025/04/17.
