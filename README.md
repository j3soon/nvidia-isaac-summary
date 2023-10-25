# NVIDIA Isaac Summary

A list of NVIDIA Isaac components. [[link](https://developer.nvidia.com/isaac)]

- (Omniverse) Isaac Sim [[link](https://developer.nvidia.com/isaac-sim)][[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/overview.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+T-OV-01+V1/)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+S-OV-03+V1/)][[youtube](https://youtu.be/pxPFr58gHmQ?list=PL3jK4xNnlCVf1SzxjCm7ZxDBNl9QYyV8X)]  
  a robotics simulation toolkit based on Omniverse.
  > a scalable robotics simulation application and synthetic data-generation tool that powers photorealistic, physically accurate virtual environments.
  >
  > -- [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim)

  Before starting, please make sure your hardware and software meet the [system requirements](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements).

  Technically, Isaac Sim is an app built upon Omniverse Kit, which is a SDK for building apps upon the Omniverse platform. The simulation is accelerated by PhysX, while the scene is rendered through RTX rendering.  
  Isaac Sim can be downloaded through [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/download/) here:
  - [Linux](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage)
  - [Windows](https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-win.exe)

  The required assets are accessed through [Omniverse Nucleus](https://docs.omniverse.nvidia.com/prod_nucleus/prod_nucleus/overview.html), which requires setting up a (local) Nucleus account. In addition, installing [Omniverse Cache](https://docs.omniverse.nvidia.com/prod_utilities/prod_utilities/cache/overview.html) can speed up the access to Nucleus.
  - Isaac Sim Unity3D [[docs](https://docs.nvidia.com/isaac/archive/2020.1/doc/simulation/unity3d.html)]  
    Unity3D support has been deprecated ([source](https://forums.developer.nvidia.com/t/no-isaac-sim-unity3d-to-download/212951)). The term `Isaac Sim` now refer to the Omniverse-based version.
    > allows you to use Unity3D as the simulation environment for Isaac robotics.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/archive/2020.1/doc/simulation/unity3d.html)
  - ROS & ROS 2 Bridges [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros_turtlebot.html)][[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros2_turtlebot.html)]
    > tools to facilitate integration with ROS systems.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_ros_turtlebot.html)
  - Isaac Cortex [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_cortex_1_overview.html)]  
    A behavior programming tool.
    > enables programming task awareness and adaptive decision making into robots, and easily switching between simulation and reality.
    >
    > -- [NVIDIA Isaac Cortex](https://www.nvidia.com/en-us/on-demand/session/gtcspring22-s42693/) (slightly rephrased)
  - Isaac Core API [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html#isaac-sim-app-tutorial-core-hello-world)]  
    A Python abstraction API (for Pixar USD API).
    > a set of APIs that are designed to be used in robotics applications, APIs that abstract away the complexity of USD APIs and merge multiple steps into one for frequently performed tasks.
    >
    > -- [NVIDIA Isaac Core API](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_core_hello_world.html#isaac-sim-app-tutorial-core-hello-world)
  - other features such as [OmniGraph](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gui_omnigraph.html), [Importers](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_isaac_extensions.html#asset-conversion-extensions), [etc.](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_isaac_extensions.html)

- (Omniverse) Isaac Gym [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_isaac_gym.html)][[github](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs)]  
  a light-weight repository based on Isaac Sim that provides a variety of GPU-accelerated reinforcement learning environments and algorithms.  
  The repository is named as Omniverse Isaac Gym Environments (OIGE), and is released under the BSD 3-Clause License ([source](https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/LICENSE.txt)).  
  > an interface for performing reinforcement learning training and inferencing in Isaac Sim.
  >
  > -- [NVIDIA Isaac Gym](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gym_isaac_gym.html)

  - Isaac Gym (Preview Release) [[link](https://developer.nvidia.com/isaac-gym)][[github](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)][[arxiv](https://arxiv.org/abs/2108.10470)][[site](https://sites.google.com/view/isaacgym-nvidia)][[youtube](https://youtu.be/nleDq-oJjGk?list=PLq2Xfjf6QzkrgDkQdtEzlnXeUAbTPEXNH)]  
    the predecessor of (Omniverse) Isaac Gym that does not base on Isaac Sim (and Omniverse).  
    The repository is named as Isaac Gym Environments (IGE), and is released under the BSD 3-Clause License ([source](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/blob/main/LICENSE.txt)). The documentation is provided in an offline form that can be accessed after download.
    > NVIDIA’s physics simulation environment for reinforcement learning research.
    >
    > -- [NVIDIA Isaac Gym](https://developer.nvidia.com/isaac-gym)

    The term `Isaac Gym` is ambiguous when viewed from a technical perspective. It's better to specify whether the mentioned Isaac Gym is based on Isaac Sim, or the preview version that does not base on Isaac Sim.

    > Until Omniverse Isaac Gym functionality is feature complete, this standalone Isaac Gym Preview release will remain available.
    >
    > -- [NVIDIA Isaac Gym](https://developer.nvidia.com/isaac-gym)

    The latest release of Isaac Gym (Preview Release) is Preview 4, and will not be further updated.

- Isaac Orbit [[docs](https://isaac-orbit.github.io/orbit/)][[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_orbit.html)][[arxiv](https://arxiv.org/abs/2301.04195)][[site](https://isaac-orbit.github.io/)][[github](https://github.com/NVIDIA-Omniverse/Orbit)]  
  a general repository based on Isaac Sim that features a number of GPU-accelerated simulation environments, a variety of motion generators, integrations with several reinforcement learning libraries, utilities for imitation learning, etc.  
  Released under the BSD 3-Clause License ([source](https://github.com/NVIDIA-Omniverse/Orbit/blob/main/LICENSE)).
  > a unified and modular framework for robot learning powered by NVIDIA Isaac Sim.
  >
  > -- [NVIDIA Isaac Orbit](https://arxiv.org/abs/2301.04195)

  Omniverse Isaac Gym is a light-weight framework focusing on reinforcement learning tasks, while Isaac Orbit is a more general and modular framework that focuses on robotics applications. ([source](https://nvidia.slack.com/archives/C01TGK0GSJG/p1675192628308169?thread_ts=1674981564.933639&cid=C01TGK0GSJG))

- Isaac Robot Operating System (ROS) [[link](https://developer.nvidia.com/isaac-ros)][[github](https://github.com/NVIDIA-ISAAC-ROS)][[docs](https://nvidia-isaac-ros.github.io/getting_started/index.html)]  
  a collection of GPU-accelerated ROS2 packages (i.e., Isaac GEMs).  
  > a collection of hardware-accelerated packages that make it easier for ROS developers to build high-performance solutions on NVIDIA hardware.
  >
  > -- [Isaac ROS](https://developer.nvidia.com/isaac-ros)

  The term `Isaac ROS` refer to the packages for ROS 2, instead of Isaac SDK. Isaac ROS should not be confused with the `ROS & ROS 2 Bridges` in Isaac Sim, or the `ROS Bridge` in Isaac SDK.  
  The packages (i.e., Isaac GEMs) are named as `Isaac ROS <Package_Name>`. Unfortunately, ambiguous terms such as `Isaac Elbrus` still exist ([source](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)). Since the Elbrus package exist in both Isaac ROS and Isaac SDK, Elbrus should be refered to as `Isaac ROS Elbrus` for preciseness.  

  Before starting, please make sure your PC/Jetson hardware and software meet the [system requirements](https://nvidia-isaac-ros.github.io/getting_started/index.html#system-requirements). After checking the requirements, I suggest you start from the Nvblox tutorial below.

  - (Isaac ROS) Nvblox [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)]
    > processes depth and pose to reconstruct a 3D scene in real-time and outputs a 2D costmap for Nav2. The costmap is used in planning during navigation as a vision-based solution to avoid obstacles.
    >
    > -- [Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)

    You can quickly experience the power of Isaac ROS by simply following the [the quick start guide](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart) of Nvblox.
  - (Isaac ROS) NVIDIA Isaac for Transport for ROS (NITROS) [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros)]
    > the NVIDIA implementation of type adaption and negotiation for ROS2 that eliminates software/CPU overhead and improves performance of hardware acceleration.
    >
    > -- [Isaac ROS](https://developer.nvidia.com/isaac-ros) (slightly rephrased)
  - [etc.](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)

- Isaac SDK [[link](https://developer.nvidia.com/isaac-sdk)][[docs](https://docs.nvidia.com/isaac/doc/index.html)]  
  a toolkit for deploying GPU-accelerated algorithms on physical robots.
  > a toolkit that includes building blocks and tools that accelerate robot developments that require the increased perception and navigation features enabled by AI.
  >
  > -- [NVIDIA Isaac SDK](https://developer.nvidia.com/isaac-sdk)

  Personally, I suggest using Isaac ROS instead of Isaac SDK for simplicity. Since researchers/engineers working on robotics tend to be more familiar with the [Robot Operating System (ROS)](https://www.ros.org/) than the `bazel` command used in Isaac SDK. The Isaac GEMs and Isaac Applications included in Isaac SDK are also available in Isaac ROS.

  > Isaac includes Isaac GEMs for both NVIDIA’s Isaac SDK Engine and ROS2. Isaac ROS has been more recently updated to contribute hardware acceleration to the growing ROS ecosystem. You can choose whichever one is more suitable for your project.
  >
  > -- [NVIDIA Forum Moderator](https://forums.developer.nvidia.com/t/is-isaac-depreciated/224402) (slightly rephrased)

  The latest release of Isaac SDK is 2021.1, since the future roadmap of Isaac SDK is still under development ([source](https://forums.developer.nvidia.com/t/isaac-sdk-next-release/217841/2), [source](https://forums.developer.nvidia.com/t/is-isaac-depreciated/224402), [source](https://forums.developer.nvidia.com/t/isaac-sdk-is-dead/226267/2), [source](https://nvidia.slack.com/archives/CHG4MCWNQ/p1661260234425319?thread_ts=1658787137.725279&cid=CHG4MCWNQ)).

  - Isaac GEMs
    > a collection of high-performance algorithms, also called GEMs, to accelerate the development of challenging robotics applications.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/doc/overview.html#isaac-gems)

    - ROS Bridge
      > create a message translation layer between Isaac SDK and ROS to communicate between the two systems.
      >
      > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/packages/ros_bridge/doc/ros_bridge.html) (slightly rephrased)
    - etc.

  - Isaac Applications
    > provides various sample applications, which highlight features of Isaac SDK Engine or focus on the functionality of a particular Isaac SDK GEM.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/doc/overview.html#isaac-applications)

  - Isaac (Robotics) Engine
    > a feature-rich framework for building modular robotics applications.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/doc/overview.html#isaac-engine)

- Isaac for AMRs [[link](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/autonomous-mobile-robots/)]  
  a list of AMR-related software and hardwares.
  > a new open platform with edge-to-cloud compute and powerful software stacks for AI-enabled Autonomous Mobile Robots (AMRs)
  >
  > -- [NVIDIA Isaac for AMRs](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/autonomous-mobile-robots/)

- Isaac AMR [[link](https://developer.nvidia.com/isaac/amr)]  
  an end-to-end solution for AMRs, including mapping, autonomy, and simulation.
  > Isaac AMR 2.0 features an autonomous navigation stack that includes lidar-based grid mapping, global and continuous localization, a global route planner, a mission client, a behavior planner, and wheel-IMU odometry, as well as a new path and trajectory planner and controller.
  >
  > This release also includes tools for data collection and a cloud-based map creation service. You can use the on-premises data center-based mission control for optimizing route planning with the NVIDIA cuOpt engine and delivering up-to-date maps to the robots.
  >
  > -- [NVIDIA Isaac AMR](https://developer.nvidia.com/isaac/amr)

- Isaac Nova Orin [[link](https://developer.nvidia.com/isaac/nova-orin)]  
  a reference architecture for AMRs based on NVIDIA Jetson AGX Orin.
  > a state-of-the-art compute and sensor reference architecture to accelerate AMR development and deployment. It features up to two Jetson AGX Orin computers and a full sensor suite for next-gen AMRs that enable surround vision-based perception with lidar-based solutions.
  >
  > -- [NVIDIA Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin)

  - Nova Carter [[link](https://robotics.segway.com/nova-carter/)] [[spec](https://docs.nvidia.com/isaac/doc/novacarter.html)]  
    a reference design robot based on the Isaac Nova Orin architecture.
    > a reference design robot that uses the Nova Orin compute and sensor architecture. It’s a complete robotics development platform that accelerates the development and deployment of next-generation Autonomous Mobile Robots (AMRs). You can learn more about it from our partner, Segway Robotics
    >
    > -- [NVIDIA Isaac Nova Orin](https://developer.nvidia.com/isaac/nova-orin)

- cuOpt [[link](https://developer.nvidia.com/cuopt-logistics-optimization)][[docs](https://docs.nvidia.com/cuopt/index.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/cuopt/containers/cuopt)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)][[github](https://github.com/NVIDIA/cuOpt-Resources)]  
  a GPU-accelerated solver for [vehicle routing problem](https://en.wikipedia.org/wiki/Vehicle_routing_problem).  
  Previously named as ReOpt.
  > a GPU-accelerated logistics solver that uses heuristics and optimizations to calculate complex vehicle routing problem variants with a wide range of constraints.
  >
  > -- [NVIDIA cuOpt](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)
  - cuOpt for Isaac Sim [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/logistics_tutorial_cuopt.html)][[github](https://github.com/NVIDIA/cuOpt-Resources/tree/branch-22.12/cuopt-isaacsim)]
    > a reference for the use of NVIDIA cuOpt to solve routing optimization problems in simulation.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/logistics_tutorial_cuopt.html)

- (Omniverse) Replicator [[link](https://developer.nvidia.com/nvidia-omniverse-platform/replicator)][[docs](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_replicator.html)][[blog](https://developer.nvidia.com/blog/build-custom-synthetic-data-generation-pipelines-with-omniverse-replicator/)]  
  a synthetic data generation (SDG) toolkit based on Omniverse.
  > an advanced, extensible SDK to generate physically accurate 3D synthetic data, and easily build custom synthetic data generation (SDG) tools to accelerate the training and accuracy of perception networks.
  >
  > -- [NVIDIA Replicator](https://developer.nvidia.com/nvidia-omniverse-platform/replicator)
  - Isaac Sim Replicator [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_replicator_getting_started.html)]
    > a collection of extensions, python APIs, workflows, and tools such as Replicator Composer that enable a variety of synthetic data generation tasks.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_replicator.html)
  - (Omniverse) Replicator Insight [[link](https://developer.nvidia.com/nvidia-omniverse/replicator-insight-eap)]  
    > an app that enables developers to quickly view, navigate, and inspect their synthetically generated renders.
    >
    > -- [NVIDIA Replicator Insight](https://developer.nvidia.com/nvidia-omniverse/replicator-insight-eap)

To the best of my knowledge, there is no public Cloud Services (such as [NVIDIA NeMo Service](https://www.nvidia.com/en-us/gpu-cloud/nemo-llm-service/)) for Isaac at the time of writing. Nevertheless, it is possible to self-host a cloud service on-premise for Isaac though.

Please [open an issue](https://github.com/j3soon/nvidia-isaac-summary/issues) if you have spotted any errors.

I have documented some bug fixes and workarounds for Isaac in the [j3soon/isaac-extended](https://github.com/j3soon/isaac-extended) repository. I recommend also checking out that repository for reference.

Last updated on 2023/10/25.
