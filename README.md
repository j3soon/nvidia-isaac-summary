# NVIDIA Isaac Summary

A list of NVIDIA Isaac components. [[link](https://developer.nvidia.com/isaac)]

## Component Descriptions

- Omniverse [[link](https://www.nvidia.com/en-us/omniverse/)][[docs](https://docs.omniverse.nvidia.com/)]
  > The platform for connecting and developing OpenUSD applications.
  >
  > -- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)

  It is worth noting that Omniverse is a platform, and is not a standalone application. Therefore, the term `Omniverse`, should not be used to refer to a specific component, but to the entire platform. Technically speaking, there is no such thing as an Omniverse app/component in the entire software stack. For an example, [Omniverse USD Composer](https://docs.omniverse.nvidia.com/composer/latest/index.html) (formerly _Omniverse Create_) and [Omniverse USD Presenter](https://docs.omniverse.nvidia.com/presenter/latest/index.html) (formerly _Omniverse View_) are apps built upon [Omniverse Kit](https://docs.omniverse.nvidia.com/dev-guide/latest/kit-architecture.html). They are software components that run on the Omniverse platform.

- Isaac [[link](https://developer.nvidia.com/isaac)]
  > The NVIDIA Isaac robotics platform includes a full suite of GPU-accelerated innovations in AI perception, manipulation, simulation, and software.
  >
  > -- [NVIDIA Isaac](https://developer.nvidia.com/isaac)

  Isaac stands as NVIDIA's GPU-accelerated solution for robotics, which refers to the robotics platform instead of a specific software component. Alongside essential tools for general AI applications (such as TensorRT, TAO Toolkit, and Triton Inference Server), Isaac comprises two primary branches for local execution: Simulation and Deployment. The Simulation branch, including components like Isaac Sim and Isaac Gym, currently requires the use of x86 CPUs and RTX GPUs to accelerated simulations. Conversely, the Deployment branch, featuring components like Isaac ROS, can run on both PC and on Jetson hardware.

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

  The assets are accessed through [Omniverse Nucleus](https://docs.omniverse.nvidia.com/nucleus/latest/index.html), which requires setting up a (local) Nucleus account. In addition, installing [Omniverse Cache](https://docs.omniverse.nvidia.com/prod_utilities/prod_utilities/cache/overview.html) can speed up the access to Nucleus. If no custom assets are used, the built-in assets can be accessed through the default remote Nucleus server.
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
  - Isaac Sim Assets [[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets/usd_assets_overview.html)]  
    A collection of USD assets including environments, robots, sensors, props, and other featured assets.
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
  > a unified and modular framework for robot learning that aims to simplify common workflows in robotics research (such as RL, learning from demonstrations, and motion planning).
  >
  > -- [NVIDIA Isaac Orbit](https://github.com/NVIDIA-Omniverse/Orbit)

  Omniverse Isaac Gym is a light-weight framework focusing on reinforcement learning tasks, while Isaac Orbit is a more general and modular framework that focuses on robotics applications. ([source](https://github.com/NVIDIA-Omniverse/orbit/blob/main/docs/source/setup/faq.rst), [source](https://forums.developer.nvidia.com/t/orbit-vs-omniisaacgymenvs/251329/3))

  Personally, I think Isaac Orbit is preferred for general usages such as creating custom environments. However, if you are writing a research paper and just want to use built-in Omniverse Isaac Gym Environments for benchmarking, Omniverse Isaac Gym may be enough.

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

    This packages includes the use of the [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) package, which uses [cuVSLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html) as the underlying algorithm.
  - (Isaac ROS) NVIDIA Isaac for Transport for ROS (NITROS) [[github](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros)]
    > the NVIDIA implementation of type adaption and negotiation for ROS2 that eliminates software/CPU overhead and improves performance of hardware acceleration.
    >
    > -- [Isaac ROS](https://developer.nvidia.com/isaac-ros) (slightly rephrased)
  - [etc.](https://nvidia-isaac-ros.github.io/repositories_and_packages/index.html)

- Isaac SDK [[docs](https://docs.nvidia.com/isaac/archive/2021.1/doc/index.html)]  
  a toolkit for deploying GPU-accelerated algorithms on physical robots.
  > a toolkit that includes building blocks and tools that accelerate robot developments that require the increased perception and navigation features enabled by AI.
  >
  > -- [NVIDIA Isaac SDK](https://developer.nvidia.com/isaac-sdk)

  Personally, I suggest using Isaac ROS instead of Isaac SDK for simplicity. Since researchers/engineers working on robotics tend to be more familiar with the [Robot Operating System (ROS)](https://www.ros.org/) than the `bazel` command used in Isaac SDK. The Isaac GEMs and Isaac Applications included in Isaac SDK are also available in Isaac ROS.

  > Isaac includes Isaac GEMs for both NVIDIA’s Isaac SDK Engine and ROS2. Isaac ROS has been more recently updated to contribute hardware acceleration to the growing ROS ecosystem. You can choose whichever one is more suitable for your project.
  >
  > -- [NVIDIA Forum Moderator](https://forums.developer.nvidia.com/t/is-isaac-depreciated/224402) (slightly rephrased)

  The latest release of Isaac SDK is 2021.1, since the future roadmap of Isaac SDK is still under development ([source](https://forums.developer.nvidia.com/t/isaac-sdk-next-release/217841/2), [source](https://forums.developer.nvidia.com/t/is-isaac-depreciated/224402), [source](https://forums.developer.nvidia.com/t/isaac-sdk-is-dead/226267/2), [source](https://nvidia.slack.com/archives/CHG4MCWNQ/p1661260234425319?thread_ts=1658787137.725279&cid=CHG4MCWNQ)).

  The Isaac Sight GUI also belongs to Isaac SDK.

  - Isaac GEMs
    > a collection of high-performance algorithms, also called GEMs, to accelerate the development of challenging robotics applications.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/archive/2021.1/doc/overview.html#isaac-gems)

  - Isaac Applications
    > provides various sample applications, which highlight features of Isaac SDK Engine or focus on the functionality of a particular Isaac SDK GEM.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/archive/2021.1/doc/overview.html#isaac-applications)

  - Isaac (Robotics) Engine
    > a feature-rich framework for building modular robotics applications.
    >
    > -- [NVIDIA Isaac SDK](https://docs.nvidia.com/isaac/archive/2021.1/doc/overview.html#isaac-engine)

- Isaac Perceptor [[link](https://developer.nvidia.com/isaac/perceptor)]  
  Formerly _Isaac for AMRs_ and _Isaac AMR_.
  > a collection of hardware-accelerated packages for visual AI, tailored for Autonomous Mobile Robot (AMR) to perceive, localize, and operate robustly in unstructured environments. Robotics software developers can now easily access turnkey AI-based perception capabilities, ensuring reliable operations and obstacle detection in complex scenarios.
  >
  > -- [NVIDIA Isaac Perceptor](https://developer.nvidia.com/isaac/perceptor)

  Personally, I think these GPU-accelerated packages would be released as Isaac ROS packages in the future. This means we can expect more versatile and powerful Isaac ROS packages than currently released packages such as [Isaac ROS Nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) for navigation tasks.

  The developer preview is planned for Q2, 2024. ([source](https://developer.nvidia.com/isaac/perceptor))

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

- Isaac AMR [[link](https://docs.nvidia.com/isaac/doc/index.html)]  
  > Isaac AMR 2.0 features an autonomous navigation stack that includes lidar-based grid mapping, global and continuous localization, a global route planner, a mission client, a behavior planner, and wheel-IMU odometry, as well as a new path and trajectory planner and controller.
  >
  > This release also includes tools for data collection and a cloud-based map creation service. You can use the on-premises data center-based mission control for optimizing route planning with the NVIDIA cuOpt engine and delivering up-to-date maps to the robots.
  >
  > -- [NVIDIA Isaac AMR](https://developer.nvidia.com/isaac/amr)

  Personally, I think Isaac AMR is an extension of Isaac SDK. This is based on the Isaac Sight UI screenshots in [source](https://docs.nvidia.com/isaac/doc/extensions/navigation_stack/doc/navigation_stack_on_isaac_sim.html), and I've noticed some Isaac SDK docs have been redirected/moved to Isaac AMR docs.

- Isaac Manipulator [[link](https://developer.nvidia.com/isaac/manipulator)]
  > a collection of foundation models and modular GPU-accelerated libraries that help build scalable and repeatable workflows for dynamic manipulation tasks by accelerating AI model training and task (re)programming. It’s revolutionizing how robotics software developers can leverage customized software components for specific tasks such as machine tending, assembly tasks, etc., enabling manipulation arms to seamlessly perceive and interact with their surroundings.
  >
  > -- [NVIDIA Isaac Manipulator](https://developer.nvidia.com/isaac/manipulator)

  Personally, I think these GPU-accelerated packages would be released as Isaac ROS packages in the future. For an example, the cuRobo library may be packaged as a cuMotion ROS 2 package. This means we can expect more versatile and powerful Isaac ROS packages than currently released packages such as [Isaac ROS Pose Estimation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/index.html) for manipulation tasks.

  The developer preview is planned for Q2, 2024. ([source](https://developer.nvidia.com/isaac/manipulator))

- Isaac Lab [[link](https://developer.nvidia.com/isaac-sim#isaac-lab)]
  > a lightweight reference application built on the Isaac Sim platform specifically optimized for robot learning and is pivotal for robot foundation model training. Isaac Lab optimizes for reinforcement, imitation, and transfer learning, and is capable of training all types of robot embodiments including the Project GR00T foundation model for humanoids.
  >
  > -- [NVIDIA Isaac Lab](https://developer.nvidia.com/isaac-sim#isaac-lab)

  Isaac Lab is built upon Isaac Sim. However, I'm not sure about its relationship with Omniverse Isaac Gym or Isaac Orbit yet.

- OSMO [[link](https://developer.nvidia.com/osmo)]
  > a cloud-native workflow orchestration platform that lets you easily scale your workloads across distributed environments—from on-premises to private and public cloud. It provides a single pane of glass for scheduling complex multi-stage and multi-container heterogeneous computing workflows.
  >
  > -- [NVIDIA OSMO](https://developer.nvidia.com/osmo)

  OSMO refers to an orchestration platform, which can take a workflow spec and run the specified workloads on Omniverse Cloud or DGX Cloud.

- Project GR00T (Generalist Robot 00 Technology) [[link](https://developer.nvidia.com/project-GR00T)]
  > a general-purpose foundation model that promises to transform humanoid robot learning in simulation and the real world. Trained in NVIDIA GPU-accelerated simulation, GR00T enables humanoid embodiments to learn from a handful of human demonstrations with imitation learning and NVIDIA Isaac Lab for reinforcement learning, as well as generating robot movements from video data. The GR00T model takes multimodal instructions and past interactions as input and produces the actions for the robot to execute.
  >
  > -- [NVIDIA Project GR00T](https://developer.nvidia.com/project-GR00T)

  The GR00T model refers to a general-purpose foundation model for humanoid robots trained in Isaac Lab with imitation learning and reinforcement learning.

- cuOpt [[link](https://developer.nvidia.com/cuopt-logistics-optimization)][[docs](https://docs.nvidia.com/cuopt/index.html)][[ngc](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/cuopt/containers/cuopt)][[dli](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)][[github](https://github.com/NVIDIA/cuOpt-Resources)]  
  a GPU-accelerated solver for [vehicle routing problem](https://en.wikipedia.org/wiki/Vehicle_routing_problem).  
  Formerly _ReOpt_.
  > a GPU-accelerated logistics solver that uses heuristics and optimizations to calculate complex vehicle routing problem variants with a wide range of constraints.
  >
  > -- [NVIDIA cuOpt](https://courses.nvidia.com/courses/course-v1:DLI+T-FX-05+V1/)
  - cuOpt for Isaac Sim [[docs](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/logistics_tutorial_cuopt.html)][[github](https://github.com/NVIDIA/cuOpt-Resources/tree/branch-22.12/cuopt-isaacsim)]
    > a reference for the use of NVIDIA cuOpt to solve routing optimization problems in simulation.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/logistics_tutorial_cuopt.html)

- cuRobo [[link](https://curobo.org/)][[github](https://github.com/NVlabs/curobo)]
  > a CUDA accelerated library containing a suite of robotics algorithms that run significantly faster than existing implementations leveraging parallel compute.
  >
  > -- [NVIDIA cuRobo](https://github.com/NVlabs/curobo)

  cuRobo is shipping as NVIDIA cuMotion with NVIDIA Isaac Manipulator in Q2 2024. ([source](https://curobo.org/))

  - cuRobo with Isaac Sim [[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_curobo.html)]

- (Omniverse) Replicator [[link](https://developer.nvidia.com/nvidia-omniverse-platform/replicator)][[docs](https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_replicator.html)][[blog](https://developer.nvidia.com/blog/build-custom-synthetic-data-generation-pipelines-with-omniverse-replicator/)]  
  a synthetic data generation (SDG) toolkit based on Omniverse.
  > an advanced, extensible SDK to generate physically accurate 3D synthetic data, and easily build custom synthetic data generation (SDG) tools to accelerate the training and accuracy of perception networks.
  >
  > -- [NVIDIA Replicator](https://developer.nvidia.com/nvidia-omniverse-platform/replicator)
  - Isaac Sim Replicator [[docs](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html)]
    > a collection of extensions, python APIs, workflows, and tools such as Replicator Composer that enable a variety of synthetic data generation tasks.
    >
    > -- [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/manual_replicator.html)
  - (Omniverse) Replicator Insight [[link](https://developer.nvidia.com/nvidia-omniverse/replicator-insight-eap)]  
    > an app that enables developers to quickly view, navigate, and inspect their synthetically generated renders.
    >
    > -- [NVIDIA Replicator Insight](https://developer.nvidia.com/nvidia-omniverse/replicator-insight-eap)

- Omniverse Cloud [[link](https://www.nvidia.com/en-us/omniverse/cloud/)]
  > a platform of APIs, SDKs, and services available within a full-stack cloud environment for enterprise developers to easily integrate Universal Scene Description (OpenUSD) and RTX rendering technologies into their 3D industrial digitalization applications.
  >
  > -- [NVIDIA Omniverse Cloud](https://www.nvidia.com/en-us/omniverse/cloud/)

- Omniverse Farm [[docs](https://docs.omniverse.nvidia.com/farm/latest/index.html)]
  > Omniverse Farm Queue and Omniverse Farm Agent allow you to run tasks in the background, and to run automated jobs defined by you or others.
  >
  > -- [NVIDIA Omniverse Farm](https://docs.omniverse.nvidia.com/farm/latest/index.html)

  Omniverse Farm is actually flexible enough to run arbitrary tasks, not just rendering tasks. See the [j3soon/omni-farm-isaac](https://github.com/j3soon/omni-farm-isaac) repository for more information.

## OpenUSD Coding References

- [API Documentation](https://openusd.org/release/apiDocs.html)
- [USD Tutorials](https://openusd.org/release/tut_usd_tutorials.html)
- [OpenUSD Developer Tutorials](https://docs.omniverse.nvidia.com/workflows/latest/openusd-developer.html)
- [NVIDIA-Omniverse/OpenUSD-Code-Samples](https://github.com/NVIDIA-Omniverse/OpenUSD-Code-Samples)

## Omniverse Kit Coding References

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

## (Omniverse) Isaac Sim Coding References

- [Python Environment Installation](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html)
- [Python Environment](https://docs.omniverse.nvidia.com/isaacsim/latest/manual_standalone_python.html)
- [Debugging With Visual Studio Code](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_python_debugging.html)
- [Isaac Sim Workflows](https://docs.omniverse.nvidia.com/isaacsim/latest/introductory_tutorials/tutorial_intro_workflows.html)
- [Extension Workflow for RL](https://docs.omniverse.nvidia.com/isaacsim/latest/isaac_gym_tutorials/tutorial_gym_extension_workflow.html)
- [Isaac Sim: Extensions API](https://docs.omniverse.nvidia.com/py/isaacsim/index.html)
- [Isaac Examples Menu](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_material/menu_examples.html) (in UI through Extensions)
- [Tips & Useful Links](https://docs.omniverse.nvidia.com/isaacsim/latest/reference_material/reference_tips.html)
- Search under `~/.local/share/ov/pkg/isaac_sim-2023.1.1` for example API usages.

## Others

Please [open an issue](https://github.com/j3soon/nvidia-isaac-summary/issues) if you have spotted any errors or have questions regarding this document. For questions regarding the Isaac components, I recommend first going through the [Known Issues](https://docs.omniverse.nvidia.com/isaacsim/latest/known_issues.html), then considering asking in the [NVIDIA Developer Forums](https://forums.developer.nvidia.com/) under the [Isaac topic](https://forums.developer.nvidia.com/c/isaac-sdk/15).

I have documented some bug fixes and workarounds for Isaac in the [j3soon/isaac-extended](https://github.com/j3soon/isaac-extended) repository. I recommend also checking out that repository for reference.

Last updated on 2024/04/26.
