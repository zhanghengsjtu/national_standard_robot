## Connection Between Different Simulators

### [Gazebo + Omniverse](https://github.com/gazebosim/gz-omni)

推荐等级 AAAA

Features
- Ignition -> IsaacSim
    * Move/rotate models
    * Create/remove models
    * Joints
    * Sensors (lidar, cameras)
- IsaacSim -> Ignition
    * Create/remove models

### [UE4 + AirSim](https://github.com/ethz-asl/unreal_airsim)

推荐等级 AAA

This repo contains simulation tools and utilities to perform realistic simulations base on Unreal Engine (UE4), using microsoft AirSim as interface to UE4.

### [UE4 + Mujoco](https://github.com/HoangGiang93/URoboViz)

推荐等级 AAA

Unreal plugin for robot visualisation using ROS connecting with WebSockets. This plugin supports controlling robots built on SkeletalMeshActor and switching physics with an external physics engine MuJoCo.

Features
- Combine photorealistic graphics from Unreal Engine with advanced physics engine from MuJoCo in runtime
- Spawning and Destroying objects in MuJoCo world using Overlap Events from Unreal Engine in runtime
- Robot control using ROS as middleware
- Run on both latest version of Unreal Engine 4.27 and 5.0.0

### [Unity + Mujoco](https://mujoco.readthedocs.io/en/latest/unity.html)

推荐等级 AAAAA

官方支持的插件，[效果看着还行](https://www.roboti.us/book/unity.html)。

The MuJoCo Unity plug-in allows the Unity Editor and runtime to use the MuJoCo physics engine. Users can import MJCF files and edit the models in the Editor. The plug-in relies on Unity for most aspects – assets, game logic, simulation time – but uses MuJoCo to determine how objects move, giving the designer access to MuJoCo’s full API.



## Connection Between ROS and Different Simulators

### [UE4 + ROS](https://github.com/ethz-asl/unreal_cv_ros)

推荐等级 AAA

unreal_cv_ros is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an Unreal Engine 4 (UE4) game. The node-game communcation is carried out utilizing the UnrealCV computer vision plugin for UE4.

### [rclUE: UE4 + ROS2](https://rclue.readthedocs.io/en/devel/)

推荐等级 AAAAA

rclUE is a ROS2 client library for Unreal Engine. rclUE is a UE Plugin which bridges Unreal Engine and ROS2, opening UE’s tools to ROS developers through a C++ interface. Furthermore non-ROS engineers should be able to quickly set up robot simulations through UE’s blueprint visual scripting language by using the instruments that ROS developers and the plugin exposed.

### [Unity + ROS](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

推荐等级 AAAA

This is a central repository for tools, tutorials, resources, and documentation for robotic simulation in Unity. Some introduction to [ros and unity](https://icave2.cse.buffalo.edu/resources/sensor-modeling/ROS%20and%20Unity.pdf).

#### [Pick-and-Place Tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/README.md)

A complete end-to-end demonstration, including how to set up the Unity environment, how to import a robot from URDF, and how to set up two-way communication with ROS for control.

#### [Object Pose Estimation Tutorial](https://github.com/Unity-Technologies/Robotics-Object-Pose-Estimation)

A complete end-to-end demonstration in which we collect training data in Unity and use that data to train a deep neural network to predict the pose of a cube. This model is then deployed in a simulated robotic pick-and-place task.

#### [Articulations Robot Demo](https://github.com/Unity-Technologies/articulations-robot-demo)

A robot simulation demonstrating Unity's new physics solver (no ROS dependency).

#### [Navigation 2 SLAM Example](https://github.com/Unity-Technologies/Robotics-Nav2-SLAM-Example)

An example simulation environment, integrated with ROS 2 and [New!] Visualizations, which enables the exercise of ROS 2's Navigation 2 and slam_toolbox packages using a simulated Turtlebot 3.


### [Blender + ROS](https://github.com/ahmad-aljabali/ROS-Blender-Addon)

推荐等级 AAAA

Add-on to use Blender for visualization with ROS (Robot Operating System).



## Others

### [SEAN2.0](https://sean.interactive-machines.com/)

The Social Environment for Autonomous Navigation (SEAN) 2.0 is a high fidelity, extensible, and open source simulation platform designed for the fair evaluation of social navigation algorithms.

SEAN2.0 is based on Unity and ROS.

### [SimBenchmark](https://leggedrobotics.github.io/SimBenchmark/)

SimBenchmark provides benchmark results of contact simulation on the state-of-the-art physics engines for various robotic tasks.

### Robsuite(https://robosuite.ai/)

### iGibson(https://svl.stanford.edu/igibson/)

推荐等级 AAAA

iGibson is a simulation environment providing fast visual rendering and physics simulation based on Bullet. iGibson is equipped with fifteen fully interactive high quality scenes, hundreds of large 3D scenes reconstructed from real homes and offices, and compatibility with datasets like CubiCasa5K and 3D-Front, providing 12000+ additional interactive scenes. Some of the features of iGibson include domain randomization, integration with motion planners and easy-to-use tools to collect human demonstrations. With these scenes and features, iGibson allows researchers to train and evaluate robotic agents that use visual signals to solve navigation and manipulation tasks such as opening doors, picking up and placing objects, or searching in cabinets.