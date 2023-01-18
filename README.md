# Protobuf based rosidl typesupport

Middleware agnostic ros2 static typesupport which uses Protobuf for serialization/deserialization.

## rosidl_typesupport_protobuf is looking for a maintainer

We (the [Eclipse eCAL Team](https://projects.eclipse.org/projects/automotive.ecal/who)) have reached our maximum capacity. Therefore, we currently cannot support this project as well as we would like to and **need your help**! 🫵

You can help by:
- Testing it with the latest ROS versions
- Creating Pull requests
- Answering other users' questions

If you would like to actively maintain this project, feel free to contact us 😊

_We are not abandoning this project and are happy to help and resolve any eCAL related issues._

## Build instructions

* Clone latest release of this repository into your [ROS2 workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
* Source ROS2 `source /path/to/your/ros/distro/folder/setup.bash`
* Run `colcon build` from your workspace folder
* Setup your workspace `source /path/to/your/workspace/install/setup.bash`

**Note**: Typesupport must be built and sourced before any project messages are built.  
**Note 2**: If you are using messages that are not being built by project (ex. builtin_interfaces) they need to be rebuilt and sourced in your workspace.
