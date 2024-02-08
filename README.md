# My ROS2 Documentation on the Be.Neutral PPS14 Project

## ROS2 Tutorial
Based on the [ROS 2: Humble Documentation](https://docs.ros.org/en/humble/index.html).

<details><summary>Installation</summary>

First, I installed `WSL` using `Windows PowerShell`, opening it as administrator and using the command:
```
wsl --install
```

Then, I installed [Ubuntu 22.04.3 LTS](https://apps.microsoft.com/detail/9PN20MSR04DW?hl=en-us&gl=US), opened and configured it.

Lastly, I followed the steps available on the [Ubuntu (Debian packages) Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#).
</details>

### ROS2 Elements
<details><summary>ROS graph, Nodes and Parameters</summary>

The `ROS graph` is a network of ROS 2 elements processing data together at the same time. It encompasses all executables and the connections between them if you were to map them all out and visualize them.

Each `node` in ROS should be **responsible for a single, modular purpose**, e.g. controlling the wheel motors or publishing the sensor data from a laser range-finder. Each node can send and receive data from other nodes via topics, services or actions.

A `parameter` is a configuration value of a node. You can think of parameters as **node settings**. A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.

![ROS graph with nodes](https://github.com/hugo-baptista/images-and-gifs/blob/main/ros2/nodes.gif)
</details>

<details><summary>Topics</summary>

`Topics` are one of the main ways in which data is moved between nodes and therefore between different parts of the system. They follow the **publisher-subscriber model**. A node may publish data to any number of topics and simultaneously have subscriptions to any number of topics. Topics allow nodes to **subscribe to data streams and get continual updates**.

![ROS graph with nodes communicating through a topic](https://github.com/hugo-baptista/images-and-gifs/blob/main/ros2/topic.gif)
</details>

<details><summary>Services</summary>

`Services` are another method of communication for nodes in the ROS graph, based on a **call-and-response model**. While topics allow nodes to subscribe to data streams and get continual updates, services **only provide data when they are specifically called** by a client.

![ROS graph with nodes communicating through a service](https://github.com/hugo-baptista/images-and-gifs/blob/main/ros2/service.gif)
</details>

<details><summary>Actions</summary>

`Actions` are one of the communication types in ROS 2, they use the **client-server model** and are **intended for long running tasks**. They are built on topics and services, consisting of three parts: a goal (service), feedback (topic), and a result (service). These elements allow actions to **provide steady feedback** like topics, **provide data only when they are called** like services, and **allow them to be cancelled**. An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.

![ROS graph with nodes communicating through a topic](https://github.com/hugo-baptista/images-and-gifs/blob/main/ros2/action.gif)
</details>

<details><summary>Interfaces</summary>

`Interfaces` are the message structures that nodes use to communicate with each other, either through topics, services or actions. While **it’s good practice to use predefined interface definitions**, sometimes there's a need to create custom interfaces.

Each communication type has its own interface files: topics use `.msg`, services `.srv` and actions `.action`. The structure of these files are:
- `.msg`:
```
<variable-types> <variable-names>
```
- `.srv`:
```
<variable-types> <variable-names>  (client's call)
---
<variable-types> <variable-names>  (server's response)
```
- `.action`:
```
<variable-types> <variable-names>  (goal)
---
<variable-types> <variable-names>  (result)
---
<variable-types> <variable-names>  (feedback)
```

</details>

<details><summary>Packages</summary>

`Packages` are the organizational units of the ROS 2 code. Package creation in ROS 2 uses ament as its build system and colcon as its build tool. **They allow users to install their code and share it** with others easily.

It's possible to create nodes, parameters, topics, services and actions using either **Python or CMake**, which are officially supported but there are other build types created by the community.

**Only CMake is officially supported to build interfaces**. The **best practice is to declare interfaces in a dedicated package** to be used by other packages, although it is possible to create and use an interface in one package (in this case, [ament_cmake_python](https://github.com/ament/ament_cmake/tree/humble/ament_cmake_python) is a useful package to use Python libraries and nodes in a CMake package).
</details>