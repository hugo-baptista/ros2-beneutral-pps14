# ROS2 Tutorial
Based on the [ROS 2: Humble Documentation](https://docs.ros.org/en/humble/index.html).

<details><summary>Installation</summary>

First, I installed `WSL` using `Windows PowerShell`, opening it as administrator and using the command:
```
wsl --install
```

Then, I installed [Ubuntu 22.04.3 LTS](https://apps.microsoft.com/detail/9PN20MSR04DW?hl=en-us&gl=US), opened and configured it.

Lastly, I followed the steps available on the [ROS 2: Humble Ubuntu (Debian packages) Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#).
</details>

## ROS2 Elements
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

`Packages` are the organizational units of the ROS 2 code. Package creation in ROS 2 uses `ament` as its build system and `colcon` as its build tool. **They allow users to install their code and share it** with others easily.

It's possible to create nodes, parameters, topics, services and actions using either **Python or CMake**, which are officially supported but there are other build types created by the community.

**Only CMake is officially supported to build interfaces**. The **best practice is to declare interfaces in a dedicated package** to be used by other packages, although it is possible to create and use an interface in one package (in this case, [ament_cmake_python](https://github.com/ament/ament_cmake/tree/humble/ament_cmake_python) is a useful package to use Python libraries and nodes in a CMake package).
</details>

## ROS2 Commands
<details><summary>General Commands</summary>

- View the ROS graph:
```
rqt_graph
```

- Control the ROS2 Elements (followed by `Plugins > Services > Service Caller`):
```
rqt
```

- View logs:
```
ros2 run rqt_console rqt_console
```

- Identify issues:
```
ros2 doctor
```
or (with full report)
```
ros2 doctor --report
```
</details>

<details><summary>Node Commands</summary>

- Start a node:
```
ros2 run <package-name> <executable-name>
```
or (changing starting parameters)
```
ros2 run <package-name> <executable-name> --ros-args --remap <parameter-name>:=<value>
```
or (loading starting parameters from a file)
```
ros2 run <package-name> <executable-name> --ros-args --params-file <file-path>
```
or (setting default logger level - `FATAL`, `ERROR`, `WARN`, `INFO`, `DEBUG`)
```
ros2 run <package-name> <executable-name> --ros-args --log-level <level>
```

- List active nodes:
```
ros2 node list
```

- View node information:
```
ros2 node info <node-name>
```
</details>

<details><summary>Parameter Commands</summary>

- List parameters:
```
ros2 param list
```

- View the type and current value of a parameter:
```
ros2 param get <node-name> <parameter-name>
```

- View all the parameters of a node:
```
ros2 param dump <node-name>
```

- Change the parameters of a node:
```
ros2 param set <node-name> <parameter-name> <value>
```

- Save the parameters of a node on a file:
```
ros2 param dump <node-name> > <file-name>.yaml
```

- Load the parameters of a file on a node:
```
ros2 param load <node-name> <file-path>
```
</details>

<details><summary>Topic Commands</summary>

- List topics:
```
ros2 topic list
```
or (with their types)
```
ros2 topic list -t
```

- View topic information:
```
ros2 topic info <topic-name>
```

- View messages passing through a topic:
```
ros2 topic echo <topic-name>
```

- View topic interface:
```
ros2 interface show <topic-type>
```

- View topic frequency:
```
ros2 topic hz <topic-name>
```

- Publish a message on a topic:
```
ros2 topic pub <topic-name> <topic-type> "<arguments>"
```
or (only once)
```
ros2 topic pub --once <topic-name> <topic-type> "<arguments>"
```
or (with a frequency, in Hz)
```
ros2 topic pub --rate <frequency> <topic-name> <topic-type> "<arguments>"
```
</details>

<details><summary>Service Commands</summary>

- List services:
```
ros2 service list
```
or (with their types)
```
ros2 service list -t
```

- View service type:
```
ros2 service type <service-name>
```

- View all services of a specific type:
```
ros2 service find <service-type>
```

- View service interface:
```
ros2 interface show <service-type>
```

- Call a service:
```
ros2 service call <service-name> <service-type> <arguments>
```
</details>

<details><summary>Action Commands</summary>

- List actions:
```
ros2 action list
```
or (with their types)
```
ros2 action list -t
```

- View action information:
```
ros2 action info <action-name>
```

- View action interface:
```
ros2 interface show <action-type>
```

- Send a goal
```
ros2 action send_goal <action-name> <action-type> <values>
```
</details>

<details><summary>Bag Command</summary>

- Record topic:
```
ros2 bag record [<topic-names>]
```
or (saving in a custom file)
```
ros2 bag record -o <file-name> [<topic-names>]
```

- View recording information:
```
ros2 bag info <file-name>
```

- Replay recording:
```
ros2 bag play <file-name>
```
or (select topics)
```
ros2 bag play <file-name> --topics [<topic-names>]
```

- View messages passing through a topic while the recording is being replayed:
```
ros2 topic echo <topic-name>
```
</details>

## Building Packages
<details><summary>Errors</summary>

- While building the [examples package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html), the error `c++: fatal error: Killed signal terminated program cc1plus compilation terminated.` occurred. This indicates that the building process is consuming too many resources, so it had to me aborted. Therefore, I limited the amount of CPU cores that the process had available to 1, using the command:
```
export MAKEFLAGS="-j1"
```

- Couldn't setup the [colcon_cd](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#setup-colcon-cd) and the [colcon tab completion](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#setup-colcon-tab-completion).
</details>

<details><summary>Creating a Workspace</summary>

- Create the workspace directory:
```
mkdir -p <path>/<workspace-name>/src
```

- Verify dependencies (in the `<workspace-name>` directory):
```
rosdep install -i --from-path src --rosdistro humble -y
```

- Add the necessary packages (in the `src` directory) and build them (in the `<workspace-name>` directory):
```
colcon build --symlink-install
```
or
```
colcon build --symlink-install --packages-select [<package-names>]
```
Usually the built artifacts (executables, libraries, etc.) are copied to the install space. But the `--symlink-install` option creates symbolic links (symlinks) to the build space instead of copying the files.

This is useful in development scenarios where changes in the source code immediately affect the installed files without the need to rebuild and reinstall. This may, however, have implications for distribution and deployment, so it's important to consider the context and requirements of the specific use case.

- Test the built package:
```
colcon test
```

- Source the environment:
```
source install/setup.bash
```

- During the tutorial, I created the `tutorial-ws` workspace.
</details>

<details><summary>Creating a Python Package</summary>

- In the `src` directory:
```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package-name>
```

- Update the fields `maintainer`, `maintainer_email`, `description` and `license` from the `package.xml` and `setup.py` files (they have to be the same in both of them).

- Every time a executable is created or removed (from the `<package-name>` directory), it is necessary to update the [dependencies](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) in the `package.xml` file and add the executable to `setup.py > entry_points > console_scripts` as `'<executable-name> = <package-name>.<file-name>:main',`  (here, `<executable-name>` is the name that ROS will recognize and `<file-name>` is the `.py` file).

- The `my_package`, `py_pubsub`, `py_srvcli` and `py_action` packages have executables that write `Hello World` in the Command Prompt, that communicate through a `topic`, that communicate through a `service` and that communicate through an `action`, respectively.
</details>

<details><summary>Creating an Interface Package (.msg, .srv and .action)</summary>

- Create a CMake package:
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <interface-name>
```

- In the package's directory, create 3 subdirectories (`msg`, `srv` and `action`):
```
mkdir msg srv action
```

- In the `msg` directory put the `.msg` files (for topics) with this formatting:
```
<parameter-type> <parameter-name>
```

- In the `srv` directory put the `.srv` files (for services) with this formatting:
```
<parameter-type> <parameter-name>     (call)
---
<parameter-type> <parameter-name>     (response)
```

- In the `action` directory put the `.action` files (for actions) with this formatting:
```
<parameter-type> <parameter-name>     (request/goal)
---
<parameter-type> <parameter-name>     (result)
---
<parameter-type> <parameter-name>     (feedback)
```

- After that, it is necessary to update the `CMakeLists.txt` file with all required packages (before the `if(BUILD_TESTING)` line):
```
find_package(rosidl_default_generators REQUIRED)
find_package(<required-package> REQUIRED)   # optional, only if packages were used

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/<msg-name>.msg"
  "srv/<srv-name>.srv"
  "action/<action-name>.action"
  DEPENDENCIES <required-package>           # optional, only if packages were used
)
```

- Update `package.xml` with the dependencies:
```
<depend><required-package></depend>         # optional, only if packages were used
<depend>action_msgs</depend>                # optional, only if actions were used

<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- To use the interface, in another package, add the `<exec_depend><package-name></exec_depend>` to the `package.xml` file and import it as `from <interface-name>.msg import <msg-name>` or `from <interface-name>.srv import <srv-name>`.

- The executables from `py_pubsub` and `py_srvcli` were changed to use the `tutorial_interfaces` interface.

- The best practice is to declare interfaces in a dedicated package to be used by other packages, although it is possible to create and use an interface in one package (in this case, [ament_cmake_python](https://github.com/ament/ament_cmake/tree/humble/ament_cmake_python) is a useful package to use Python libraries and nodes in a CMake package).
</details>

<details><summary>Creating Launch files with custom Parameters</summary>

- Create a `launch` directory in the package, there create the launch files with custom parameters:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package-name>',
            executable='<executable-name>',
            name='<custom-executable-name>',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'<parameter>': '<value>'}
            ]
        )
    ])
```

- Then, update the `setup.py` file with:
```
import os
from glob import glob
# ...

setup(
  # ...
  data_files=[
      # ...
      (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ]
  )
```

- Build the package and launch the custom executable:
```
ros2 launch <package-name> <custom-executable-name>
```

- The `python_parameters` package has the result from the [tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html#).
</details>

<details><summary>Managing Dependencies with rosdep</summary>

- `rosdep` is a tool that identifies and installs the dependencies that it finds in the `package.xml` files.
```
rosdep install --from-paths src -y --ignore-src
```
or
```
rosdep install -i --from-path src --rosdistro humble -y
```
</details>

# AIML_bag_191223 file
The `AIML` directory contains the bag file (`AIML_bag_191223`) and the required interfaces (`src > ccpm_msgs`).

<details><summary>Replaying the bag file and viewing the data</summary>

- After building the workspace and sourcing the environment, we can see the bag file's metadata with the command:
```
'ros2 bag info AIML_bag_191223
```
This displays a lot of information, such as the size of the recording, the duration, the start and end time, the number of messages recorded and a list of the recorded topics, as well as their type of messages.

- Focusing on the `/gps/receive` topic, it uses the interface `ccpm_msgs/msg/GPSMessage`:
```
std_msgs/Header header
float64 latitude
float64 longitude
float64 altitude
float32 accuracy
float32 speed
float32 heading
uint8 tracking
uint8 gps_fix
uint8 status
```

- To see the messages that pass through that topic, first it is needed to start the bag file's replay:
```
ros2 bag play AIML_bag_191223 --topics /gps/receive -p
```
And then, in a different Command Line, it is needed to echo the topic:
```
ros2 topic echo /gps/receive > gps_receive.txt
```
Using the `> gps_receive.txt` command, all the messages that pass through that topic are stored in the `gps_receive.txt` file.
</details>

## Convert Rosbag file to Pandas dataframe