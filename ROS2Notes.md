# ROS2 Lessons

## Colcon Setup
```bash
sudo apt install python3-colcon-common-extensions
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
```

### .zshrc File Configuration
```bash
# argcomplete for ROS2 & Colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

### Source ROS2 and Colcon
```bash
source /opt/ros/humble/setup.zsh
source /home/bbuyukyuksel/Projects/ros2_ws/install/setup.zsh
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
```

---

## Create Workspace

```bash
$ mkdir ros2_ws
```

Create a `src` directory inside the workspace:
```bash
$ mkdir src
```

Test Colcon build:
```bash
$ colcon build
```

---

## Create Python Package

```bash
cd src
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```

## Create C++ Package

```bash
cd src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

---

## Build Packages

```bash
cd ros2_ws
colcon build
```

Or, to build a specific package:
```bash
colcon build --packages-select my_py_pkg
colcon build --packages-select my_cpp_pkg
```

---

## Understanding ROS2 Nodes

ROS2 applications consist of subprograms (nodes), each responsible for a specific task. Nodes communicate through topics, services, and parameters.

### Benefits:
- Reduced code complexity
- Fault tolerance
- Support for Python and C++
- All node names must be unique

---

## Create a Python Node

### Python Node Structure

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello ROS2")
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info(f"Hello" + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Python Package

```bash
ros2 run my_py_pkg py_node
```

Or directly:
```bash
ros2_ws/install/my_py_pkg/lib/my_py_pkg/py_node
```

---

## Create a C++ Node

### C++ Node Structure

```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("cpp_test"), counter_{0} {
        RCLCPP_INFO(this->get_logger(), "Hello CPP Node");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

    void timerCallback() {  
        ++counter_;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### CMake File (C++ Node)

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS
 cpp_node
 DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## Check Available Nodes

```bash
$ ros2 node list
/py_test
/cpp_test
```

Node info:
```bash
$ ros2 node info /py_test
```

Rename a node at runtime:
```bash
$ ros2 run my_py_pkg py_node --ros-args --remap __node:=abc
```

---

## Colcon Commands

Build all packages:
```bash
$ colcon build
```

Build specific package:
```bash
$ colcon build --packages-select my_py_pkg
```

Symlink install (useful for Python):
```bash
$ colcon build --packages-select my_py_pkg --symlink-install
```

With `--symlink-install`, the Python file is directly run without recompiling after each change:
```bash
$ ros2 run my_py_pkg py_node
```

---

## Visualization with rqt

Open rqt for node visualization:
```bash
rqt -> plugins -> introspection -> node graph
```

Or use:
```bash
rqt_graph
```

---

## TurtleSim

Install TurtleSim:
```bash
$ sudo apt install ros-humble-turtlesim
```

Run TurtleSim:
```bash
$ ros2 run turtlesim turtlesim_node
```

---

## Python Publisher

Show interface:
```bash
$ ros2 interface show example_interfaces/msg/String
```

List topics:
```bash
$ ros2 topic list
```

Listen to topic:
```bash
$ ros2 topic echo /robot_news
```

---

## Python Subscriber

Topic info:
```bash
$ ros2 topic info /robot_news
$ ros2 topic hz /robot_news
$ ros2 topic bw /robot_news
```

Publish to topic:
```bash
$ ros2 topic pub -r 10 /robot_news example_interfaces/msg/String “{data: ‘hello from terminal’}”
```

Remap a topic at runtime:
```bash
$ ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station
```

Remap topic name:
```bash
$ ros2 run my_py_pkg robot_news_station --ros-args -r robot_news:=my_news
$ ros2 run my_py_pkg smartphone --ros-args -r robot_news:=my_news
```

---

## Conclusion

This guide covers the basic setup for ROS2, Colcon, Python/C++ node creation, building packages, and useful debugging/visualization tools. Experiment with different nodes and topics to further explore the functionality of ROS2.

