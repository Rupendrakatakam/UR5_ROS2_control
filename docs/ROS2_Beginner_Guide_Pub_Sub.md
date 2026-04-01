# ROS2 Beginner's Guide: Building Publisher & Subscriber Nodes in C++

## Table of Contents
1. [What is ROS2?](#1-what-is-ros2)
2. [Core Concepts You Need to Know](#2-core-concepts-you-need-to-know)
3. [Workspace & Package Structure](#3-workspace--package-structure)
4. [Understanding the Original Code & Its Mistakes](#4-understanding-the-original-code--its-mistakes)
5. [How We Fixed the Publisher Node (pub.cpp)](#5-how-we-fixed-the-publisher-node-pubcpp)
6. [How We Fixed the Subscriber Node (sub.cpp)](#6-how-we-fixed-the-subscriber-node-subcpp)
7. [Creating Custom Messages (master_ros2 package)](#7-creating-custom-messages-master_ros2-package)
8. [CMakeLists.txt & package.xml Explained](#8-cmakeliststx--packagexml-explained)
9. [Building & Running Your Nodes](#9-building--running-your-nodes)
10. [Complete Final Code Reference](#10-complete-final-code-reference)

---

## 1. What is ROS2?

ROS2 (Robot Operating System 2) is a framework for writing robot software. It provides:
- **Communication** between different programs (called *nodes*)
- **Message passing** using topics (like radio channels)
- **Hardware abstraction** to work with real robots and simulators

Think of ROS2 as a messaging system where programs talk to each other without needing to know about each other directly.

---

## 2. Core Concepts You Need to Know

### Nodes
A **node** is a single executable program that does one job. In our case:
- `pub.cpp` → A node that **sends** messages (publisher)
- `sub.cpp` → A node that **receives** messages (subscriber)

### Topics
A **topic** is like a radio channel. Publishers broadcast on a topic, and subscribers listen to it.
- `"std_string_topic"` → Topic for standard string messages
- `"custom_topic"` → Topic for our custom messages

### Messages
A **message** is the data structure being sent. ROS2 has built-in types like `std_msgs/msg/String`, and you can create your own custom types.

### QoS (Quality of Service)
QoS defines **how** messages are delivered — reliability, history depth, etc. We used:
```cpp
rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local()
```
This means: keep last 10 messages, guarantee delivery, and deliver old messages to late subscribers.

### Callbacks
A **callback** is a function that runs automatically when a message arrives. You don't call it manually — ROS2 calls it for you.

### Timers
A **timer** triggers a function at regular intervals. Our publisher uses a timer to send messages every 1 second.

---

## 3. Workspace & Package Structure

A ROS2 workspace is organized like this:

```
UR5_control_ros2/           ← Workspace root
└── src/                    ← Source code folder
    ├── ur5_control/        ← Your main package
    │   ├── CMakeLists.txt  ← Build instructions
    │   ├── package.xml     ← Package metadata & dependencies
    │   └── src/            ← Your C++ source files
    │       ├── pub.cpp     ← Publisher node
    │       └── sub.cpp     ← Subscriber node
    └── master_ros2/        ← Custom message package (we created this)
        ├── CMakeLists.txt
        ├── package.xml
        └── msg/
            └── CustomMsg.msg  ← Custom message definition
```

---

## 4. Understanding the Original Code & Its Mistakes

### Original pub.cpp Problems:

```cpp
// PROBLEM 1: Method defined INSIDE the constructor
PublishNode() : Node("ur5_joint_state_publisher"){
    ...
    timer_ = this->create_wall_timer(..., std::bind(&PublisherNode::publish_messages, this));
    // Wrong class name: PublisherNode vs PublishNode

    void publish_messages(){  // ← C++ doesn't allow defining functions inside other functions!
        ...
    }
}

// PROBLEM 2: No member variable declarations
// publisher_, custom_publisher_, timer_ were never declared

// PROBLEM 3: Missing semicolon after class
}  // ← Should be };

// PROBLEM 4: No main() function
// Without main(), this cannot be compiled into an executable
```

### Original sub.cpp Problems:

```cpp
// PROBLEM 1: Callbacks defined INSIDE the constructor
SubscribeNode() : Node("ur5_joint_state_subscriber"){
    ...
    void string_callback(...) { }  // ← Cannot define functions inside functions!
    void custom_callback(...) { }
}

// PROBLEM 2: Timer binding to non-existent function
timer_ = this->create_wall_timer(..., std::bind(&SubscribeNode::subscribe_messages, this));
// subscribe_messages() doesn't exist!

// PROBLEM 3: No member variable declarations

// PROBLEM 4: Missing semicolon

// PROBLEM 5: No main() function
```

---

## 5. How We Fixed the Publisher Node (pub.cpp)

### Step-by-Step Fix:

**Step 1: Proper class structure**
In C++, class methods must be declared as members, not defined inside other functions:

```cpp
class PublishNode : public rclcpp::Node {
public:
    PublishNode() : Node("ur5_joint_state_publisher") {
        // Constructor only does initialization
    }

private:
    void publish_messages() {
        // This is a proper member function
    }
};
```

**Step 2: Declare member variables**
Every variable used across methods needs to be declared:

```cpp
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<master_ros2::msg::CustomMsg>::SharedPtr custom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
```

- `SharedPtr` is a smart pointer that manages memory automatically
- `rclcpp::Publisher<MsgType>` is the publisher for a specific message type

**Step 3: Fix the class name typo**
Changed `&PublisherNode::publish_messages` to `&PublishNode::publish_messages`

**Step 4: Add main() function**
Every C++ executable needs a `main()` entry point:

```cpp
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                          // Initialize ROS2
    rclcpp::spin(std::make_shared<PublishNode>());     // Run the node
    rclcpp::shutdown();                                // Clean shutdown
    return 0;
}
```

- `rclcpp::init()` — Starts ROS2 communication
- `rclcpp::spin()` — Keeps the node alive, processing callbacks and timers
- `rclcpp::shutdown()` — Cleans up resources

---

## 6. How We Fixed the Subscriber Node (sub.cpp)

### Step-by-Step Fix:

**Step 1: Move callbacks to proper member functions**
```cpp
private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
    }

    void custom_callback(const master_ros2::msg::CustomMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Custom: %s, %d", msg->data.c_str(), msg->number);
    }
```

- `msg->data` accesses the string field
- `msg->number` accesses the integer field
- `RCLCPP_INFO` prints to the console (like `printf` but ROS2-aware)

**Step 2: Bind callbacks correctly in subscriptions**
```cpp
subscription_ = this->create_subscription<std_msgs::msg::String>(
    "std_string_topic",
    qos_profile,
    std::bind(&SubscribeNode::string_callback, this, std::placeholders::_1)
);
```

- `std::bind` connects the callback to the subscription
- `std::placeholders::_1` means "the first argument (the message) goes here"

**Step 3: Removed unnecessary timer**
A subscriber doesn't need a timer — it reacts to incoming messages via callbacks automatically.

**Step 4: Declare member variables**
```cpp
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<master_ros2::msg::CustomMsg>::SharedPtr custom_subscription_;
```

---

## 7. Creating Custom Messages (master_ros2 package)

### Why Create a Custom Message Package?

ROS2 has standard messages like `String`, `Int32`, etc. But when you need a message with **multiple fields** (like a string AND a number together), you create a custom message.

### Step 1: Create the message definition

File: `master_ros2/msg/CustomMsg.msg`
```
string data
int32 number
```

This is a simple text format. Each line is `type field_name`.

### Step 2: Create package.xml

This declares your package and its dependencies:
```xml
<package format="3">
  <name>master_ros2</name>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

Key points:
- `rosidl_default_generators` — Tool that converts `.msg` files to C++/Python code
- `member_of_group` — Tells ROS2 this package contains message definitions

### Step 3: Create CMakeLists.txt

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)
```

This tells the build system to generate C++ headers from your `.msg` file.

### How It Gets Used

After building, ROS2 generates: `master_ros2/msg/custom_msg.hpp`

You include it like:
```cpp
#include "master_ros2/msg/custom_msg.hpp"
```

And use it like:
```cpp
auto msg = master_ros2::msg::CustomMsg();
msg.data = "Hello";
msg.number = 42;
```

---

## 8. CMakeLists.txt & package.xml Explained

### package.xml (ur5_control)

```xml
<depend>master_ros2</depend>
```

This tells ROS2 that `ur5_control` needs `master_ros2` to be built first.

### CMakeLists.txt (ur5_control)

**Finding dependencies:**
```cmake
find_package(master_ros2 REQUIRED)
```

**Creating executables:**
```cmake
add_executable(pub_node src/pub.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs master_ros2)
```

- `add_executable` — Creates a build target from your `.cpp` file
- `ament_target_dependencies` — Links the required ROS2 libraries

**Installing:**
```cmake
install(TARGETS ur5_control_node pub_node sub_node
  DESTINATION lib/${PROJECT_NAME}
)
```

This makes your nodes discoverable by `ros2 run`.

---

## 9. Building & Running Your Nodes

### Build the workspace:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select master_ros2 ur5_control
```

- `colcon build` — ROS2's build tool
- `--packages-select` — Only build these packages (saves time)

### Source the setup:
```bash
source install/setup.bash
```

This makes your newly built nodes discoverable.

### Run the publisher:
```bash
ros2 run ur5_control pub_node
```

### Run the subscriber (in a new terminal):
```bash
ros2 run ur5_control sub_node
```

### What You'll See:

**Publisher terminal:**
```
[INFO] [ur5_joint_state_publisher]: Publishing: Hello from ROS2
[INFO] [ur5_joint_state_publisher]: Publishing Custom: Custom Hello, 42
[INFO] [ur5_joint_state_publisher]: Publishing: Hello from ROS2
[INFO] [ur5_joint_state_publisher]: Publishing Custom: Custom Hello, 42
...
```

**Subscriber terminal:**
```
[INFO] [ur5_joint_state_subscriber]: Received: Hello from ROS2
[INFO] [ur5_joint_state_subscriber]: Received Custom: Custom Hello, 42
[INFO] [ur5_joint_state_subscriber]: Received: Hello from ROS2
[INFO] [ur5_joint_state_subscriber]: Received Custom: Custom Hello, 42
...
```

---

## 10. Complete Final Code Reference

### pub.cpp — Publisher Node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2/msg/custom_msg.hpp"

class PublishNode : public rclcpp::Node{
    public:
        PublishNode() : Node("ur5_joint_state_publisher"){
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
            
            publisher_ = this->create_publisher<std_msgs::msg::String>("std_string_topic", qos_profile);
            custom_publisher_ = this->create_publisher<master_ros2::msg::CustomMsg>("custom_topic", qos_profile);
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublishNode::publish_messages, this));
        }

    private:
        void publish_messages(){
            auto string_msg = std_msgs::msg::String();
            string_msg.data = "Hello from ROS2";
            RCLCPP_INFO(this->get_logger(), "Publishing: %s", string_msg.data.c_str());
            publisher_->publish(string_msg);

            auto custom_msg = master_ros2::msg::CustomMsg();
            custom_msg.data = "Custom Hello";
            custom_msg.number = 42;
            RCLCPP_INFO(this->get_logger(), "Publishing Custom: %s, %d", custom_msg.data.c_str(), custom_msg.number);
            custom_publisher_->publish(custom_msg);
        }

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<master_ros2::msg::CustomMsg>::SharedPtr custom_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishNode>());
    rclcpp::shutdown();
    return 0;
}
```

### sub.cpp — Subscriber Node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2/msg/custom_msg.hpp"

class SubscribeNode : public rclcpp::Node{
    public:
        SubscribeNode() : Node("ur5_joint_state_subscriber"){
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
            
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "std_string_topic", qos_profile,
                std::bind(&SubscribeNode::string_callback, this, std::placeholders::_1));

            custom_subscription_ = this->create_subscription<master_ros2::msg::CustomMsg>(
                "custom_topic", qos_profile,
                std::bind(&SubscribeNode::custom_callback, this, std::placeholders::_1));
        }

    private:
        void string_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
        }

        void custom_callback(const master_ros2::msg::CustomMsg::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received Custom: %s, %d", msg->data.c_str(), msg->number);
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<master_ros2::msg::CustomMsg>::SharedPtr custom_subscription_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscribeNode>());
    rclcpp::shutdown();
    return 0;
}
```

---

## Summary of Key Lessons

| Mistake | Why It's Wrong | Fix |
|---------|---------------|-----|
| Functions inside constructor | C++ doesn't allow nested function definitions | Move to class members |
| Missing member declarations | Compiler doesn't know what `publisher_` is | Add `Type::SharedPtr name_;` |
| Wrong class name in `std::bind` | `PublisherNode` ≠ `PublishNode` | Match exact class name |
| Missing semicolon after class | C++ syntax requirement | Add `;` after `}` |
| No `main()` function | No entry point for executable | Add `main()` with `rclcpp::init/spin/shutdown` |
| Subscriber with timer | Subscribers react via callbacks, not timers | Remove unnecessary timer |
