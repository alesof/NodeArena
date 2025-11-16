#!/usr/bin/env python3
import os
import sys

# -----------------------------
# CONFIGURABLE TEMPLATES
# -----------------------------

CMAKE_TEMPLATE = """cmake_minimum_required(VERSION 3.5)
project({project_name})

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${{PROJECT_NAME}} src/${{PROJECT_NAME}}.cpp)

install(TARGETS
    ${{PROJECT_NAME}}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${{PROJECT_NAME}}
)

ament_auto_package()
"""

PACKAGE_XML_TEMPLATE = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>1.0.0</version>
  <description>ROS2 {package_name}</description>
  <maintainer email="alessandro1sofia@gmail.com">Alessandro Sofia</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

CPP_TEMPLATE = """#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {{
public:
    ChallengeNode() : Node("{node_name}") {{
        RCLCPP_INFO(this->get_logger(), "🧩 Challenge {node_name} started!");
        RCLCPP_INFO(this->get_logger(), "Flag: CTF{{{flag}}}");
    }}
}};

int main(int argc, char* argv[]) {{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChallengeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}}
"""

README_TEMPLATE = """# Challenge {node_name} — {title}

## 🏁 Objective
{objective}

## 🧩 Task
{task}

## 💡 Concepts Explained
{concepts}
"""

# -----------------------------
# GENERATOR FUNCTION
# -----------------------------
def create_challenge(ch_id, title, flag="changeme",
                     objective="Solve the challenge to get the flag.",
                     task="Use ROS2 CLI to find the flag.",
                     concepts="ROS2 basics."):
    folder = f"challenge_{ch_id}"
    src_folder = os.path.join(folder, "src")
    os.makedirs(src_folder, exist_ok=True)
    
    # CMakeLists.txt
    with open(os.path.join(folder, "CMakeLists.txt"), "w") as f:
        f.write(CMAKE_TEMPLATE.format(project_name=folder))
    
    # package.xml
    with open(os.path.join(folder, "package.xml"), "w") as f:
        f.write(PACKAGE_XML_TEMPLATE.format(package_name=folder))
    
    # src cpp
    cpp_file = os.path.join(src_folder, f"{folder}.cpp")
    with open(cpp_file, "w") as f:
        f.write(CPP_TEMPLATE.format(node_name=ch_id, flag=flag))
    
    # README.md
    readme_file = os.path.join(folder, "README.md")
    with open(readme_file, "w") as f:
        f.write(README_TEMPLATE.format(
            node_name=ch_id,
            title=title,
            objective=objective,
            task=task,
            concepts=concepts
        ))
    
    print(f"✅ Challenge template created at {folder}")

# -----------------------------
# MAIN
# -----------------------------
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 generate_challenge.py <id> <title> [flag]")
        sys.exit(1)
    ch_id = sys.argv[1]
    title = sys.argv[2]
    flag = sys.argv[3] if len(sys.argv) > 3 else "changeme"
    create_challenge(ch_id, title, flag)
