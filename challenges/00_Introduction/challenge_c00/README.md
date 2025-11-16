# Challenge C00 — Node Runner 🚀

Welcome to your first ROS2 challenge! This is where your journey into robotics begins.

## 🎯 Objective
Launch a ROS2 node and discover the hidden flag in its console output.

## 📖 Story
You've just joined a robotics team, and your first task is simple: run a diagnostic node that's been prepared for you. Somewhere in its output lies a secret flag. Can you find it?

## 🧩 Your Mission
1. Navigate to the challenge directory
2. Build the ROS2 workspace
3. Launch the node
4. Observe the console output carefully
5. Find and submit the flag

## 🛠️ Commands You'll Need

### Navigate to the challenge
```bash
cd challenge0
```

### Source the Workspace
Before running any ROS2 commands, you need to "source" the workspace:
```bash
source install/setup.bash
```

**What does sourcing do?**  
When you source the setup file, you're telling your terminal where to find all the ROS2 packages and executables in this workspace. Think of it like adding the workspace to your terminal's "address book" so it knows where everything is located.

**Why is this important?**  
Without sourcing, your terminal won't recognize the `challenge_c00` package or any other packages in the workspace. You'll get a "package not found" error!

**Pro tip:** You need to source the workspace in every new terminal window you open. It's a good habit to source right after navigating to your workspace.

### Running ROS2 Nodes
To execute a ROS2 node, you use the `ros2 run` command with this syntax:
```bash
ros2 run <package_name> <executable_name>
```

- **package_name**: The name of the ROS2 package containing the node
- **executable_name**: The name of the node you want to run

**Package Naming Convention:**  
All challenges follow the pattern `challenge_c<number>`. For this challenge, the package name is `challenge_c00`.

**Finding the Executable:**  
After typing the package name, press **TAB** on your keyboard. The terminal will show you all available executables in that package! This autocomplete feature is your best friend in ROS2.

Try it:
```bash
ros2 run challenge_c00 <press TAB here>
```

**Tip:** The flag will appear in the terminal output. Look for a string that stands out!

## 📚 What You're Learning

### ROS2 Nodes: The Building Blocks
A ROS2 **node** is like a mini-program that performs a specific task. Think of it as a specialized worker in a factory:
- Each node has a **unique name** (like an employee ID)
- Nodes can **communicate** with each other through topics, services, and parameters
- Multiple nodes work together to create complex robotic behaviors

### Why This Matters
Every robot you'll ever program starts with a node. Whether you're controlling a drone, a self-driving car, or a robot arm, understanding nodes is your foundation.

## 💡 Hints
- The flag format is typically: `FLAG{...}`
- All the information you need is in the console output
- Look for the executable name in the challenge description!

---

*Good luck, and welcome to NodeArena! 🎮*