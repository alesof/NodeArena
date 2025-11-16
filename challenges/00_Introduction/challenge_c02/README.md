# Challenge C02 — Subscriber Trigger 🔐

Time to learn about interactive communication! Nodes can react to your messages.

## 🎯 Objective
Trigger a node to release the flag by sending the correct message to its input topic.

## 📖 Story
The flag is locked away inside a node that's waiting for a specific password. Send the right message to the right topic, and the node will publish the flag on another topic. It's like knocking on the right door with the right secret phrase!

## 🧩 Your Mission
1. Navigate to the challenge directory
2. Source the workspace
3. Run the challenge node
4. Discover the input topic the node is listening to
5. Find out what message format is expected
6. Send the correct message to trigger the flag
7. Listen to the output topic to capture the flag

## 🛠️ Commands You'll Need

### Navigate to the challenge
```bash
cd challenge2
```

### Source the Workspace
```bash
source install/setup.bash
```

### Running the Challenge Node
The package name is `challenge_c02`. Find the executable using TAB:
```bash
ros2 run challenge_c02 <press TAB>
```

### Discovering Topics
List all active topics to find which ones this challenge uses:
```bash
ros2 topic list
```

Look for topics with names related to "c02". There should be at least one for input!

### Getting Topic Information
To learn more about a specific topic (like what type of messages it expects), use:
```bash
ros2 topic info <topic_name>
```

This shows you the message type being used on that topic.

### Publishing to a Topic
To send a message to a topic from the command line:
```bash
ros2 topic pub <topic_name> <message_type> '<message_data>'
```

For string messages, the format looks like:
```bash
ros2 topic pub /example_topic std_msgs/msg/String "data: 'your message here'"
```

**Tip:** After running the pub command, press `Ctrl+C` to stop publishing (unless you only want to send it once - see the hint below!)

### Listening to Topics
Use the echo command to listen for the flag on the output topic:
```bash
ros2 topic echo <topic_name>
```

## 📚 What You're Learning

### Subscribers: The Listeners
While publishers broadcast messages, **subscribers** listen and react to them. Think of it as:
- **Publisher**: "I'm announcing something to anyone who cares"
- **Subscriber**: "I'm waiting for specific information, and I'll do something when I hear it"

### Reactive Programming
This challenge demonstrates **event-driven behavior**:
1. The node waits for a specific message
2. When it receives the correct input, it performs an action (publishing the flag)
3. If the input is wrong, it gives you feedback

This pattern is everywhere in robotics:
- A robot arm waits for position commands
- A navigation system reacts to obstacle detections
- A drone responds to flight control messages

### Message Types
Every topic has a **message type** that defines the structure of the data. In this challenge, you're working with `std_msgs/msg/String`, which is a simple text message. Understanding message types is crucial for proper communication between nodes.

### Bidirectional Communication
Unlike Challenge C01 where you only listened, here you're both **sending** and **receiving** messages. This is how complex robotic systems coordinate - nodes talk back and forth to accomplish tasks.

## 💡 Hints
- You'll need at least TWO terminal windows: one for the node, one for your commands
- Try THREE terminals for convenience: node, publishing, and echoing
- The node will give you feedback if your message is incorrect
- Look carefully at the topic names - they tell you their purpose
- To send a message just once instead of continuously, add the `--once` flag to `ros2 topic pub`
- The trigger word is the flag from challenge_c00!

---

*Interactive communication unlocked! You're becoming a ROS2 expert. �