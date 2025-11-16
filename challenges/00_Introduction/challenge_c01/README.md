# Challenge C01 — Publisher Basics 📡

Your ROS2 journey continues! Time to learn how nodes communicate with each other.

## 🎯 Objective
Retrieve a flag that's being published on a ROS2 topic.

## 📖 Story
A node is broadcasting a secret message continuously. Your job is to listen in on the right channel and capture the flag. But first, you need to find out which channel it's using!

## 🧩 Your Mission
1. Navigate to the challenge directory
2. Source the workspace
3. Run the challenge node
4. Discover which topic the flag is being published on
5. Listen to that topic to retrieve the flag

## 🛠️ Commands You'll Need

### Navigate to the challenge
```bash
cd challenge1
```

### Source the Workspace
Remember to source the workspace so your terminal knows where to find the ROS2 packages:
```bash
source install/setup.bash
```

### Running the Challenge Node
The package name is `challenge_c01`. Use TAB completion to find the executable:
```bash
ros2 run challenge_c01 <press TAB>
```

### Discovering Topics
To see all active topics in the ROS2 system, use:
```bash
ros2 topic list
```

This command shows you all the "channels" that nodes are currently using to communicate. One of these topics contains your flag!

### Listening to a Topic
Once you've identified the correct topic, you can listen to messages being published on it:
```bash
ros2 topic echo <topic_name>
```

Replace `<topic_name>` with the topic you want to listen to. The messages will appear in your terminal in real-time.

**Tip:** Use `Ctrl+C` to stop listening to a topic when you're done.

## 📚 What You're Learning

### Topics: The Communication Channels
A ROS2 **topic** is like a radio frequency - nodes can broadcast messages on it, and other nodes can tune in to listen. Key concepts:
- **Publisher**: A node that sends messages to a topic (like a radio transmitter)
- **Subscriber**: A node that receives messages from a topic (like a radio receiver)
- **Message**: The data being sent (in this case, a string containing the flag)

### Why Topics Matter
Topics enable **decoupled communication** - the publisher doesn't need to know who's listening, and subscribers don't need to know who's sending. This makes ROS2 systems flexible and scalable. A single topic can have multiple publishers and subscribers!

### The Publish-Subscribe Pattern
This challenge demonstrates asynchronous message passing:
1. A node publishes data to a topic at regular intervals
2. You subscribe to that topic using the CLI
3. Messages flow from publisher to subscriber automatically

This is one of the most fundamental patterns in robotics - sensors publish data, and other nodes subscribe to process it.

## 💡 Hints
- The challenge node will tell you it started - that means it's publishing!
- Topic names often relate to what they're publishing
- You need TWO terminal windows: one to run the node, one to echo the topic
- The flag format is: `FLAG{...}`

---

*Keep exploring! You're learning the language of robots. 🤖*