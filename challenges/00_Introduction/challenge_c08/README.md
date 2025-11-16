# Challenge C08 — Service Unlock + Topic Validation

## 🏁 Objective
Call a service that gives you a challenge word, then publish that word to a topic to receive the flag.

## 🧩 Task
Find the service endpoint that returns the challenge word. Publish that word on the node's response topic — when the node receives the correct word, it will publish the flag.

## Explanation
This exercise mixes synchronous (service) and asynchronous (topic) interactions. You will:
- Discover and call a service to receive a one-time challenge word.
- Use that word as a payload on a topic the node listens to.
- The node validates the message and, if correct, publishes the flag.

This pattern resembles challenge-response flows in distributed systems and teaches how different ROS2 communication primitives can be combined.
