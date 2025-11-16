# Challenge C05 — Topic Relay

## 🏁 Objective
Make the node relay messages and discover the secret trigger that causes it to publish the flag.

## 🧩 Task
Find the relay input channel and send the right message to cause the node to output the flag.

## Explanation
This challenge combines publishing and subscribing: one node receives messages on an input topic and republishes them on an output topic. A relay pattern is common in robotics where one component forwards or transforms messages for another. Here, the node also watches for a particular trigger message; when that trigger arrives it publishes the flag on the output topic.  

You will practice:
- Inspecting running topics and their flow
- Sending messages that other nodes will react to
- Observing how a node can act both as a subscriber and as a publisher simultaneously
