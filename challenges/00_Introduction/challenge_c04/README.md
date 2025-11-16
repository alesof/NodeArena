# Challenge C04 — Service Unlocker

## 🏁 Objective
Retrieve a flag by calling a ROS2 service.

## 🧩 Task
Discover the service and call it to obtain the flag.

## Services and Synchronous Communication
A **service** in ROS2 is a synchronous request/response mechanism between nodes.  
Unlike topics, which are asynchronous streams, a service call waits for a response.  
Nodes can offer services to perform actions or provide information upon request.  
In this challenge, calling the correct service triggers the node to return the flag.  
This introduces the concept of **RPC-like communication** in ROS2, which complements the asynchronous topic-based messaging.
