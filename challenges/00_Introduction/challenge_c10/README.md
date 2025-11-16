# Challenge C10 — Multi-Node Graph Puzzle

## 🏁 Objective
Use two nodes working together to discover and obtain the final flag.

## 🧩 Task
Discover Node A's hint service, call it to receive a hint, then use that hint by sending it to Node B's topic. If correct, Node B will publish the final flag.

## Explanation
This puzzle requires discovering and interacting with multiple nodes and combining service and topic interactions. Node A provides a hint via a service; Node B expects that hint on a specific topic and will publish the final flag when the correct hint is received.  

Skills reinforced:
- Graph exploration: finding nodes, services, and topics in a multi-node system
- Combining synchronous (service) and asynchronous (topic) communication
- Thinking across the system rather than focusing on a single node
