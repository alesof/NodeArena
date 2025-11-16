# Challenge C07 — Multi-Step Unlock

## 🏁 Objective
Perform a sequence of interactions (message → parameter → message) in order to unlock the flag.

## 🧩 Task
Discover the three-step sequence and execute it in the correct order to get the flag.

## Explanation
This challenge simulates workflows where multiple conditions must be satisfied in sequence. A typical robotic system may require a startup handshake, a configuration change, and then a final confirmation.  

Here you will:
- Send a first message to a topic to start the sequence.
- Modify a node parameter to satisfy an intermediate gate.
- Finally, send a second message to complete the sequence.

This enforces reasoning about state, ordering, and multi-modal node interaction (topics + parameters).
