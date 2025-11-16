# Challenge C06 — Parameter-Gated Topic

## 🏁 Objective
A node publishes an encoded message. Inspect the node's parameters to learn the decoding key and recover the flag.

## 🧩 Task
Find the published encoded string and the node parameter that explains how to decode it. Use the information to decode and obtain the flag.

## Explanation
This challenge introduces how parameters and topics can be used together to convey and protect information. The node regularly publishes a message which is a simple Caesar-ciphered version of the real flag. The decoding key is exposed as a node parameter (a runtime configuration value).  

Key learning points:
- **Parameters** can contain metadata or keys that affect how data should be interpreted.
- **Topics** deliver data streams; sometimes the data needs post-processing or decoding.
- By combining parameter inspection with topic observation, you can reconstruct hidden information.
