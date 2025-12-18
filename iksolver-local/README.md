# Inverse Kinematics Solver — Local CLI Version

This repository contains both a Streamlit-based interactive demo and a local implementation of the inverse kinematics solver.  
The local solver contains a purely terminal-based command line interface (CLI) platform `ik_cli` and a graphical forward kinematics visualizer coupled with an inverse kinematics solver.  
It is intended as a **lightweight, reproducible version** of the solver that runs entirely on a local machine without any web framework or remote deployment.

---

## Overview

The solver models a planar multi-link kinematic chain and iteratively updates joint angles to move the end-effector toward a target position.

This version emphasizes:
- Core IK logic
- Minimal dependencies
- Visualization using Matplotlib

---

## Differences from the Streamlit Version

- Runs entirely from the command line
- No UI or web framework
- Direct Matplotlib plotting
- Focus on solver behavior rather than interaction

---

## Requirements
```
matplotlib
numpy
math
```

