# Two-Link Inverse Kinematics Solver  
A Geometric Engine for Solving the Inverse Kinematics of a Planar Two-Link Robot Arm<br>

This project is a visual and interactive 2-DOF planar robot arm built with **Streamlit** and **Python**, featuring both a deployed interactive web [app](www.iksolver-app.streamlit.app) and a standalone local Python IK solver (CLI + Matplotlib).

This repository contains all the code needed to study the mathematics, visualization, and implementation of inverse kinematics for a simple 2-link robotic arm.

---

###  Live Demo  
Explore the IK solver directly in your browser:

👉 **https://iksolver-app.streamlit.app/**  

(You may have to click the button to wake the app up if the webpage has been inactive for a while)  

---

### Features:
- Move sliders to manually control joint angles  
- Input desired (x, y) and compute joint angles via IK  
- Realtime 2D visualization of the robot arm  
- Built-in validation for reachable/unreachable targets

---

### Repository Structure  
.<br>
```
├── iksolver-app.py             Streamlit app (web UI + visualization)<br>
├── ik_solver.py                Standalone Python IK + CLI + Matplotlib renderer<br>
├── requirements.txt            Dependencies for local & Streamlit deployment<br>
├── LICENSE                     Project license<br>
└── README.md                   You're reading this :)
```
---

### Overview
I've attempted to show FK and solve IK purely geometrically, and avoiding the use of matrices. <br>
α<sub>1</sub> is defined as +ve CCW from the origin (where link 1 is joined to the surface),  My notation for α<sub>2</sub> is unorthodox though. <br>
α<sub>2</sub> is defined +ve CW from the global x-axis. Meaning both link angles are universally independent and don't depend on the position of the other.  

  
