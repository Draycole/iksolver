# Two-Link Inverse Kinematics Solver  
A Geometric Engine for Solving the Inverse Kinematics of a Planar Two-Link Robot Arm<br>

This project is a visual and interactive 2-DOF planar robot arm built with **Streamlit** and **Python**, featuring both a deployed interactive web [app](www.iksolver-app.streamlit.app) and a standalone local Python IK solver (CLI + Matplotlib).

This repository contains all the code needed to study the mathematics, visualization, and implementation of inverse kinematics for a simple 2-link robotic arm.

---
![ik solver screen recording](<./docs/figures/screenrec_demo.gif>)
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
└── docs                        Documentation
    └── IK_Derivation.pdf       PDF file containing derivation calculations
    └── figures                 Associated diagrams
      └── alpha1                  
      └── alpha2                  
      └── problem setup                
└── iksolver-local              Local version - runs offline
    └── README.md                   
    └── ik_cli.py               CLI script to output α<sub>1</sub> and α<sub>2</sub>
    └── ik_graphical.py         GUI script with matplotlib for angle verification locally.
    └── screenshots             Screenshots      
      └── base_position            
      └── pose1                  
      └── pose2    
└── results                     Assets from verification script
    └── fig4.png                    
    └── fig5.png
    └── results.json
├── iksolver-app.py             Streamlit app (web UI + visualization)
├── ik_solver.py                Standalone Python IK + CLI + Matplotlib renderer
├── ik_verification.py          Verification script 
├── requirements.txt            Dependencies for local & Streamlit deployment
├── LICENSE                     Project license
└── README.md                   You're reading this :)
```
---

### Overview
I've attempted to show FK and solve IK purely geometrically, and avoiding the use of matrices. <br>
α<sub>1</sub> is defined as +ve CCW from the origin (where link 1 is joined to the surface),  My notation for α<sub>2</sub> is unorthodox though. <br>
α<sub>2</sub> is defined +ve CW from the global x-axis. Meaning both link angles are universally independent and don't depend on the position of the other. <br>
All my derivations and calculations are in the docs folder so you can walk through my steps to the solution.  

---

### Using the Local Engine

Clone the repo<br>
```
git clone https://github.com/Draycole/iksolver.git 
cd iksolver
```

Install dependencies<br>
```
pip install streamlit
pip install matplotlib
pip install numpy
```

---

### License 
This project is licensed under the MIT License.
  
