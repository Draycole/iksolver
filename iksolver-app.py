import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import math


L = 5  # link length

def forward_kinematics(alpha1, alpha2):
    # to radians
    a1_rad = np.radians(alpha1)
    a2_rad = np.radians(alpha2)
    
    # j1 position
    x1 = L * np.cos(a1_rad)
    y1 = L * np.sin(a1_rad)
    

    # alpha2 is CW from absolute horizontal at joint1
    theta2_abs = -a2_rad  # Convert CW to CCW for calculation
    x2 = x1 + L * np.cos(theta2_abs)
    y2 = y1 + L * np.sin(theta2_abs)
    
    return (x1, y1), (x2, y2)

def inverse_kinematics(x, y):
    
    try:
        
        t2 = np.arccos(1 - ((x**2 + y**2) / (2 * L**2)))
        t1 = np.arccos(np.sqrt(x**2 + y**2) / (2 * L))
        
        # alpha 1 (absolute angle of link 1)
        a1 = math.degrees(t1 + math.atan2(y, x))
        
        # compute j1 coordinates
        x1 = L * math.cos(math.radians(a1))
        y1 = L * math.sin(math.radians(a1))
        
        # absolute orientation of link 2 (CCW)
        theta2_abs = math.atan2(y - y1, x - x1)
        
        # convert to CW definition
        a2 = -math.degrees(theta2_abs)
        
        return a1, a2
        
    except Exception as e:
        st.error(f"IK calculation failed: {e}")
        return None, None

def plot_robot(alpha1, alpha2, target_pos=None):
    
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # calculate positions
    joint1, end_effector = forward_kinematics(alpha1, alpha2)
    
    # plot links
    ax.plot([0, joint1[0]], [0, joint1[1]], 'b-', linewidth=4, label='Link 1')
    ax.plot([joint1[0], end_effector[0]], [joint1[1], end_effector[1]], 'r-', linewidth=4, label='Link 2')
    
    # plot joints
    ax.plot(0, 0, 'ko', markersize=12, label='Base')
    ax.plot(joint1[0], joint1[1], 'ko', markersize=10, label='Joint 1')
    ax.plot(end_effector[0], end_effector[1], 'ro', markersize=10, label='End Effector')
    
    # plot target
    if target_pos:
        ax.plot(target_pos[0], target_pos[1], 'gx', markersize=15, markeredgewidth=3, label='Target')
    
    # grid
    ax.set_xlim(-11, 11)
    ax.set_ylim(-11, 11)
    ax.set_xticks(np.arange(-10, 11, 1))  # Steps of 1
    ax.set_yticks(np.arange(-10, 11, 1))  # Steps of 1
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend()
    
    # Title with angles
    title = f'2-Link Robot Arm - IK SOLVER\nα₁ = {alpha1:.2f}°, α₂ = {alpha2:.2f}°\nEnd Effector: ({end_effector[0]:.2f}, {end_effector[1]:.2f})'
    if target_pos:
        title += f'\nTarget: ({target_pos[0]:.2f}, {target_pos[1]:.2f})'
    ax.set_title(title)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    
    return fig

# Streamlit UI
st.title("2-Link Robot Arm - FK Engine and IK Solver")
st.write("Henry Munachimso - A Geometric Approach to Solving Forward and Inverse Kinematics")

# Sidebar for controls
st.sidebar.header("Control Panel")

# Mode selection
mode = st.sidebar.radio("Select Mode:", 
                        ["Forward Kinematics (Sliders)", 
                         "Inverse Kinematics (Solve for Target)"])

if mode == "Forward Kinematics (Sliders)":
    st.sidebar.subheader("Joint Angle Controls")
    
    # Sliders for joint angles
    alpha1 = st.sidebar.slider("α₁ - First joint angle (degrees)", 
                               -180.0, 180.0, 45.0, 1.0)
    alpha2 = st.sidebar.slider("α₂ - Second joint angle (degrees)", 
                               -180.0, 180.0, 30.0, 1.0)
    
    # end effector
    _, end_pos = forward_kinematics(alpha1, alpha2)
    st.sidebar.write(f"**End Effector Position:**")
    st.sidebar.write(f"X: {end_pos[0]:.2f}")
    st.sidebar.write(f"Y: {end_pos[1]:.2f}")
    
    # plot
    fig = plot_robot(alpha1, alpha2)
    st.pyplot(fig)

else:  # IK Mode
    st.sidebar.subheader("Target Position")
    
    # input buttons
    col1, col2 = st.sidebar.columns(2)
    with col1:
        target_x = st.number_input("Target X", -10.0, 10.0, 7.0, 0.1)
    with col2:
        target_y = st.number_input("Target Y", -10.0, 10.0, 3.0, 0.1)
    
    if st.sidebar.button("Solve Inverse Kinematics"):
        
        alpha1, alpha2 = inverse_kinematics(target_x, target_y)
        
        if alpha1 is not None and alpha2 is not None:
            st.sidebar.success("IK Solution Found")
            st.sidebar.write(f"**Required Joint Angles:**")
            st.sidebar.write(f"α₁ = {alpha1:.2f}°")
            st.sidebar.write(f"α₂ = {alpha2:.2f}°")
            
            # plot again
            fig = plot_robot(alpha1, alpha2, (target_x, target_y))
            st.pyplot(fig)
            
            # verify fk
            _, calculated_pos = forward_kinematics(alpha1, alpha2)
            error = np.sqrt((target_x - calculated_pos[0])**2 + 
                          (target_y - calculated_pos[1])**2)
            st.write(f"**Verification:** Position error = {error:.4f} units")
            
            # show setup workspace
            max_reach = 2 * L  # 10 units
            current_distance = np.sqrt(target_x**2 + target_y**2)
            st.write(f"**Setup Info:**")
            st.write(f"Max reach: {max_reach} units")
            st.write(f"Target distance: {current_distance:.2f} units")
            if current_distance > max_reach:
                st.warning("Target is outside maximum reach")
