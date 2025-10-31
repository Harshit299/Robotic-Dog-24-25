import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

# Link lengths
a1 = 1  
a2 = 1  
a3 = 1 

def get_transformation_matrices(T1, T2, T3):
    # Convert angles to radians
    T1 = np.radians(T1)
    T2 = np.radians(T2) 
    T3 = np.radians(T3) 

    # Rotation matrices
    R0_1 = np.array([[-np.sin(T1), 0, np.cos(T1)],
                     [np.cos(T1),  0, np.sin(T1)],
                     [0,           1,          0]])

    R1_2 = np.array([[np.cos(T2), -np.sin(T2), 0],
                     [np.sin(T2),  np.cos(T2), 0],
                     [0,           0,          1]])

    R2_3 = np.array([[1, 0, 0],
                     [0, 0, 1],
                     [0, -1, 0]])

    # Displacement vectors
    D0_1 = np.array([[a1 * np.cos(T1)], [a1 * np.sin(T1)], [0]])
    D1_2 = np.array([[a2 * np.cos(T2)], [a2 * np.sin(T2)], [0]])
    D2_3 = np.array([[a3 * np.cos(T3)], [a3 * np.sin(T3)], [0]])

    # Homogeneous transformation matrices
    H0_1 = np.concatenate((R0_1, D0_1), axis=1)
    H0_1 = np.concatenate((H0_1, [[0, 0, 0, 1]]), axis=0)

    H1_2 = np.concatenate((R1_2, D1_2), axis=1)
    H1_2 = np.concatenate((H1_2, [[0, 0, 0, 1]]), axis=0)

    H2_3 = np.concatenate((R2_3, D2_3), axis=1)
    H2_3 = np.concatenate((H2_3, [[0, 0, 0, 1]]), axis=0)

    H0_2 = np.dot(H0_1, H1_2)
    H0_3 = np.dot(H0_2, H2_3)

    return H0_1, H0_2, H0_3

# Initialize the plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Default angles
T1, T2, T3 = 180, 180, 180

# Function to draw the manipulator
def draw_manipulator():
    global T1, T2, T3

    # Clear the previous plot
    ax.cla()

    # Get transformation matrices
    H0_1, H0_2, H0_3 = get_transformation_matrices(T1, T2, T3)

    # Extract joint positions
    origin = np.array([0, 0, 0])
    joint1 = H0_1[:3, 3]
    joint2 = H0_2[:3, 3]
    joint3 = H0_3[:3, 3]

    # Extract the end effector position
    end_effector = H0_3[:3, 3]

    # Plot the robotic arm
    ax.plot([origin[0], joint1[0]], [origin[1], joint1[1]], [origin[2], joint1[2]], 'k', linewidth=2)
    ax.plot([joint1[0], joint2[0]], [joint1[1], joint2[1]], [joint1[2], joint2[2]], 'k', linewidth=2)
    ax.plot([joint2[0], joint3[0]], [joint2[1], joint3[1]], [joint2[2], joint3[2]], 'k', linewidth=2)

    # Scatter the joint points
    ax.scatter(origin[0], origin[1], origin[2], color='k', s=50)
    ax.scatter(joint1[0], joint1[1], joint1[2], color='k', s=50)
    ax.scatter(joint2[0], joint2[1], joint2[2], color='k', s=50)
    ax.scatter(joint3[0], joint3[1], joint3[2], color='k', s=50)

    # Scatter the end effector
    ax.scatter(end_effector[0], end_effector[1], end_effector[2], color='r', s=50)

    # Central axes as lines
    ax.plot([-3, 3], [0, 0], [0, 0], color='black', linestyle='--', linewidth=1)  # X-axis
    ax.plot([0, 0], [-3, 3], [0, 0], color='black', linestyle='--', linewidth=1)  # Y-axis
    ax.plot([0, 0], [0, 0], [-3, 3], color='black', linestyle='--', linewidth=1)  # Z-axis

    # Set plot limits
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])

    # Set axis labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Robotic Dog Leg Simulation')

    # Display coordinates in the blank space (outside the 3D plot)
    # Place the coordinates in the top-right area of the figure
    ax.text2D(0.95, 0.95, f'End Effector: ({end_effector[0]:.2f}, {end_effector[1]:.2f}, {end_effector[2]:.2f})',
             transform=fig.transFigure, fontsize=12, color='black', ha='right', va='top')

# Update function for sliders
def update(val):
    global T1, T2, T3
    T1 = slider_T1.val
    T2 = slider_T2.val
    T3 = slider_T3.val
    draw_manipulator()

# Button functions
def set_view_xy(event):
    ax.view_init(elev=90, azim=-90)
    fig.canvas.draw_idle()

def set_view_xz(event):
    ax.view_init(elev=0, azim=-90)
    fig.canvas.draw_idle()

def set_view_yz(event):
    ax.view_init(elev=0, azim=0)
    fig.canvas.draw_idle()

# Sliders
ax_T1 = plt.axes([0.2, 0.02, 0.6, 0.02])  
ax_T2 = plt.axes([0.2, 0.06, 0.6, 0.02]) 
ax_T3 = plt.axes([0.2, 0.10, 0.6, 0.02])  

slider_T1 = Slider(ax_T1, 'T1',  90, 270, valinit=T1)
slider_T2 = Slider(ax_T2, 'T2', 0, 180, valinit=T2)
slider_T3 = Slider(ax_T3, 'T3', 0, 180, valinit=T3)

# Connect sliders to update function
slider_T1.on_changed(update)
slider_T2.on_changed(update)
slider_T3.on_changed(update)

# Buttons
ax_btn_xy = plt.axes([0.1, 0.15, 0.2, 0.05])
ax_btn_xz = plt.axes([0.4, 0.15, 0.2, 0.05])
ax_btn_yz = plt.axes([0.7, 0.15, 0.2, 0.05])

btn_xy = Button(ax_btn_xy, 'XY Plane')
btn_xz = Button(ax_btn_xz, 'XZ Plane')
btn_yz = Button(ax_btn_yz, 'YZ Plane')

# Connect buttons to their functions
btn_xy.on_clicked(set_view_xy)
btn_xz.on_clicked(set_view_xz)
btn_yz.on_clicked(set_view_yz)

# Initial plot
draw_manipulator()

# Show the plot
plt.show()