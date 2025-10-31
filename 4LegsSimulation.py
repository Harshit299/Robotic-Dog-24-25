import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

# Link lengths
a1 = 5 
a2 = 6  
a3 = 5.5 

# Fixed offsets
x_offset = 1  
z_offset = 8.5  

# Global variables for joint angles of each leg
T1_r, T2_r, T3_r = 0, 180, 180  # Front right leg
T1_l, T2_l, T3_l = 0, 180, 180  # Front left leg
T1_r_b, T2_r_b, T3_r_b = 0, 180, 180  # Back right leg
T1_l_b, T2_l_b, T3_l_b = 0, 180, 180  # Back left leg

# Function to calculate transformation matrices for each leg
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

# Function to get joint positions for a leg with fixed offsets
def get_leg_positions(T1, T2, T3, x_offset=0, z_offset=0):
    H0_1, H0_2, H0_3 = get_transformation_matrices(T1, T2, T3)

    # Apply the offset to all joints
    origin = np.array([x_offset, 0, z_offset])  # Fixed offsets for x and z
    joint1 = H0_1[:3, 3] + origin
    joint2 = H0_2[:3, 3] + origin
    joint3 = H0_3[:3, 3] + origin

    return origin, joint1, joint2, joint3

# Initialize the plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Function to draw the manipulator with the chassis (connecting the leg bases)
def draw_manipulator():
    global T1_r, T2_r, T3_r, T1_l, T2_l, T3_l, T1_r_b, T2_r_b, T3_r_b, T1_l_b, T2_l_b, T3_l_b

    # Clear the previous plot
    ax.cla()

    # Get positions for both front legs
    origin_r, joint1_r, joint2_r, joint3_r = get_leg_positions(T1_r, T2_r, T3_r, x_offset=x_offset, z_offset=z_offset)
    origin_l, joint1_l, joint2_l, joint3_l = get_leg_positions(T1_l, T2_l, T3_l, x_offset=-x_offset, z_offset=z_offset)

    # Get positions for back legs
    origin_r_b, joint1_r_b, joint2_r_b, joint3_r_b = get_leg_positions(T1_r_b, T2_r_b, T3_r_b, x_offset=x_offset, z_offset=-z_offset)
    origin_l_b, joint1_l_b, joint2_l_b, joint3_l_b = get_leg_positions(T1_l_b, T2_l_b, T3_l_b, x_offset=-x_offset, z_offset=-z_offset)

    # Plot the chassis (connect all leg bases)
    ax.plot([origin_r[0], origin_l[0]], [origin_r[1], origin_l[1]], [origin_r[2], origin_l[2]], 'purple', linewidth=2)
    ax.plot([origin_r[0], origin_r_b[0]], [origin_r[1], origin_r_b[1]], [origin_r[2], origin_r_b[2]], 'purple', linewidth=2)
    ax.plot([origin_l[0], origin_l_b[0]], [origin_l[1], origin_l_b[1]], [origin_l[2], origin_l_b[2]], 'purple', linewidth=2)
    ax.plot([origin_r_b[0], origin_l_b[0]], [origin_r_b[1], origin_l_b[1]], [origin_r_b[2], origin_l_b[2]], 'purple', linewidth=2)

    # Plot right front leg+
    ax.plot([origin_r[0], joint1_r[0]], [origin_r[1], joint1_r[1]], [origin_r[2], joint1_r[2]], 'k', linewidth=2)
    ax.plot([joint1_r[0], joint2_r[0]], [joint1_r[1], joint2_r[1]], [joint1_r[2], joint2_r[2]], 'k', linewidth=2)
    ax.plot([joint2_r[0], joint3_r[0]], [joint2_r[1], joint3_r[1]], [joint2_r[2], joint3_r[2]], 'k', linewidth=2)

    # Plot left front leg
    ax.plot([origin_l[0], joint1_l[0]], [origin_l[1], joint1_l[1]], [origin_l[2], joint1_l[2]], 'k', linewidth=2)
    ax.plot([joint1_l[0], joint2_l[0]], [joint1_l[1], joint2_l[1]], [joint1_l[2], joint2_l[2]], 'k', linewidth=2)
    ax.plot([joint2_l[0], joint3_l[0]], [joint2_l[1], joint3_l[1]], [joint2_l[2], joint3_l[2]], 'k', linewidth=2)

    # Plot right back leg
    ax.plot([origin_r_b[0], joint1_r_b[0]], [origin_r_b[1], joint1_r_b[1]], [origin_r_b[2], joint1_r_b[2]], 'b', linewidth=2)
    ax.plot([joint1_r_b[0], joint2_r_b[0]], [joint1_r_b[1], joint2_r_b[1]], [joint1_r_b[2], joint2_r_b[2]], 'b', linewidth=2)
    ax.plot([joint2_r_b[0], joint3_r_b[0]], [joint2_r_b[1], joint3_r_b[1]], [joint2_r_b[2], joint3_r_b[2]], 'b', linewidth=2)

    # Plot left back leg
    ax.plot([origin_l_b[0], joint1_l_b[0]], [origin_l_b[1], joint1_l_b[1]], [origin_l_b[2], joint1_l_b[2]], 'b', linewidth=2)
    ax.plot([joint1_l_b[0], joint2_l_b[0]], [joint1_l_b[1], joint2_l_b[1]], [joint1_l_b[2], joint2_l_b[2]], 'b', linewidth=2)
    ax.plot([joint2_l_b[0], joint3_l_b[0]], [joint2_l_b[1], joint3_l_b[1]], [joint2_l_b[2], joint3_l_b[2]], 'b', linewidth=2)

    # Scatter points for all joints
    ax.scatter(origin_r[0], origin_r[1], origin_r[2], color='k', s=50)
    ax.scatter(joint1_r[0], joint1_r[1], joint1_r[2], color='k', s=50)
    ax.scatter(joint2_r[0], joint2_r[1], joint2_r[2], color='k', s=50)
    ax.scatter(joint3_r[0], joint3_r[1], joint3_r[2], color='r', s=50)

    ax.scatter(origin_l[0], origin_l[1], origin_l[2], color='k', s=50)
    ax.scatter(joint1_l[0], joint1_l[1], joint1_l[2], color='k', s=50)
    ax.scatter(joint2_l[0], joint2_l[1], joint2_l[2], color='k', s=50)
    ax.scatter(joint3_l[0], joint3_l[1], joint3_l[2], color='r', s=50)

    ax.scatter(origin_r_b[0], origin_r_b[1], origin_r_b[2], color='k', s=50)
    ax.scatter(joint1_r_b[0], joint1_r_b[1], joint1_r_b[2], color='k', s=50)
    ax.scatter(joint2_r_b[0], joint2_r_b[1], joint2_r_b[2], color='k', s=50)
    ax.scatter(joint3_r_b[0], joint3_r_b[1], joint3_r_b[2], color='r', s=50)

    ax.scatter(origin_l_b[0], origin_l_b[1], origin_l_b[2], color='k', s=50)
    ax.scatter(joint1_l_b[0], joint1_l_b[1], joint1_l_b[2], color='k', s=50)
    ax.scatter(joint2_l_b[0], joint2_l_b[1], joint2_l_b[2], color='k', s=50)
    ax.scatter(joint3_l_b[0], joint3_l_b[1], joint3_l_b[2], color='r', s=50)

    # Central axes as lines
    ax.plot([-12, 12], [0, 0], [0, 0], color='black', linestyle='--', linewidth=1)  # X-axis
    ax.plot([0, 0], [-12, 12], [0, 0], color='black', linestyle='--', linewidth=1)  # Y-axis
    ax.plot([0, 0], [0, 0], [-12, 12], color='black', linestyle='--', linewidth=1)  # Z-axis

    # Set plot limits
    ax.set_xlim([-12, 12])
    ax.set_ylim([-12, 12])
    ax.set_zlim([-12, 12])

    # Set axis labels and title
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Robotic Dog Simulation (All Legs)')

    # Display coordinates of end effectors
    ax.text2D(0.95, 0.95, 
              f'Right End Effector Front: ({joint3_r[0]:.2f}, {joint3_r[1]:.2f}, {joint3_r[2]:.2f})\n'
              f'Left End Effector Front: ({joint3_l[0]:.2f}, {joint3_l[1]:.2f}, {joint3_l[2]:.2f})\n'
              f'Right End Effector Back: ({joint3_r_b[0]:.2f}, {joint3_r_b[1]:.2f}, {joint3_r_b[2]:.2f})\n'
              f'Left End Effector Back: ({joint3_l_b[0]:.2f}, {joint3_l_b[1]:.2f}, {joint3_l_b[2]:.2f})', 
              transform=ax.transAxes, fontsize=10, verticalalignment='top')

    plt.draw()


# View change functions
def set_view_xy(event):
    ax.view_init(elev=90, azim=-90)
    plt.draw()

def set_view_yz(event):
    ax.view_init(elev=0, azim=90)
    plt.draw()

def set_view_xz(event):
    ax.view_init(elev=90, azim=0)
    plt.draw()

# Create buttons for plane views
ax_xy_button = plt.axes([0.01, 0.9, 0.1, 0.05])
button_xy = Button(ax_xy_button, 'XY Plane')
button_xy.on_clicked(set_view_xy)

ax_yz_button = plt.axes([0.01, 0.85, 0.1, 0.05])
button_yz = Button(ax_yz_button, 'YZ Plane')
button_yz.on_clicked(set_view_yz)

ax_xz_button = plt.axes([0.01, 0.8, 0.1, 0.05])
button_xz = Button(ax_xz_button, 'XZ Plane')
button_xz.on_clicked(set_view_xz)

# Sliders for each leg
ax_T1_r = plt.axes([0.75, 0.21, 0.2, 0.02])
ax_T2_r = plt.axes([0.75, 0.18, 0.2, 0.02])
ax_T3_r = plt.axes([0.75, 0.15, 0.2, 0.02])

slider_T1_r = Slider(ax_T1_r, 'Front Right T1', -90, 90, valinit=T1_r, color='black')
slider_T2_r = Slider(ax_T2_r, 'Front Right T2', 0, 180, valinit=T2_r, color='black')
slider_T3_r = Slider(ax_T3_r, 'Front Right T3', 0, 180, valinit=T3_r, color='black')

ax_T1_l = plt.axes([0.1, 0.21, 0.2, 0.02])
ax_T2_l = plt.axes([0.1, 0.18, 0.2, 0.02])
ax_T3_l = plt.axes([0.1, 0.15, 0.2, 0.02])

slider_T1_l = Slider(ax_T1_l, 'Front Left T1', 90, 270, valinit=T1_l, color='black')
slider_T2_l = Slider(ax_T2_l, 'Front Left T2', 0, 180, valinit=T2_l, color='black')
slider_T3_l = Slider(ax_T3_l, 'Front Left T3', 0, 180, valinit=T3_l, color='black')

ax_T1_r_b = plt.axes([0.75, 0.11, 0.2, 0.02])
ax_T2_r_b = plt.axes([0.75, 0.08, 0.2, 0.02])
ax_T3_r_b = plt.axes([0.75, 0.05, 0.2, 0.02])

slider_T1_r_b = Slider(ax_T1_r_b, 'Back Right T1', -90, 90, valinit=T1_r_b, color='blue')
slider_T2_r_b = Slider(ax_T2_r_b, 'Back Right T2', 0, 180, valinit=T2_r_b, color='blue')
slider_T3_r_b = Slider(ax_T3_r_b, 'Back Right T3', 0, 180, valinit=T3_r_b, color='blue')

ax_T1_l_b = plt.axes([0.1, 0.11, 0.2, 0.02])
ax_T2_l_b = plt.axes([0.1, 0.08, 0.2, 0.02])
ax_T3_l_b = plt.axes([0.1, 0.05, 0.2, 0.02])

slider_T1_l_b = Slider(ax_T1_l_b, 'Back Left T1', 90, 270, valinit=T1_l_b, color='blue')
slider_T2_l_b = Slider(ax_T2_l_b, 'Back Left T2', 0, 180, valinit=T2_l_b, color='blue')
slider_T3_l_b = Slider(ax_T3_l_b, 'Back Left T3', 0, 180, valinit=T3_l_b, color='blue')

# Attach the update function to sliders
def update(val):
    global T1_r, T2_r, T3_r, T1_l, T2_l, T3_l, T1_r_b, T2_r_b, T3_r_b, T1_l_b, T2_l_b, T3_l_b

    T1_r = slider_T1_r.val
    T2_r = slider_T2_r.val
    T3_r = slider_T3_r.val
    
    T1_l = slider_T1_l.val
    T2_l = slider_T2_l.val
    T3_l = slider_T3_l.val
    
    T1_r_b = slider_T1_r_b.val
    T2_r_b = slider_T2_r_b.val
    T3_r_b = slider_T3_r_b.val
    
    T1_l_b = slider_T1_l_b.val
    T2_l_b = slider_T2_l_b.val
    T3_l_b = slider_T3_l_b.val
    
    draw_manipulator()  # Redraw the manipulator after angle changes
    plt.draw()

slider_T1_r.on_changed(update)
slider_T2_r.on_changed(update)
slider_T3_r.on_changed(update)
slider_T1_l.on_changed(update)
slider_T2_l.on_changed(update)
slider_T3_l.on_changed(update)
slider_T1_r_b.on_changed(update)
slider_T2_r_b.on_changed(update)
slider_T3_r_b.on_changed(update)
slider_T1_l_b.on_changed(update)
slider_T2_l_b.on_changed(update)
slider_T3_l_b.on_changed(update)

# Initialize plot
draw_manipulator()

# Show plot
plt.show()