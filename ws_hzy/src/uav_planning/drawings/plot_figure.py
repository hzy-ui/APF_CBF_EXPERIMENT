import rosbag
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
# import matplotlib as mpl

# # Clear the font cache
# mpl.font_manager._rebuild()
# import matplotlib.font_manager as fm

# fonts = fm.findSystemFonts()
# for font in fonts:
#     if 'times new roman' in font.lower():
#         print(font)
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble'] =  r'\usepackage{amsmath}\usepackage{mathptmx}\usepackage{amssymb}\usepackage{bm}'

bag_path = '/home/xujun/catkin_ws/src/uav_planning/data/my_data_9_26_no_update.bag'
bag = rosbag.Bag(bag_path)
information = bag.read_messages(topics=['/barrier_information'])
# print(information)

t_values = []
h_values = []
gamma_values = []
b_values = []
u1_values = []
u2_values = []
x_values = []
y_values = []
b_t_values = []

for topic, msg, t in bag.read_messages(topics=['/barrier_information']):
    # Extract the data from the message
    t = msg.t
    h = msg.h
    gamma = msg.gamma
    b = msg.b
    u1 = msg.u1
    u2 = msg.u2
    x = msg.x
    y = msg.y
    b_t = msg.b_t
    #check if b < 0
    # if b < 0:
    #     print([x,y,b,t])
    # else:
    #     continue
    

    # Append the data to the lists
    t_values.append(t)
    h_values.append(h)
    gamma_values.append(gamma)
    b_values.append(b)
    u1_values.append(u1)
    u2_values.append(u2)
    x_values.append(x)
    y_values.append(y)
    b_t_values.append(b_t)



plt.figure(figsize=(3, 3))  # Replace width and height with the desired dimensions of the graph in inches
plt.figure(1)
# Enable LaTeX rendering
# plt.rcParams['text.usetex'] = True

# plt.xlabel('X', fontname='Times New Roman', fontsize=12)
# plt.ylabel('Y', fontname='Times New Roman', fontsize=12)
# plt.title('The trajectory of the UAV', fontname='Times New Roman', fontsize=16)
# Add grid
plt.grid(True)
# Define circle parameters
# visit areas (blue)
circle_v1 = Circle((-6, -6), radius=1, facecolor='blue', edgecolor='black', zorder=1)
circle_v2 = Circle((6, -6), radius=1, facecolor='blue', edgecolor='black', zorder=1)
circle_v3 = Circle((-6, 6), radius=1, facecolor='blue', edgecolor='black', zorder=1)
circle_v4 = Circle((0, 0), radius=1, facecolor='blue', edgecolor='black', zorder=1)
# obstacle areas (red)
circle_ob1 = Circle((-4, -4), radius= 0.6, facecolor='red', edgecolor='black', zorder=1)
circle_ob2 = Circle((1, -6), radius= 1, facecolor='red', edgecolor='black', zorder=1)
circle_ob3 = Circle((-2, 2), radius= 1.5, facecolor='red', edgecolor='black', zorder=1)
# Add circles to the plot
plt.gca().add_patch(circle_v1)
plt.gca().add_patch(circle_v2)
plt.gca().add_patch(circle_v3)
plt.gca().add_patch(circle_v4)
plt.gca().add_patch(circle_ob1)
plt.gca().add_patch(circle_ob2)
plt.gca().add_patch(circle_ob3)
# plot the trajectory
# plt.scatter(x_values, y_values, s=5, zorder=2)
plt.plot(x_values, y_values, label='$traj(\mathbf{{\pmb{x}}})$', linewidth=2.5, zorder=2)
# Set the range of the coordinate axes
plt.xlim(-7, 7)  # Replace x_min and x_max with the desired minimum and maximum values for the x-axis
plt.ylim(-7, 7)  # Replace y_min and y_max with the desired minimum and maximum values for the y-axis
plt.xticks(fontsize=12)  # Replace 8 with the desired font size
plt.yticks(fontsize=12)  # Replace 8 with the desired font size
# Adjust labels and spacing
plt.legend(loc='best', fontsize=12)  # Place the legend at the best position
plt.subplots_adjust(left=0.15, right=0.93, bottom=0.1, top=0.95)
# plt.tight_layout()  # Automatically adjust the spacing




# Set the size of the graph
plt.figure(figsize=(7, 2))  # Replace width and height with the desired dimensions of the graph in inches
plt.figure(2) # Create a figure
plt.plot(t_values, h_values, label='$h(\mathbf{{\pmb{x}}})$', linewidth=2.5, color='blue') # Plot y1
plt.plot(t_values, gamma_values, label='$\gamma(t)$', linewidth=2.5, color='red') # Plot y2
# Set labels and title
plt.xlabel('Time (s)', fontname='Times New Roman', fontsize=12, usetex= False)
# plt.ylabel('Y', fontname='Times New Roman', fontsize=10)
# plt.title('$h(\mathbf{x})$ and $\gamma(t)$', fontname='Times New Roman', fontsize=16)
# Add grid
plt.grid(True)
plt.xlim(20, max(t_values))  # Replace x_min and x_max with the desired minimum and maximum values for the x-axis
plt.xticks(fontsize=12)  # Replace 8 with the desired font size
plt.yticks(fontsize=12)  # Replace 8 with the desired font size
# Adjust labels and spacing
plt.legend(loc='best', fontsize=12)  # Place the legend at the best position
plt.tight_layout()  # Automatically adjust the spacing


# Set the size of the graph
plt.figure(figsize=(7, 2))  # Replace width and height with the desired dimensions of the graph in inches
plt.figure(3) # Create a figure
plt.plot(t_values, b_values, label='$\mathfrak{b}(\mathbf{{\pmb{x}}})$', linewidth=2.5, color='blue') # Plot y1
# plt.plot(t_values, gamma_values, label='y2') # Plot y2
# Set labels and title
plt.xlabel('Time (s)', fontname='Times New Roman', fontsize=12, usetex= False)
# plt.ylabel('$b(\mathbf{{\pmb{x}}})$', fontname='Times New Roman', fontsize=10)
# plt.title('The value of barrier function $b(\mathbf{x})$', fontname='Times New Roman', fontsize=16)
# Add grid
plt.grid(True)
# Adjust labels and spacing
plt.legend(loc='best')  # Place the legend at the best position
plt.tight_layout()  # Automatically adjust the spacing
# Add legend
plt.legend()
plt.xlim(20, max(t_values))  # Replace x_min and x_max with the desired minimum and maximum values for the x-axis
plt.xticks(fontsize=12)  # Replace 8 with the desired font size
plt.yticks(fontsize=12)  # Replace 8 with the desired font size
# Adjust labels and spacing
plt.legend(loc='best', fontsize=12)  # Place the legend at the best position
plt.tight_layout()  # Automatically adjust the spacing



# Set the size of the graph
plt.figure(figsize=(7, 2))  # Replace width and height with the desired dimensions of the graph in inches
plt.figure(4) # Create a figure
plt.plot(t_values, u1_values, label='$u_{1}$', linewidth=2.5, color='blue') # Plot y1
plt.plot(t_values, u2_values, label='$u_{2}$', linewidth=2.5, color='red') # Plot y2
# Set labels and title
plt.xlabel('Time (s)', fontname='Times New Roman', fontsize=12, usetex= False)
# plt.ylabel('$b(\mathbf{x})$', fontname='Times New Roman', fontsize=10)
# plt.title('The value of barrier function $b(\mathbf{x})$', fontname='Times New Roman', fontsize=16)
# Add grid
plt.grid(True)
# Adjust labels and spacing
# plt.legend(loc='best')  # Place the legend at the best position
# plt.tight_layout()  # Automatically adjust the spacing
# Add legend
plt.legend()
plt.xlim(20, max(t_values))  # Replace x_min and x_max with the desired minimum and maximum values for the x-axis
plt.xticks(fontsize=12)  # Replace 8 with the desired font size
plt.yticks(fontsize=12)  # Replace 8 with the desired font size
# Adjust labels and spacing
plt.legend(loc='lower right', fontsize=12)  # Place the legend at the best position
plt.tight_layout()  # Automatically adjust the spacing
# Adjust padding
# plt.subplots_adjust(left=0.05, right=0.95, bottom=0.3, top=0.95)


# plt.figure(5) # Create a figure
# plt.plot(t_values, b_t_values, label='$u_{1}$', linewidth=2) # Plot y1

plt.show()
bag.close()
