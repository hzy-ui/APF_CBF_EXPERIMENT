from potential_field import potential_field

obstacles = [(2, 2, 1)]  # Example obstacle positions
goal = (5, 5)  # Example goal position
#robot_radius = 0.5  # Example robot radius
attractive_gain = 0.1 # Example attractive gain
repulsive_gain = 10.0  # Example repulsive gain
repulsive_radius = 2.0  # Example repulsive radius

# Current robot position
x = 4
y = 4

# Calculate the total force using the potential_field function
total_force_x, total_force_y = potential_field(x, y, obstacles, goal, attractive_gain, repulsive_gain, repulsive_radius)
print(total_force_x)
print(total_force_y)