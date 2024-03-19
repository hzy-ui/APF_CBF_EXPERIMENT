import math
import random

def potential_field(x, y, obstacles, goal, attractive_gain, repulsive_gain, repulsive_radius):
    # Calculate attractive force
    dx = goal[0] - x
    dy = goal[1] - y
    distance_to_goal = math.sqrt(dx**2 + dy**2)
    attractive_force_x = attractive_gain * dx 
    attractive_force_y = attractive_gain * dy 
    # print(attractive_force_x)
    # Calculate repulsive forces
    repulsive_force_x = 0
    repulsive_force_y = 0
    for obstacle in obstacles:
        ox, oy, oR = obstacle
        dx = x - ox
        dy = y - oy
        distance_to_obstacle = math.sqrt(dx**2 + dy**2)
        if distance_to_obstacle < repulsive_radius:
            repulsive_force_x += repulsive_gain * (1 / distance_to_obstacle - 1 / repulsive_radius) * (dx / distance_to_obstacle**3)
            repulsive_force_y += repulsive_gain * (1 / distance_to_obstacle - 1 / repulsive_radius) * (dy / distance_to_obstacle**3)
            # print(repulsive_gain)
            # print(1 / distance_to_obstacle - 1 / repulsive_radius)
            # print(1 / distance_to_obstacle**3)

    # Calculate total force
    total_force_x = attractive_force_x + repulsive_force_x
    total_force_y = attractive_force_y + repulsive_force_y

    return total_force_x, total_force_y



def potential_field_dynamic(x, y, obstacles, goal, attractive_gain, repulsive_gain):
    # Calculate attractive force
    dx = goal[0] - x
    dy = goal[1] - y
    distance_to_goal = math.sqrt(dx**2 + dy**2)
    attractive_force_x = attractive_gain * dx 
    attractive_force_y = attractive_gain * dy 
    # print(attractive_force_x)
    # Calculate repulsive forces
    repulsive_force_x = 0
    repulsive_force_y = 0
    for obstacle in obstacles:
        ox, oy, oR = obstacle
        dx = x - ox
        dy = y - oy
        if oR <=1:
            repulsive_radius = oR + 1.2
        else:
            repulsive_radius = oR + 2.0
        distance_to_obstacle = math.sqrt(dx**2 + dy**2)
        if distance_to_obstacle < repulsive_radius:
            repulsive_force_x += repulsive_gain * (1 / distance_to_obstacle - 1 / repulsive_radius) * (dx / distance_to_obstacle**3)
            repulsive_force_y += repulsive_gain * (1 / distance_to_obstacle - 1 / repulsive_radius) * (dy / distance_to_obstacle**3)
            # print(repulsive_gain)
            # print(1 / distance_to_obstacle - 1 / repulsive_radius)
            # print(1 / distance_to_obstacle**3)

    # Calculate total force
    total_force_x = attractive_force_x + repulsive_force_x
    total_force_y = attractive_force_y + repulsive_force_y
    # if math.sqrt(total_force_x**2 + total_force_y**2) <1:
    #     total_force_x = total_force_x +  random.uniform(0, 0.)
    #     total_force_y = total_force_y +  random.uniform(0, 1)



    return total_force_x, total_force_y

# obstacles = [(2, 2, 1)]  # Example obstacle positions
# goal = (5, 5)  # Example goal position
# #robot_radius = 0.5  # Example robot radius
# attractive_gain = 0.1 # Example attractive gain
# repulsive_gain = 10.0  # Example repulsive gain
# repulsive_radius = 2.0  # Example repulsive radius

# # Current robot position
# x = 4
# y = 4

# # Calculate the total force using the potential_field function
# total_force_x, total_force_y = potential_field(x, y, obstacles, goal, attractive_gain, repulsive_gain, repulsive_radius)
# print(total_force_x)
# print(total_force_y)
