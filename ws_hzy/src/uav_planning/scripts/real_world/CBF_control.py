from CBF_toolbox import *
from CBF_toolbox_dyn import *
import numpy as np
from cvxopt import matrix, solvers

def CBF_controller(x, x_goal, R, rho, k, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    R: 
    x: position;
    x_goal: goal position 
    R: radius of goal area
    t: time period t = t-t_{0}
    t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    rho, k : RCBF parameters
    '''

    # Define the quadratic cost matrix
    P = matrix(np.array([[2.0, 0.0], [0.0, 2.0]]))

    # Define the linear cost vector
    q = matrix(np.array([0.0, 0.0]))

    # calculate the CBF 
    # print(t)
    h , gamma, b = CBF_generator(x, x_goal, R, t_star, t) 

    # calculate the partial derivatives
    p_b_x1, p_b_x2, b_t= partial_calculate(x, x_goal, R, t_star, t) 

    # Define the inequality constraint matrix
    G = matrix(np.array([[-p_b_x1, -p_b_x2]])) 

    # Define the inequality constraint vector
    h1 = matrix(np.array([ k*(b**(1)) -rho + b_t ]))

    # Solve the quadratic programming problem
    solvers.options['show_progress'] = False  # Disable solver progress output
    sol = solvers.qp(P, q, G, h1)

    # Extract the optimal solution
    optimal_solution = np.array(sol['x'])
    # Extract the objective value
    objective_value = sol['primal objective']

    
    # # Print the optimal solution
    # print("Optimal solution:")
    # print(optimal_solution)
    # # Print the objective value
    # print("Objective value:", objective_value)

    v1 = optimal_solution[0,0]
    v2 = optimal_solution[1,0]

    return v1, v2, h, gamma, b

# t = 0.1
# x= (0.1,0.1) 
# x_inital = (0, 0)
# x_goal = (5, 5)  
# R = 1.414 
# k = 0.1
# rho = 0.01 
# t_star =50 
# v1, v2, h, gamma, b= CBF_controller(x, x_inital, x_goal, R, rho, k, t_star, t)
# print(h) 
# print(gamma) 
# print(b) 






def CBF_controller_updates(x, x_initial, x_goal, R, rho, k, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    R: 
    x: position;
    x_goal: goal position 
    R: radius of goal area
    t: time period t = t-t_{0}
    t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    rho, k : RCBF parameters
    '''

    # Define the quadratic cost matrix
    P = matrix(np.array([[2.0, 0.0], [0.0, 2.0]]))

    # Define the linear cost vector
    q = matrix(np.array([0.0, 0.0]))

    # calculate the CBF 
    # print(t)

    h , gamma, b, p_b_x1, p_b_x2, b_t= CBF_generator_updates(x, x_initial, x_goal, R, t_star, t) 
    # print("l"+str(l))

    # calculate the partial derivatives
    # p_b_x1, p_b_x2, b_t= partial_calculate_updates(x, x_initial, x_goal, R, t_star, t) 
    # print([p_b_x1, p_b_x2, b_t])

    # Define the inequality constraint matrix
    G = matrix(np.array([[-p_b_x1, -p_b_x2]])) 

    # Define the inequality constraint vector
    h1 = matrix(np.array([ k*(b) -rho + b_t ]))

    

    # Solve the quadratic programming problem
    solvers.options['show_progress'] = False  # Disable solver progress output
    sol = solvers.qp(P, q, G, h1)

    # Extract the optimal solution
    optimal_solution = np.array(sol['x'])
    # Extract the objective value
    objective_value = sol['primal objective']

    
    # # Print the optimal solution
    # print("Optimal solution:")
    # print(optimal_solution)
    # # Print the objective value
    # print("Objective value:", objective_value)

    v1 = optimal_solution[0,0]
    v2 = optimal_solution[1,0]

    return v1, v2, h, gamma, b, b_t

