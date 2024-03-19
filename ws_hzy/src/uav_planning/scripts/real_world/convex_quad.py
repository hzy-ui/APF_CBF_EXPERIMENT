#! /usr/bin/env python

import numpy as np
from cvxopt import matrix, solvers

# Define the quadratic cost matrix
P = matrix(np.array([[2.0, 0.5], [0.5, 1.0]]))

# Define the linear cost vector
q = matrix(np.array([-1.0, -2.0]))

# Define the inequality constraint matrix
G = matrix(np.array([[-1.0, 0.0], [0.0, -1.0], [-1.0, -3.0], [2.0, 5.0]]))

# Define the inequality constraint vector
h = matrix(np.array([0.0, 0.0, -15.0, 100.0]))

# Solve the quadratic programming problem
sol = solvers.qp(P, q, G, h)

# Extract the optimal solution
optimal_solution = np.array(sol['x'])
# Extract the objective value
objective_value = sol['primal objective']

# Print the optimal solution
print("Optimal solution:")
print(optimal_solution[1,0])
# Print the objective value
print("Objective value:", objective_value)
