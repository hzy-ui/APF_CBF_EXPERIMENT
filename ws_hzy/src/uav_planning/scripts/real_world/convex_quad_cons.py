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

# Define additional constraints or range restrictions
A = matrix(np.array([[1.0, 0.0], [0.0, 1.0]]))  # Equality constraint matrix
b = matrix(np.array([10.0, 5.0]))  # Equality constraint vector

# Combine the inequality and equality constraints
G = matrix(np.vstack((G, A)))
h = matrix(np.vstack((h, b)))

# Solve the quadratic programming problem
sol = solvers.qp(P, q, G, h)

# Extract the optimal solution
optimal_solution = np.array(sol['x'])

# Print the optimal solution
print("Optimal solution:")
print(optimal_solution)
