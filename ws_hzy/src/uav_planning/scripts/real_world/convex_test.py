#! /usr/bin/env python

import cvxpy as cvx

# Define the variables
x1 = cvx.Variable()
x2 = cvx.Variable()

# Define the objective function
objective = cvx.Minimize(0.3 * x1 + 0.4 * x2)

# Define the constraints
constraints = [
    4.6 * x1 + 58.8 * x2 >= 300,
    52 * x1 + 33 * x2 >= 2000,
    152 * x1 + 133 * x2 >= 5000,
]

# Create the problem instance
problem = cvx.Problem(objective, constraints)

# Solve the problem
result = problem.solve()

# Print the optimal value and solution
print("Optimal value:", result)
print("Optimal solution:")
print("x1 =", x1.value)
print("x2 =", x2.value)
