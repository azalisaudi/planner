# Planner: Robot Path Planning

Path Planning simulator. The map is in PNG format. 

The planner computes the harmonic functions obtained from the solutions of Laplace's equation.

The Laplace's equation is solved using the finite difference approximation via iterative method such as Gauss-Seidel and SOR.

The obtained harmonic functions is then used to guide the planner to find path from any start point to the goal point by following the gradient descent search procedure.
