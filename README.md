# Planner: Robot Path Planning

Path Planning simulator. The map is in PNG format. It is implemented in Java. 

Use mouse left and right click to specify the start and goal points.

The planner computes the harmonic functions obtained from the solutions of Laplace's equation.

The Laplace's equation is solved using the finite difference approximation via iterative method such as Gauss-Seidel and SOR.

The obtained harmonic functions is then used to guide the planner to find path from any start point to the goal point by following the gradient descent search procedure.

If you enjoy this code, please credit it by citing the following papers in your citation.

References:

1. Saudi, A., Sulaiman, J. Red-Black strategy for mobile robot path planning, (2010). Proceedings of the International MultiConference of Engineers and Computer Scientists 2010, IMECS 2010, pp. 2215-2220. http://www.iaeng.org/publication/IMECS2010/IMECS2010_pp2215-2220.pdf
2. Dahalan, A. A., Saudi, A. Rotated TOR-5P Laplacian Iteration Path Navigation for Obstacle Avoidance in Stationary Indoor Simulation, (2021). Advances in Intelligent Systems and Computing, 1350 AISC, pp. 285-295. https://link.springer.com/chapter/10.1007%2F978-3-030-70917-4_27
3. Saudi, A., Sulaiman, J. Indoor path planning for mobile robot using LBBC-EG, (2013). International Journal of Imaging and Robotics, 11 (3), pp. 37-45. https://www.researchgate.net/publication/281875280_Indoor_path_planning_for_mobile_robot_using_LBBC-EG
4. Saudi, A., Sulaiman, J. Robot path planning using Laplacian Behaviour-Based Control (LBBC) via Half-Sweep SOR, (2013). 2013 The International Conference on Technological Advances in Electrical, Electronics and Computer Engineering, TAEECE 2013, art. no. 6557312, pp. 424-429. https://ieeexplore.ieee.org/document/6557312

My other works can be accessed at:
1. WoS ResearchID: https://publons.com/researcher/Q-1006-2018/
2. Personal site: https://sites.google.com/ums.edu.my/azali/home
3. ResearchGate: https://www.researchgate.net/profile/Azali-Saudi
