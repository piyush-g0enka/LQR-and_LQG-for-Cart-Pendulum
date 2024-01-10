# LQR-and_LQG-for-Cart-Pendulum

## First Component (100 points):

Consider a crane that moves along a one-dimensional track. It behaves as a frictionless cart with mass M actuated by an external force F that constitutes the input of the system. There are two loads suspended from cables attached to the crane. The loads have mass m1 and m2, and the lengths of the cables are l1 and l2, respectively. The following figure depicts the crane and associated variables used throughout this project.

### A) (25 points)

Obtain the equations of motion for the system and the corresponding nonlinear state-space representation.

### B) (25 points)

Obtain the linearized system around the equilibrium point specified by x = 0 and θ1 = θ2 = 0. Write the state-space representation of the linearized system.

### C) (25 points)

Obtain conditions on M, m1, m2, l1, l2 for which the linearized system is controllable.

### D) (25 points)

Choose M = 1000Kg, m1 = m2 = 100Kg, l1 = 20m and l2 = 10m. Check that the system is controllable and obtain an LQR controller. Simulate the resulting response to initial conditions when the controller is applied to the linearized system and also to the original nonlinear system. Adjust the parameters of the LQR cost until you obtain a suitable response. Use Lyapunov’s indirect method to certify stability (locally or globally) of the closed-loop system.

## Second Component (100 points):

Consider the parameters selected in C) above.

### E)

Suppose that you can select the following output vectors: x(t), (θ1(t), θ2(t)), (x(t), θ2(t)) or (x(t), θ1(t), θ2(t)). Determine for which output vectors the linearized system is observable.

### F)

Obtain your "best" Luenberger observer for each one of the output vectors for which the system is observable and simulate its response to initial conditions and unit step input. The simulation should be done for the observer applied to both the linearized system and the original nonlinear system.

### G)

Design an output feedback controller for your choice of the "smallest" output vector. Use the LQG method and apply the resulting output feedback controller to the original nonlinear system. Obtain your best design and illustrate its performance in simulation. How would you reconfigure your controller to asymptotically track a constant reference on x? Will your design reject constant force disturbances applied to the cart?

___
## Our project has 4 Matlab files.

- **Part_D_LQR.mlx:** Run this file in Matlab. In case you encounter an error (Matlab bug), please run the sections one by one. This file contains code for Part D.

- **Part_G_LQG.mlx:** Run this file in Matlab. In case you get an error (Matlab bug), then please run the sections one by one. This file contains code for Part G.

- **Part_EF_linear.m:** Run this file in Matlab. This file contains code for E and F linear condition.

- **Part_EF_nonlinear.m:** Run this file in Matlab. This file contains code for E and F nonlinear condition.
