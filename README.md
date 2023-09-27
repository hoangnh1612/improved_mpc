# improved_mpc
Run main1.py for standard MPC controller. Its setup in MPCController1.py.

Run main.py for improved MPC with external param($t_o$).

MPCController1.py is a application of standard MPC. 

in MPCController.py, Altitude controller, Position controller and Attitude Controller have been modified for adding time elements described in the following figure.

![Alt text](image/model.png?raw=true "Model")

Its formula described in this figure. Standard MPC contains only $J_x$ and $J_u$ while improved MPC have additional element $J_i$

![Alt text](image/formula.png?raw=true "Model")

TODO in this week: Create an environment for RL application.

