import roboticstoolbox as rtb
from spatialmath import SE3                           # for representing the pose and orientation
from roboticstoolbox.backends.Swift import Swift      # instantiate 3D browser-based visualizer

robot = rtb.models.DH.Panda()                         # loading a franka model described using DH notation

T = robot.fkine(robot.qz)                             # Forward kinematics -- qz is 0-based joints angles
T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])  # change the fk format to SE3 pose (position and orientation)

sol = robot.ikine_LM(T)                               # Inverse Kinematics iterative numerical solution based on Levenberg-Marquadt minimization
q_pickup = sol.q

# animate a path (using the default matplotlib backend) from the upright qz config to the pickup location we ik solved for --  third argument is time-related
qt = rtb.jtraj(robot.qz, q_pickup, 15000)             # Compute a joint space trajectory between two configurations ([q,qd,qdd] = jtraj(q0, qf, T))

robot = rtb.models.URDF.Panda()                       # To load kinematic representation of the franka based on a rigid-body tree, not DH parameters

backend = Swift()           # instantiate our robot inside a browser-based 3d-simulation environment
backend.launch()            # activate it
backend.add(robot)          # add robot to the 3D scene

for qk in qt.q:             # for each joint configuration on trajectory
    robot.q = qk          # update the robot state
    backend.step()        # update visualization

qt = rtb.jtraj(q_pickup, robot.qz, 15000)

for qk in qt.q:             # for each joint configuration on trajectory
    robot.q = qk          # update the robot state
    backend.step()        # update visualization

