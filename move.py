import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import SE3, SO3                #for representing the pose and orientation
from roboticstoolbox.backends.Swift import Swift  # 3D browser-based visualizer
from roboticstoolbox.tools.urdf import Sphere, Box
import numpy as np
import qpsolvers as qp
from time import sleep
import math

targetPose = SE3(0.5, 0.5, 0.5) * SO3.Rz(90, unit='deg')
"""
# No error but the box doesn't show up at the simulator. for more info check appdata/roaming/python/python39/site-packages/roboticstoolbox/tools/urdf/urdf.py
#target = Sphere(radius=2.0)
target = Sphere(
    radius=0.02,
    base=sm.SE3(0.6, -0.2, 0.0)
)
backend = Swift()
backend.launch()      
backend.add(target)   
"""

robot = rtb.models.DH.Panda()                  #loading a franka model described using DH notation
#print(robot)

T = robot.fkine(robot.qz)                      # Forward kinematics -- qz is default pose
#print(T)

T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])   #change the fk format to SE3 pose (position and orientation)
TT = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, 1])   #change the fk format to SE3 pose (position and orientation)
#print(T)

sol = robot.ikine_LM(T)                        # Inverse Kinematics iterative numerical solution based on Levenberg-Marquadt minimization
#print(sol)

q_pickup = sol.q
#print(robot.fkine(q_pickup))                  # read fk again to see that desired end-effector pose was achieved

#animate a path (using the default matplotlib backend) from the upright qz config to the pickup location we ik solved for
qt = rtb.jtraj(robot.qz, q_pickup, 15000)      #Compute a joint space trajectory between two configurations ([q,qd,qdd] = jtraj(q0, qf, T))
#robot.plot(qt.q, movie='panda1.gif')

#To load kinematic representation of the franka based on a rigid-body tree, not DH parameters
robot = rtb.models.URDF.Panda()                #The symbol @ indicates the link as an end-effector, a leaf node in the rigid-body tree.
#print(robot)

#instantiate our robot inside a browser-based 3d-simulation environment.
backend = Swift()
backend.launch()            # activate it

"""
backend.add(robot)          # add robot to the 3D scene
for qk in qt.q:             # for each joint configuration on trajectory
    robot.q = qk            # update the robot state
    backend.step()          # update visualization

qt = rtb.jtraj(q_pickup, robot.qz, 15000)  # 15000 is the time

for qk in qt.q:             # for each joint configuration on trajectory
    robot.q = qk            # update the robot state
    backend.step()          # update visualization

backend.remove(robot)
"""
# To plot a 3d franka in target pose
#targetPose = SE3(0.5, 0.5, 0.5) * SE3.Rz(90, unit='deg')   # pose in xyz, rpy
#targetPose = SE3(0.5, 0.5, 0.5) * SE3.RPY([0.0, 0.0, math.pi/2], order='xyz') * SE3.Ry(-90, unit='deg')   #pose in xyz, rpy
sol = robot.ikine_LM(T)        #inverse kinematics
robot.q = sol.q                         #update robot state

backend.add(robot)                      # add robot to the 3D scene
#backend.remove(robot) 
pose = SE3(0.5, 0.5, 0.5)* SE3.Rz(-90, unit='deg')   # pose in xyz, rpy
sol = robot.ikine_LM(TT)  # inverse kinematics
robot.q = sol.q  # update robot state

backend.add(robot)

sleep(15)
#backend.remove(robot)

"""
#TypeError: __init__() got an unexpected keyword argument 'base'
s0 = Sphere(
    radius=0.05,
    base=sm.SE3(0.52, 0.4, 0.3)
)
obstacle = Box([1, 1, 1], SE3(1, 0, 0))
backend.add(obstacle)
backend.add(s0)   
"""
