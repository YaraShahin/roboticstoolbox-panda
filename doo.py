import roboticstoolbox as rtb
from spatialmath import *
from spatialmath import SE3 
from roboticstoolbox.backends.Swift import Swift

panda = rtb.models.URDF.Panda()

T = SE3(0.5, 0.5, 0.5) * SE3.RPY([0.0, 0.0, 0.0], order='xyz') * SE3.Rx(-90, unit='deg')   #pose in xyz, rpy

sol = panda.ikine_LM(T)
q_pickup = sol.q
panda.q = q_pickup

backend = Swift()
backend.launch()            # activate it
backend.add(panda)          # add robot to the 3D scene
