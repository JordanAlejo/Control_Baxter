import time
import pybullet as p
p.connect(p.GUI)
p.loadURDF("cube.urdf")
while 1:
  p.stepSimulation()
  time.sleep(1./240.)
