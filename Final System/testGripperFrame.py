import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
gripper_frame = sm.SE3()
R = np.array([[0,-1,0],[1,0,0],[0,0,1]])
t = np.array([9,0,-59.3])
# t = np.array([0,0,0])
extrinsic_matrix = sm.SE3.Rt(R,t)
print(extrinsic_matrix)
gripper_frame.plot(frame="G")
extrinsic_matrix.plot(frame="C")

plt.show()