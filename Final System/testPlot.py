# Robotics Toolbox used to determine camera coordinate frame given joint angles
import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
class RTB_Model():
    # Kinematic Model of the Robot in the Robotics Toolbox for Python (RTB)
    def __init__(self):
        # Model has units in meters
        self.ur5 = rtb.models.DH.UR5()
        homePose = np.array(np.radians([53,-112,144,-27.5,55,171.7]))
        self.ur5.q = homePose
        
    def setJointAngles(self,thetas):
        # thetas: 6 x 1 numpy array of joint angles (radians)
        self.ur5.q = thetas
        
    
    def plotRobot(self):
        # Displays robot in matplotlib
        ur5 = self.ur5
        env = ur5.plot(ur5.q) # PyPlot backend
        x,y,z = [],[],[]
        redPCD = o3d.io.read_point_cloud("redPCD.pcd")
        for point in redPCD.points:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        env.ax.scatter(x,y,z)
        T_C = self.getCameraFrame()
        T_C.plot(frame="C",length=0.1)
        env.hold()
        
    
    def getCameraFrame(self):
        # Robot joint angles need to be set pior 
        # Returns a SE3 SPatial Math Object (4 x 4 Homogenous Transform) corresponding to the robot's camera frame 
        T_N = self.ur5.fkine_all(self.ur5.q)[-1] # T_N - end-effector frame (before optoforce/gripper)
        d = 0.1 # distance between end-effector frame origin and center of camera frame along z-axis (m)
        P_C = np.array([0,0,d]) # Translation from frame T_N to origin of camera frame (m)
        # theta = np.radians(90) # Rotation about the Z-axis between the camera frame and end-effector frame
        T_C = T_N # * SE3.Tz(d) * SE3.Rz(theta) # Camera coordinate frame
        return T_C
    

r = RTB_Model()
r.plotRobot()