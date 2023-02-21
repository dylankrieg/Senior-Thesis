import runpy
import roboticstoolbox as rtb
from roboticstoolbox import ET # ET - elementary transform
from spatialmath import SE3
import swift
import matplotlib.pyplot as plt
import numpy as np
from roboticstoolbox.backends.PyPlot import PyPlot2
import open3d as o3d
import numpy.linalg as LA
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})


class rtbModel():
    def __init__(self):
        # Model has units in meters
        self.ur5 = rtb.models.DH.UR5()
        homePose = np.array(np.radians([53.12,-112.14,144.11,-27.45,55.00,171.68]))
        self.ur5.q = homePose
        self.pcd = o3d.io.read_point_cloud("front1.ply")
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.05) # voxel_size in m
        pass

    def CamToBase(self):
        # returns a new point cloud with points in the robot frames  coordinate system (frame G, base frame) from their location in the camera frame (frame C)
        # T_C = self.getCameraFrame()
        # P_G = np.array(T_C)[0:3,3] # translation to origin of robot coordinate system (3 x 1) from camera frame origin
        # R_CG = np.array(T_C)[0:3,0:3]  # rotation matrix that describes the basis vectors for the camera frame in frame G
        # Q_G = P_G + (R_CG * Q_C) # Get point location in robot frame by scaling the coordinates by the basis vectors of the camera frame and adding the translation P_G
        CamToBaseTransform = np.array(self.getCameraFrame())
        transformedPCD = self.pcd.transform(CamToBaseTransform)
        baseSphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.25)
        # robotFrame = self.ur5.fkine_all(ur5.q)[0] # SE3
        o3d.geometry.TriangleMesh.create_sphere
        baseFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        cameraFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        cameraFrame.transform(CamToBaseTransform)
        # o3d.visualization.draw([self.pcd,baseSphere,baseFrame,cameraFrame])
        return transformedPCD
        

    def plot(self):
        '''
        x,y,z=[],[],[]
        self.pcd = self.CamToBase()
        for point in self.pcd.points:
            xi,yi,zi = tuple(point)
            x.append(xi)
            y.append(yi)
            z.append(zi)
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.set_aspect('equal')
        ax.scatter(x,y,z)
        plt.grid()
        plt.xlim([-3,3])
        plt.ylim([-3,3])
        plt.show()
        '''
        ur5 = self.ur5
        
        env = ur5.plot(ur5.q) # PyPlot backend
        T_C = self.getCameraFrame()
        T_C.plot(frame="C",length=0.1)
        self.pcd = self.CamToBase()
        env.hold() 

    def getCameraFrame(self):
        # Robot joint angles need to be prior
        # Returns a SE3 Spatial Math Object (4 x 4  Homogenous Transform) corresponding to robot camera frame 
        T_N = self.ur5.fkine_all(self.ur5.q)[-1] # T_N - end-effector frame (before optoforce/gripper)
        d = 0.1 # distance between end-effector frame origin and center of camera frame along z-axis (m)
        P_C = np.array([0,0,d]) # Translation from frame T_N to origin of camera frame (m)
        # theta = np.radians(90) # Rotation about the Z-axis between the camera frame and end-effector frame (None,)
        T_C = T_N # * SE3.Tz(d) # * SE3.Rz(theta) # Camera coordinate frame
        # T_C.plot(frame="C",length=0.1)
        return T_C 


class o3dModel():
    def __init__(self):
        self.pcd = o3d.io.read_point_cloud("front1_2.ply")
        self.rtbModel = rtbModel()
    
    def plot(self):
        T_0 = self.rtbModel.ur5.fkine_all(self.rtbModel.ur5.q)[0] # SE3 base frame
        # (0,0,0) in open3d is origin of cameraFrame in real-life
        # Want to find where robot base frame is relative to cameraFrame
        print(T_0)
        
        # baseFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        # baseFrame.transform(np.array(T_0))
        # self.pcd.transform(np.array(self.rtbModel.getCameraFrame()))
        T_C = self.rtbModel.getCameraFrame()
        R_C = np.array(T_C)[0:3,0:3]
        P_C = np.array(T_C)[0:3,3]  # camera origin in base frame       
        # Need to determine -P_C in base frame
         
        X_hat_base_in_camera = np.matmul(R_C,np.array([1,0,0])) 
        Y_hat_base_in_camera = np.matmul(R_C,np.array([0,1,0]))
        Z_hat_base_in_camera = np.matmul(R_C,np.array([0,0,1]))
        # baseSphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.25)
        # o3dFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
        baseFrameMesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5) 
        cameraFrameMesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5,origin=[0,0,0]) 
        baseFrameRotation_in_camera = np.column_stack((X_hat_base_in_camera,Y_hat_base_in_camera,Z_hat_base_in_camera))
        baseFrameMesh.rotate(baseFrameRotation_in_camera)
        baseFrameMesh.translate(-P_C,relative=False)
        displacement = np.array(cameraFrameMesh.get_center()) - np.array(baseFrameMesh.get_center())
        print(displacement)
        print(LA.norm(displacement))
        # print(baseFrameMesh.get_center())
        # print(baseFrameRotation_in_camera)
        # print(LA.norm(np.array(X_hat_base_in_camera)))
        #for col in baseFrameRotation_in_camera.T:
        #    print(LA.norm(col))
        # put cameraFrame in o3d world (should have same coordinates as in rtb model)
        # cameraFrameMesh.transform(np.array(self.rtbModel.getCameraFrame())) # SE3
        # print(np.array(self.rtbModel.getCameraFrame()))
        # print(cameraFrameMesh.get_center() - np.array([0.05168,0.05168,0.05168]))
        o3d.visualization.draw([self.pcd,cameraFrameMesh,baseFrameMesh])


o = o3dModel()
o.plot()
# r.CamToBase()
# r = rtbModel()
# r.plot()


'''
meshFrames = [] # frames for open3d
urFrames = ur5.fkine_all(ur5.q) # spatial math list of SE3 link frames

for T_i in urFrames:
    # transforms each T_i into open3d coordinate system
    # +x right, +y down, +z forward
    # T_i = T_i * SE3.Rx(np.radians(90)) * SE3.Rz(np.radians(180))
    linkFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
    linkFrame.transform(np.array(T_i))
    meshFrames.append(linkFrame)


baseSphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
baseFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
cameraFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
cameraFrame.transform(np.array(T_C)) # relative = false
meshFrames.extend([baseFrame,cameraFrame,baseSphere])
'''
'''
T_C = T_C * SE3.Rx(np.radians(90)) * SE3.Rz(np.radians(180))
camFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
camFrame.transform(T_C)
pcd = o3d.io.read_point_cloud("front1.ply")
meshFrames = [camFrame]
meshFrames.extend([pcd])
o3d.visualization.draw(meshFrames)
'''