import open3d as o3d
import os 
import sys
import numpy as np
import cv2

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

def setup():
    pipeline = rs.pipeline()
    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    # Start streaming
    pipeline.start(config)
    frate_count=[]
    align_to = rs.stream.color
    align = rs.align(align_to)
    return pipeline, align

def grabPointCloud(pipeline,align):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    intrinsics = aligned_frames.profile.as_video_stream_profile().intrinsics
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not aligned_depth_frame or not color_frame:
        return
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    img_color = o3d.geometry.Image(color_image)
    img_depth = o3d.geometry.Image(depth_image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth,convert_rgb_to_intensity=False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image, pinhole_camera_intrinsic, project_valid_depth_only=True)
    return pcd

def downsizePoints(pcd):
    downpcd = pcd.voxel_down_sample(voxel_size=0.10)

    plane_model, inliers = downpcd.segment_plane(distance_threshold=0.03,
                                         ransac_n=3,
                                         num_iterations=1000)
    inlier_cloud = downpcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = downpcd.select_by_index(inliers, invert=True)
    return outlier_cloud, plane_model

def postProcessing(pcd):
    labels = np.array(pcd.cluster_dbscan(eps=0.3, min_points=30, print_progress=False))
    if len(labels) != 0:
        max_label = labels.max()
        #print("fpoint cloud has ",max_label + 1," clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([pcd])
        np_points = np.asarray(pcd.points)
        for idx,value in enumerate(labels):
            index = len(labels)- idx - 1
            if labels[index] < 0:
                np_points = np.delete(np_points,index,0)
        print(np_points)
        pcd.points = o3d.utility.Vector3dVector(np_points)
        aabb = pcd.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)
        obb = pcd.get_oriented_bounding_box()
        obb.color = (0, 1, 0)
        o3d.visualization.draw_geometries([pcd, aabb, obb])
        xyzpoints = np.asarray(obb.get_box_points())
        [a, b, c, d] = plane_model
    return pcd, aabb, obb

def calculations(pcd, aabb, obb, plane_model):
    xyzpoints = np.asarray(obb.get_box_points())
    return xyzpoints

if __name__ == "__main__":
    
    pipline, align = setup()
    while True:
        pcd = grabPointCloud(pipeline,align)
        pcd, plane_model = downsizePoints(pcd)
        pcd, aabb, obb = postProcessing(pcd)
        ___ = calculations(pcd, aabb, obb, plane_model)
        
        
        
