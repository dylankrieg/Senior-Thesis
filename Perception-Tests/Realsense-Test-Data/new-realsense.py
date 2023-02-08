# Requires Python3 Kernel (3.6) as pyrealsense is installed here on hand
import pyrealsense2 as rs
import numpy as np
import math
import cv2
# Used to display Matplotlib plots in Jupyter
import matplotlib.pyplot as plt
import time

# PIL used to save images as pngs
from PIL import Image

pipe = rs.pipeline()
config = rs.config()

# Getting information about the connected realsense model (device object) - D410
pipeProfile = config.resolve(rs.pipeline_wrapper(pipe))
device = pipeProfile.get_device()

# Setting attributes for stream
# Depth Stream (640 x 480) 30 fps - D410 Sensor has max 1280 x 720
config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)

# No Color, only Infrared on D410 Stream (640 x 480) 30 fps - D410 Sensor has max 1280 x 720
config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)

# Starting the pipeline based on the specified configuration
# pipe.start(config)

def takeImages(pipe,config,saveImages=False):
    # Starting the pipeline based on the specified configuration
    pipe.start(config)
    # Return's unprocessed depth and infrared images as tuple of 2 numpy array
    frames = pipe.wait_for_frames()
    depthFrame = frames.get_depth_frame() # pyrealsense2.depth_frame
    colorFrame = frames.get_color_frame()
    # irFrame = frames.get_infrared_frame() # pyrealsense2.video_frame
    # other method is get_infrared_frame
    rawColorImage = np.asanyarray(colorFrame.get_data())
    rawDepthImage = np.asanyarray(depthFrame.get_data())
    subFix = "Back" # append to end of file paths
    if saveImages:
        np.save(f"newRealsense/depthImage{subFix}",rawDepthImage)
        np.save(f"newRealsense/colorImage{subFix}",rawColorImage)
        colorIM = Image.fromarray(rawColorImage)
        colorIM.save(f"newRealsense/colorImage{subFix}.jpeg")
    pipe.stop()
    return rawDepthImage,rawColorImage
    # Set all distances equal or greater than 32" (812 mm) to 812
    # depthImage[depthImage >= 812] = 812
    # print(np.max(depthImage))
    # depthImage = 255 - ((depthImage/812)*255)# Apply linear scaling to depthImage with darker regions further away
    # plt.imshow(depthImage,cmap="gray")
    # plt.show()
    
# def processRawDepthImage(rawDepthImage)
rawDepth,rawIR = takeImages(pipe,config,True)
plt.imshow(rawDepth)
plt.show()
plt.imshow(rawIR)
plt.show()
