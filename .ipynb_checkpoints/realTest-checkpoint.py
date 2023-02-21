import pyrealsense2 as rs
pipe = rs.pipeline()
profile = pipe.start()
try:
    frame = pipe.wait_for_frames()
    print(frame.profile)

finally:
    pipe.stop()