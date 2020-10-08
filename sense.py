import pyrealsense2 as rs
import numpy as np
#import cv2
import time

start=time.time()

name=str( time.time() )
fo = open( name, "w")

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
#color_frame = frames.get_color_frame()

#depth_image = np.asanyarray(depth_frame.get_data())
	
#color_image = np.asanyarray(color_frame.get_data())

#depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

for i in range(0,640):
    for j in range(0,480):
		distance = depth_frame.get_distance(i,j)
		t = rs.rs2_deproject_pixel_to_point(depth_intrin,[i,j],distance)
		fo.write( str(t).strip('[').strip(']')+'\n' )

print(name)
print(time.time()-start)
pipeline.stop()
