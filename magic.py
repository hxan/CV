import open3d 
import numpy as np
import pyrealsense2 as rs
import time
import cv2
class Open3dCalc:
	def __init__(self, xyz):#初始化，传入一个n*3的矩阵（numpy类型 ）
		self.xyz=xyz#
		self.pcd=open3d.geometry.PointCloud()#生成点云
		self.pcd.points = open3d.utility.Vector3dVector(xyz)
		self.pcd = self.pcd.voxel_down_sample(voxel_size=0.005)
		self.axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])

	def __del__(self):
		pass

	def planeSeg(self):#平面分割，在初始化之后调用，返回一个n*3的矩阵
		self.plane_model, self.inliers = self.pcd.segment_plane(distance_threshold=0.008	,
                                         ransac_n=400,
                                         num_iterations=1000)
		[a, b, c, d] = self.plane_model
		print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

		self.inlier_cloud = self.pcd.select_by_index(self.inliers)

		return np.asarray(self.inlier_cloud.points)



s = time.time()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()


fo = open('name.txt', "w")
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

for i in range(0,640):
    for j in range(0,480):
        distance = depth_frame.get_distance(i,j)
        if distance==0.0 or distance==-0.0:
            continue
        t = rs.rs2_deproject_pixel_to_point(depth_intrin,[i,j],distance)
        fo.write( str(t).strip('[').strip(']')+'\n' )





dets = np.loadtxt('1602150616.621838',delimiter=',')

a = Open3dCalc(dets)

pic_raw = cv2.imread('aa.jpg')
pic = cv2.imread('aa.jpg')



dict_x_miny=[None] * 651
for i in range(0,650):
	dict_x_miny[i]=-999999

#寻找每个x对应下y最小的点
for i in a.planeSeg():
    temp=rs.rs2_project_point_to_pixel( depth_intrin,[i[0],i[1],i[2]] )
    if( dict_x_miny[ round(temp[0]) ]<=round(temp[1]) ):
    	dict_x_miny[ round(temp[0]) ]=round(temp[1])
    cv2.circle(pic_raw,(round(temp[0]),round(temp[1]) ),1,(0,0,255))
#print(dict_x_miny)
#pic_raw = cv2.GaussianBlur(pic_raw,(7,7),0)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))#定义结构元素的形状和大小
pic_raw = cv2.erode(pic_raw, kernel)#腐蚀操作
pic_raw  = cv2.blur(pic_raw,(2,2))
#approx = cv2.approxPolyDP(dict_x_miny[0:640], 2, True)
    # cv2.drawContours(img, approx, -1, (0, 0, 255), 3)
#cv2.polylines(pic, [approx], True, (0, 255, 255), 2)







max_x=0
min_x=640
#找x最大最小的点
for i in range(0,640):
	if dict_x_miny[i]!=-999999:
		cv2.circle( pic,(i,dict_x_miny[i]),1,(0,0,255) ) 
		if i>max_x:
			max_x=i
		if i<min_x:
			min_x=i



cv2.circle( pic,(max_x,dict_x_miny[max_x]),3,(0,255,0) )
cv2.circle( pic,(min_x,dict_x_miny[min_x]),3,(0,255,0) )
#
#print(dict_x_miny)
#cv2.imshow('picture',pic)
'''
print("asas")
for x in range(0,640):
	for y in range(0,480):
		print( pic[x,y] )
'''

cv2.imshow('picture',pic)
cv2.imshow('picture_raw',pic_raw)
print(time.time()-s)
while True:
	cv2.waitKey(1)
