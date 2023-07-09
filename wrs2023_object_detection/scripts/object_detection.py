# Author : Dwi Kurnia Basuki
# Modified by Muhammad Labiyb Afakh

import sys
import cv2
import numpy
import torch
import os
import socket
import pykinect_azure as pykinect
import sys
import math
import rospy
from rospy import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

numpy.set_printoptions(threshold=sys.maxsize)

object_flag = 0

sys.path.insert(1, '../')

device = torch.device('cpu')

# Load Model
model = torch.hub.load('/home/mobinuc/01/seed_ws/src/WRS2023/wrs2023_object_detection/yolov5', 'custom', path='/home/mobinuc/01/seed_ws/src/WRS2023/wrs2023_object_detection/object.pt', source='local', force_reload=True)  # local repo

model.to(device)


def centroid(vertexes):
     _x_list = [vertex [0] for vertex in vertexes]
     _y_list = [vertex [1] for vertex in vertexes]
     _len = len(vertexes)
     _x = sum(_x_list) / _len
     _y = sum(_y_list) / _len
     return(int(_x), int(_y))




if __name__ == "__main__":
	rospy.init_node('obect_detection', anonymous=False)
	_publisher = rospy.Publisher('object_detection', PoseStamped, queue_size=10)
	_publisher_name = rospy.Publisher('object_name', Int8, queue_size= 1)
	_rate = rospy.Rate(100)


	# Initialize the library, if the library is not found, add the library path as argument
	pykinect.initialize_libraries()

	# Modify camera configuration
	device_config = pykinect.default_configuration
	device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
	# device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
	device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED
	# print(device_config)

	# Start device
	device = pykinect.start_device(config=device_config)

	cv2.namedWindow('Transformed Color Image',cv2.WINDOW_NORMAL)

	# print(device.calibration)
	intrinsic_device = device.calibration.color_params


	while not rospy.is_shutdown():

		# Get capture
		capture = device.update()

		# Get the color image from the capture
		# ret_color, color_image = capture.get_color_image()

		# Get the 3D point cloud
		ret, points = capture.get_pointcloud()

		# Get the colored depth
		# ret, depth_image = capture.get_transformed_depth_image()

		# Get the color image in the depth camera axes
		ret, color_image = capture.get_transformed_color_image()

		if not ret:
			continue

		results = model(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
		points_map = points.reshape((color_image.shape[0], color_image.shape[1], 3))

		# Calculate Multiple Object
		panjang = len(results.pandas().xyxy[0])
		converted = results.pandas().xyxy[0].to_numpy()


		# Draw Bounding Box on Object Detected
		true_tail_point = (0, 0)
		true_head_point = (0, 0)
		true_left_point = (0, 0)
		true_right_point = (0, 0)
		tail_coordinate = (0, 0, 0)
		head_coordinate = (0, 0, 0)
		left_coordinate = (0, 0, 0)
		right_coordinate = (0, 0, 0)
		centroid_coordinate = (0, 0, 0)
		final_depth_left = 0
		final_depth_head = 0
		final_depth_tail = 0
		final_depth_right = 0
		countTail = 0
		countHead = 0
		x_head_world = 0
		y_head_world = 0
		z_head_world = 0
		x_tail_world = 0
		y_tail_world = 0
		z_tail_world = 0
		x_pos = 0
		y_pos = 0
		z_pos = 0

		try:	
			object_flag = 0	
			for i in range(panjang):
				cv2.rectangle(color_image, (int(converted[i][0]), int(converted[i][1])),
							  (int(converted[i][2]), int(converted[i][3])), (255, 0, 0), 1)
				cv2.putText(color_image, f'{converted[i][6]} {int(converted[i][4] * 100)}%',
							(int(converted[i][0]), int(converted[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (100, 255, 0), 1)
				
				if converted[i][6] == 'caffe latte':
					countTail += 1
					coordinates_tail = (converted[i][0], converted[i][2], converted[i][1], converted[i][3])
					centerx_tail, centery_tail = (numpy.average(coordinates_tail[:2]), numpy.average(coordinates_tail[2:]))
					true_tail_point = (int(centerx_tail), int(centery_tail))

					x_pos = (points_map[int(centery_tail), int(centerx_tail), :][0])
					y_pos = (points_map[int(centery_tail), int(centerx_tail), :][1])
					z_pos = (points_map[int(centery_tail), int(centerx_tail), :][2])

					object_flag = 1


				elif converted[i][6] == 'milk coffee':
					countHead += 1
					coordinates_head = (converted[i][0], converted[i][2], converted[i][1], converted[i][3])
					centerx_head, centery_head = (numpy.average(coordinates_head[:2]), numpy.average(coordinates_head[2:]))
					true_head_point = (int(centerx_head), int(centery_head))

					x_pos = (points_map[int(centery_head), int(centerx_head), :][0])
					y_pos = (points_map[int(centery_head), int(centerx_head), :][1])
					z_pos = (points_map[int(centery_head), int(centerx_head), :][2])

					object_flag = 2

				elif converted[i][6] == 'pringles':
					coordinates_left = (converted[i][0], converted[i][2], converted[i][1], converted[i][3])
					centerx_left, centery_left = (numpy.average(coordinates_left[:2]), numpy.average(coordinates_left[2:]))
					true_left_point = (int(centerx_left), int(centery_left))
					x_pos = (points_map[int(centery_left), int(centerx_left), :][0])
					y_pos= (points_map[int(centery_left), int(centerx_left), :][1])
					z_pos = (points_map[int(centery_left), int(centerx_left), :][2])

					object_flag = 3
				else:					
					continue

			



		except Exception as e:
			print(e)
		
		object_flag_msg = Int8()
		object_flag_msg.data = object_flag
		_publisher_name.publish(object_flag_msg)


		pose_msg = PoseStamped()
		pose_msg.header.frame_id	= "object"
		pose_msg.header.stamp		= rospy.Time.now()

		pose_msg.pose.position.x 	= z_pos
		pose_msg.pose.position.y	= -x_pos
		pose_msg.pose.position.z	= -y_pos

		pose_msg.pose.orientation.x	= 0
		pose_msg.pose.orientation.y	= 0
		pose_msg.pose.orientation.z = 0
		pose_msg.pose.orientation.w = 1

		_publisher.publish(pose_msg)


		# Overlay body segmentation on depth image
		cv2.imshow('Transformed Color Image',color_image)

		# Press q key to stop
		if cv2.waitKey(1) == ord('q'):
			break

		_rate.sleep()