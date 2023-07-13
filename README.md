# WRS2023
This code is used to compete at World Robot Summit 2023, Japan. On the customer interaction task, we try to handle the number of queue by using a robot as a cashier and use the robot to bring back the cancellation product to its shelf. 2 Camera were used to monitor the queue and the object that will be returned.

# Install some packages required
## 1. Seed Solution R7 ROS.
This repository is not standalone, it needs [Seed Solution R7 ROS](https://github.com/seed-solutions/seed_r7_ros_pkg) repository.

Copy the map.pgm and map.yaml inside maps folder in this repository to the seed_r7_navigation/maps/ folder.
Change this path to your map.pgm file path.
```yaml
image: /home/mobinuc/01/seed_ws/src/seed_r7_ros_pkg/seed_r7_navigation/maps/map.pgm
```
## 2. Object Detection.
To use the object detection package by using a Azure Kinect Camere DK, [Azure Kinect Camera SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) and pykinect_azure packages should be installed.
```bash
pip install pykinect_azure
```
and [YoloV5](https://github.com/ultralytics/yolov5).
Change that line of code to your yolo and model path.
```python
# Load Model
model = torch.hub.load('/home/mobinuc/01/seed_ws/src/WRS2023/wrs2023_object_detection/yolov5', 'custom', path='/home/mobinuc/01/seed_ws/src/WRS2023/wrs2023_object_detection/object.pt', source='local', force_reload=True)  # local repo
```

## 3.Queue Monitoring.
This package requires YoloV8 that can be installed by 

```bash
pip install ultralytics
```

## 4. NFC Reader.
To read the data of NFC card, pyscard should be installed.

```bash
pip install pyscard
```


# Compile those packages and this repository.
## If u do not have clone this repository try to clone it first.
```bash
cd /path/to/your/catkin/src/
git clone https://github.com/labiybafakh/WRS2023
```
Since the size of a bag file is large, please download it separately and locate the practice-2.bag inside bag folder.

## Compile all of the packages
```bash
catkin build
```

# Run the system.
## Run the robot.
```bash
roslaunch wrs2023_navigation navigation.launch
```

## Run the object detection.
```bash
rosrun wrs2023_object_detection object_detection.py
```

## Run the queue monitoring.
```bash
rosrun wrs2023_queue_monitor monitor.py
```

# Play the visualization using bag file.
If u just want to run the visualization using a bag file,  you don't need to do step 2, 3, and 4 on installing packages. The robot will start to move when the bag file runs about 60 seconds, so please wait.
## 1. Run the navigation package.
```bash
roslaunch wrs2023_navigation navigation.launch
```
## 2. Play the bag file.
```bash
cd bag/
rosbag play practice-2.bag
```

# Team
1. Dwi Kurnia Basuki
2. Fernando Ardila
3. Rama Okta Wiyagi
4. Muhammad Labiyb Afakh
5. Syadza Atika Rahmah
6. Muhammad Ramadhan Hadi Setyawan