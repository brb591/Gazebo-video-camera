# Streaming video from Gazebo through ROS2 to a Python program

Use one of the example worlds that gets installed with ROS2 and Gazebo (this assumes ROS2 Foxy on Ubuntu 20.04):

They are in ```/opt/ros/foxy/share/gazebo_plugins/worlds```.  The simple one is ```gazebo_ros_camera_demo.world``` and you can see that it creates a camera and uses the ```libgazebo_ros_camera.so``` plugin to stream the camera data to ROS2:

### Start Gazebo with an example world
```
gazebo --verbose gazebo_ros_camera_demo.world
```

You should see gazebo start up and see a ball bouncing, along with a screen showing the ball bouncing.  The screen is showing the ball and that is also what is being streamed to ROS2.

### Get a list of topics from ROS2:
```
ros2 topic list -t
```
The output:
```
/clock [rosgraph_msgs/msg/Clock]
/demo_cam/camera1/camera_info [sensor_msgs/msg/CameraInfo]
/demo_cam/camera1/image_raw [sensor_msgs/msg/Image]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

### Get a sample of the camera data
The camera data is on the topic ```/demo_cam/camera1/image_raw``` and we can take a look at it to verify that it is really being streamed:

```
/demo_cam/camera1/image_raw
```
The output:
```
header:
  stamp:
    sec: 243
    nanosec: 252000000
  frame_id: camera_link
height: 480
width: 640
encoding: rgb8
is_bigendian: 0
step: 1920
data:
- 181
- 181
- 181
- 177
- 177
- 177
- 177
- 177
- 177
- 178
- 178
- 178
<CTRL-C>
```
### Learn a little bit more about these messages:
```
ros2 interface show sensor_msgs/msg/Image
```
The output:
```
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)
```

# Now we are ready to access this data from Python.

## What can we see from python?
It is useful to know what we can see using a python program interacting with ROS2.  Remember that we are still running the sample world in gazebo from above.

```
python3 get-ros2-info.py
```
The output:
```
ACTIVE TOPICS
('/clock', ['rosgraph_msgs/msg/Clock'])
('/demo_cam/camera1/camera_info', ['sensor_msgs/msg/CameraInfo'])
('/demo_cam/camera1/image_raw', ['sensor_msgs/msg/Image'])
('/parameter_events', ['rcl_interfaces/msg/ParameterEvent'])
('/rosout', ['rcl_interfaces/msg/Log'])
ACTIVE NAMESPACES
('ros2_info_node', '/')
```
You can see that the list of topics is still the same, however you have to pay attention to namespaces.  Topic names in ROS2 are appended to the active namespace.  A topic name with a forward slash at the beginning is an absolute name and without the slash, it is relative (to the namespace).  The topic ```/demo_cam/camera1/camera_info``` is in the ```/demo_cam``` namespace and its absolute topic name is ```/demo_cam/camera1/camera_info``` while its relative topic name is ```camera1/camera_info```.  (This can get confusing!)  But you do see that our python program doesn't see a ```/demo_cam``` namespace, right?

That's not really a problem, per se.  We just specify the absolute topic name when subscribing to the video stream:
```
python3 basic-camera-view.py /demo_cam/camera1/image_raw
```
This becomes a problem if you want to do more advanced stuff, like have a lot of identical robots running in Gazebo and you want to control them or view their video stream from your python program.  Wouldn't it be nice to name your robots ```robot1```, ```robot2```, and ```robot3```, with each robot is streaming video on the ```camera/image_raw``` topic, and your python program only needs to know the name of the robot to get access to its video stream?  Yes, indeed.

It turns out that you can map the namespace when initializing your Node in the python program: (https://docs.ros2.org/latest/api/rclpy/api/node.html).  Specify the robot name as the namespace in our (just slightly) more advanced example:
```
python3 stream-ros2-camera.py /demo_cam camera1/image_raw
```

# The big gotcha with streaming camera data from Gazebo to Python through ROS2
## Warning: needless complaining

None of the python tutorials I could find specified anything for the Quality of Service profile other than a queue of 10.  This queue setting means that your subscription is using the system default.  Looking at the rclpy documentation (https://docs.ros2.org/latest/api/rclpy/api/node.html) is not at all helpful in this regard.

The ROS2 documentation is a little better, but you are unlikely to find it unless you have done a lot of research on ROS2.  If you go through all the tutorials in order, you will eventually get to one about dealing with lossy networks (https://docs.ros.org/en/foxy/Tutorials/Quality-of-Service.html).  I'm running everything on a single PC with no network involved, so why would I pay attention to a tutorial about lossy networks, right?  That tutorial leads you to a concept page about quality of service (https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html) which again, but you are thinking in terms of bad network connections so why bother?  Sorry, you need to set this.

If you leave the Quality of Service settings as the default in all the python examples you find, your program will probably just be permanently stuck in the ```_wait_for_ready_callbacks``` function.  Yuck.

**Use the sensor data profile, because the camera is a sensor:**
```
from rclpy.qos import qos_profile_sensor_data

...

self.subscription = self.create_subscription(Image,
    topic_name,
    self.listener_callback,
    qos_profile_sensor_data)
self.subscription
```

The end.