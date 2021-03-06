## Camera Calibration:

We are going to be using the kalibr package to do all our calibration. In this example we are going to be using the point grey [chameleon mono](https://www.ptgrey.com/chameleon-13-mp-mono-usb-2-sony-icx445-camera) camera. We aim to first get the calibration matrix for the camera and then work on calibrating this camera with an attached IMU.

First download and install the kalibr package (here we started a new workspace named kalibr_workspace apart from vins_workspace for vins_source, to install kalibr). We are using ros indigo on ubuntu 14.04 LTS. We will be following the [multi-camera calibration](https://github.com/udrg/kalibr/wiki/multiple-camera-calibration) tutorial. We need to record a ros bag of our camera feed. It is important to note that we will keep the "camera system" stationary. The camera will be placed on a stationary surface, and the grid moved in front of it.

The target that we will be using is the [aprilgrid](https://github.com/udrg/kalibr/wiki/calibration-targets#a-aprilgrid). This has been printed to size at the universities printing center. We will use the default yaml file (here is the "april_6x6_80x80cm.yaml" corresponding to "Aprilgrid 6x6 0.8x0.8 m", and put it into kalibr_workspace) for the calibration.

We will be using the ROS [point grey camera driver](https://github.com/udrg/pointgrey_camera_driver). We will be launching the camera and specifying to use a low FPS. This low FPS will allow use to gather less data, so we can run the calibration quicker. Note that it can be calibrated at higher fps but the calibration process will take far longer then needed. We are looking for unique orientations of our target.

Our launch file for calibration names calibration-kalibr-static.launch:
```xml
<launch>

<!-- Determine this using rosrun pointgrey_camera_driver list_cameras.
       If not specified, defaults to first camera found. -->
  <arg name="camera_serial" default="0" />
  <arg name="calibrated" default="0" />

  <!-- Launch our ROS camera node -->
  <group ns="camera">
    <node pkg="nodelet" type="nodelet" name="camera_nodelet_manager" args="manager" />

    <node pkg="nodelet" type="nodelet" name="camera_nodelet" args="load pointgrey_camera_driver/PointGreyCameraNodelet camera_nodelet_manager" >
      <param name="frame_id" value="camera" />
      <param name="serial" value="$(arg camera_serial)" />
      <param name="frame_rate" value="4" />
    </node>
  </group>

  <!-- View the raw output -->
  <node pkg="image_view" type="image_view" name="xtioncam_view" >
    <remap from="image" to="/camera/image_raw" />
  </node>

</launch>
```

In a separate window, run our launch file (here is under vins_workspace, and maybe need root authority (sudo su first) to run because we need to access serial port):
```
roslaunch /path/to/calibration-kalibr-static.launch
```

In a separate window run our rosbag recorder (here is under kalibr_workspace, as it will be convenient to read the .bag file in next step):
```
rosbag record -O <output-file> <topics-to-record>
rosbag record -O calibration_static.bag /camera/image_raw
```
After moving the grid in front of the camera for an enough time, we end rosbag and get the .bag file in the workspace (here until ending rosbag, "calibration_static" will be a .active file).

We will now run the kalibr suite for muli-camera calibration to get our intrinsic calibration matrix (here is under kalibr_workspace). Kalibr will generate multiple files that show both the error and the properties of the values found. After running the program, we can use this camera calibration to find the IMU to camera matrix.

```
kalibr_calibrate_cameras --bag [filename.bag] --topics [TOPIC_0 ... TOPIC_N] --models [MODEL_0 ... MODEL_N] --target [target.yaml]

rosrun kalibr kalibr_calibrate_cameras --bag ./calibration_static.bag --topics /camera/image_raw --models pinhole-radtan --target ./april_6x6_80x80cm.yaml
```


## Camera IMU Calibration:

Next we are going to find the camera to IMU relationship. To do this we will will be using the camera intrinsics we found in the first part. Unlike the first part, we will be dynamically moving the camera and keeping the target still. A rosbag will be recorded with both the camera and IMU topics recorded. This can then be processed to find the end values of the system. Note we remove the slow camera rate of 4 fps and go back to our normal 30fps.

We will be using the point grey [chameleon mono](https://www.ptgrey.com/chameleon-13-mp-mono-usb-2-sony-icx445-camera) camera along with the microstrain 3dmgx2 25 IMU. The ROS driver that we will be using is the [microstrain_comm](https://github.com/udrg/microstrain_comm). We will use a low IMU rate so that we can match the lower FPS of the camera.


Our launch file for calibration names calibration-kalibr-dynamic.launch:
```xml
<launch>

<!-- Determine this using rosrun pointgrey_camera_driver list_cameras.
       If not specified, defaults to first camera found. -->
  <arg name="camera_serial" default="0" />
  <arg name="calibrated" default="0" />

  <!-- Launch our ROS camera node -->
  <group ns="camera">
    <node pkg="nodelet" type="nodelet" name="camera_nodelet_manager" args="manager" />

    <node pkg="nodelet" type="nodelet" name="camera_nodelet" args="load pointgrey_camera_driver/PointGreyCameraNodelet camera_nodelet_manager" >
      <param name="frame_id" value="camera" />
      <param name="serial" value="$(arg camera_serial)" />
      <param name="frame_rate" value="60" />
    </node>
  </group>

  <!-- View the raw output -->
  <node pkg="image_view" type="image_view" name="xtioncam_view" >
    <remap from="image" to="/camera/image_raw" />
  </node>

  <!-- imu -->
  <node pkg="microstrain_comm" type="microstrain_comm_node" name="microstrain_comm_node" output="screen">
    <param name="rate" value="low" />
    <param name="verbose" value="falsed" />
    <param name="quiet" value="true" />
  </node>

</launch>
```


In a separate window, run our launch file:
```
roslaunch /path/to/calibration-kalibr-dynamic.launch
```

In a separate window run our rosbag recorder:
```
rosbag record -O <output-file> <topics-to-record>
rosbag record -O calibration_dynamic.bag /camera/image_raw /imu/data
```


We will need to create a [imu.yaml](https://github.com/udrg/kalibr/wiki/yaml-formats#imu-configuration-imuyaml) file that describes the intrinsic properties of our IMU. These can normally be gotten from your manufacturer's specification page.

Example imu.yaml file:
```yaml
#Accelerometers
accelerometer_noise_density: 7.845e-04   #Noise density (continuous-time)
accelerometer_random_walk:   0.0196      #Bias random walk

#Gyroscopes
gyroscope_noise_density:     5.24e-04   #Noise density (continuous-time)
gyroscope_random_walk:       8.727e-05  #Bias random walk

# Update methods
update_rate:                 100.0      #Hz (for discretization of the values above)
rostopic:                    /imu/data  #Topic data is published to
```

Run our calibration using the kalibr suite IMU camera calibration method:
```
kalibr_calibrate_imu_camera --bag [filename.bag] --cam [camchain.yaml] --imu [imu.yaml] --target [target.yaml]

kalibr_calibrate_imu_camera --bag ./calibration_dynamic.bag --cam ./camchain-calibration_static.yaml --imu ./imu.yaml --target ./april_6x6_80x80cm.yaml
```
