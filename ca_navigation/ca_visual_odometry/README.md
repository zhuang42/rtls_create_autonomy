# Visual Odometry

[Reference link @ ROS Discourse](https://discourse.ros.org/t/ros-visual-odometry/2465)

----

## System architecture

So, the graph of our system looks like this:

![System architecture](media/architecture.png)

As you can see in this picture, we have Raspberry Camera connected and raspicam creating multimedia pipeline and sending video from camera to `gst-launch`. The latter then transmit the image to our server over UDP. gscam will broadcast the video to `/raspicam/image_raw` topic. This image should be rectified with `image_proc` node. And finally, rectified image is taken by `mono_odometer`, which handles it and computes position and orientation of the robot publishing this data straight to the `/vision/pose` topic.

## Preparing the environment

This tutorial will consider that you already have the hardware conected. For full information, see the reference link.

### Getting started

Firstly, ssh **into Raspberry** and start broadcasting video **to our server**:

```sh
$ raspivid -n -w 640 -h 480 -b 1000000 -fps 40 -t 0 -o - | gst-launch-1.0 -v fdsrc ! h264parse ! rtph264pay config-interval=10 pt=96 ! udpsink host=<IP> port=9000
```

Find the IP of the server using `ifconfig`.

Then launch the ROS nodes in the server:

```
$ roslaunch ca_visual_odometry stream_test.launch
```

## Calibrating the camera

You will need to have printed [this checkerboard](media/check-108.pdf).

Run the calibration node with this command:

```
$ roslaunch ca_visual_odometry calibrate.launch
```

![calibration node opened](media/calibration_01.png)

In order to get a good calibration you will need to move the checkerboard around in the camera frame such that:

* checkerboard on the camera’s left, right, top and bottom of field of view
    * X bar - left/right in field of view
    * Y bar - top/bottom in field of view
    * Size bar - toward/away and tilt from the camera
* checkerboard filling the whole field of view
* checkerboard tilted to the left, right, top and bottom

As you move the checkerboard around you will see three bars on the calibration sidebar increase in length. When the **CALIBRATE** button lights, you have enough data for calibration and can click **CALIBRATE** to see the results.

![calibration done](media/calibration_02.png)

Calibration can take about a minute. The windows might be greyed out but just wait, it is working.

