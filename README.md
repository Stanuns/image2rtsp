# **How to use?**

## Dependencies

- ROS2 Humble

- gstreamer libs:
```bashrc
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```

- opencv-python:

```bashrc
pip install opencv-python
```

- v4l-utils
```bashrc
sudo apt install v4l-utils
```

## Install

- Navigate to the home directory, create a new directory named `ros2_ws/src`, and then change the current working directory to `ros2_ws/src`:
  ```bashrc
  cd
  mkdir -p ros2_ws/src
  cd ros2_ws/src/
  ```
- Clone or unzip the image2rtsp.zip to `~/ros2_ws/src`:
- Adjust  `parameters.yaml` according to your needs:
  ```bashrc
  gedit ~/ros2_ws/src/image2rtsp/config/parameters.yaml
  ```

# Example ROS2 Image topic stream

    # If camera serves as a source (if u need to transform topic to rtsp, then 'False', default is 'True')
    camera: True
    source: "v4l2src device=/dev/video0"

    topic: "/color/image_raw"  # The ROS2 topic to subscribe to. Dont change, if you use a camera

    # Parameters for both cases
    mountpoint: "/front"        # Choose the mountpoint for the rtsp stream.
                                # This will be able to be accessed from rtsp://<server_ip>/portAndMountpoint
    bitrate: "500"
    framerate: "30"            # Make sure that your framerate corresponds to the frequency of a topic you are subscribing to
    caps_1: "video/x-raw,framerate="
    capr_2: "/1,width=640,height=480"
    # Set the caps to be applied after getting the ROS2 Image and before the x265 encoder. Ignore
    # framerate setting here.
    port: "8554"
    local_only: False          # True = rtsp://127.0.0.1:portAndMountpoint (The stream is accessible only from the local machine)
                               # False = rtsp://0.0.0.0:portAndMountpoint (The stream is accessible from the outside)
                               # For example, to access the stream running on the machine with IP = 192.168.20.20,
                               # use rtsp://192.186.20.20:portAndMountpoint
- Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:
  ```bashrc
  cd ~/ros2_ws/
  colcon build --packages-select image2rtsp
  ```

## Run

- Source `install` and launch the package:
  ```bashrc
  source install/setup.bash
  ros2 launch image2rtsp image2rtsp.launch.py
  ```

## Check the RTSP stream
- VLC player
- Android App

## some issues
### 1.when use a usb camera, RTSP video stream cannot render successfully:
- 1.1 check the usb camera framesizes and framrate:
```bashrc
v4l2-ctl -d /dev/video2 --list-formats-ext

ioctl: VIDIOC_ENUM_FMT
Type: Video Capture

[0]: 'MJPG' (Motion-JPEG, compressed)
  Size: Discrete 1280x720
    Interval: Discrete 0.033s (30.000 fps)
  Size: Discrete 640x480
    Interval: Discrete 0.040s (25.000 fps)
[1]: 'YUYV' (YUYV 4:2:2)
  Size: Discrete 1280x720
    Interval: Discrete 0.100s (10.000 fps)
  Size: Discrete 640x480
    Interval: Discrete 0.040s (25.000 fps)
```
Get the frame rate from "YUYV" part,  the framesize is 1280x720 while the framrate is 10; the framesize is 640x480 while the framrate is 25; 

- 1.2 modify the parameter.yaml 

when we use the 640x480 framesize:
```bashrc
ramerate: "25"
caps_2: "/1,width=640,height=480"
```  
when we use the 1280x720 framesize:
```bashrc
ramerate: "10"
caps_2: "/1,width=1280,height=720"
```
- 1.3 You should colcon build  every time when you modify the .yaml file
```bashrc
colcon build --packages-select image2rtsp
```
- 1.4 ps. check the full info. about the usb camera
```bashrc
sudo v4l2-ctl -d /dev/video2 --all
```