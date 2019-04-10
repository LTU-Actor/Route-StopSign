# Sign Detection

![](img/example.png)

Detects if there is a stop sign in front of the car.

  - Publish:
    - **UInt8 on "~/stop_sign"**: Boolean value, true if sign detected, false otherwise
    - **UInt32 on "~/sign_size"**: Size of sign in pixels

## Launching


Make sure to have [simple_camera_publisher](https://github.com/LTU-AutoEV/simple_camera_publisher) in your ROS workspace.

By default, the included launch file will launch `simple_camera_publisher example.launch`. You may launch the camera somewhere else.

Update the camera subscription topic `camera_topic` in the launch file if you are launching the camera on a different topic.

```xml

<node name="stop_sign_detection" pkg="sign_detection" type="stop_sign_detection">
        <!-- Name of camera topic this node subscribes to -->
        <param name="camera_topic" type="string" value="/my_camera/cam_pub/image_raw" />
        <!-- Classifier to load and run detections on -->
        <param name="cascade_path" type="string" value="$(find sign_detection)/classifiers/stop_sign.xml" />
</node>

```


