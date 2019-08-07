# Stop Sign Detection

Detects if there is a stop sign in front of the vehicle.

  - Publish:
    - **UInt8 on "~/stop_sign"**: Boolean value, true if sign detected, false otherwise
    - **UInt32 on "~/sign_size"**: Size of sign in pixels
    
## Required ROS Params

`~camera_topic` name of camera topic this node subscribes to 

`~cascade_path` path to classifier to load and run detections on

## Other ROS Params

`~max_size` mamimum size of the area a stop sign can be detected in

`~min_size` minimum size of the area a stop sign can be detected in

`~scale_factor` 

`~min_neighbors` specifies the minimum number of matching color neighbors to be considered

`~trigger_width` 

`~red_mask` enables or disables a red mask on the image

`~hue_lower` lower bound value for hue

`~saturation_lower` lower bound value for saturation

`~value_lower` lower bound value for value

`~hue_upper` upper bound value for hue

`~saturation_upper` upper bound value for saturation

`~value_upper` upper bound value for value

## Example Launch File

Make sure to have [simple_camera_publisher](https://github.com/LTU-AutoEV/simple_camera_publisher) in your ROS workspace.

By default, the included launch file will launch `simple_camera_publisher example.launch`. You may launch the camera somewhere else.

Update the camera subscription topic `camera_topic` in the launch file if you are launching the camera on a different topic.

```xml

<node name="stop_sign_detection" pkg="sign_detection" type="stop_sign_detection">
        <param name="camera_topic" type="string" value="/my_camera/cam_pub/image_raw" />
        <param name="cascade_path" type="string" value="$(find sign_detection)/classifiers/stop_sign.xml" />
</node>

```


