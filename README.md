## qr_code_finder
`` roslaunch launch/qrfinder.launch [input_topic] [output_topic]``

Reads images from specified input topic and publishes cropped to the most likely QR code fourth on the specified output topic.

Default input: `/bottomcamera/image_raw`
Default output: `/qr_parts`




#### Camera 
`` $ ./launch/run_debug.sh ``

Starts a [camera_publisher](https://github.com/AscendNTNU/camera_publisher) node as well as launching the qr finder connected to the correct topics and image visualization.
The camera_publisher launchfile needs v4l2-ctl to calibrate properly. Install with ``sudo apt install v4l-utils``


#### ZBar
If the [zbar_ros](https://github.com/ros-drivers/zbar_ros) package is installed, `roslaunch launch/zbar.launch` will automatically launch the ZBar reader and publish results to `/barcode`

>note: There is no reason to run ZBar on the fourth detection as there is no complete QR code
