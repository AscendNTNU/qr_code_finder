## qr_code_finder
`` roslaunch launch/qrfinder.launch ``

Reads images from ` /cv_camera/image_raw`
Publishes images cropped to the most likely QR code fourth on ` /qr_code_finder/output_image`




#### ZBar
If the [zbar_ros](https://github.com/ros-drivers/zbar_ros) package is installed, `roslaunch launch/zbar.launch` will automatically launch the ZBar reader and publish results to `/barcode`

>note: There is no reason to run ZBar on the fourth detection as there is no complete QR code