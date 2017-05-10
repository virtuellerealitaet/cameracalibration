# Camera Calibration for monocular and stereo camera setup using OpenCV

Info : The provided code has been tested only under Visual Studio 2015, x64, Windows 10.

## Method 1 : Single Camera calibration using live camera stream


The program uses the connected camera it finds first when using the live-feed option.

start program with the following parameters
```
singlecamcalibration.exe -w 9 -h 6 -pt chessboard -n 15 -d 2000 -o mycam -op -oe
```

In case you are using the Sony Eye 3 camera using the dedicated driver for this camera may work better. This driver can be chosen by the flag -useSonyEye

```
singlecamcalibration.exe -useSonyEye -w 9 -h 6 -pt chessboard -n 15 -d 2000 -o mycam -op -oe
```

increase the number of images (-n) to increase calibration quality
increase the duration between image capture (-d) in milliseconds to have more time repositioning the camera/checkerboard
change the output name of the calibration file (-o) if required
skip writing of the feature points (-op) or extrinsic camera parameters (-oe) for the captured images if not required
When starting the program you can adjust the camera angle to a central view before hitting 'g' to start the capture process.

Then move the camera from angle to angle. Stop movement while the program captures an image (image flashes) to reduce image motion blur and rolling shutter effects as much as possible.

After the defined number of images has been captures the program computes a camera calibration and writes the data into the defined file. The resulting average pixel error when - based on the derived camera matrix - projecting the detected features points back into the captured images is written into the console (mostly about 0.1 and 0.3 pixels for a successful calibration). In the calibrated state you can hit 'u' to toggle between the lens distortion corrected version and the original camera image. Distortion correction is performed using a build-in OpenCV function.

## Method 2 : Single Camera calibration using image list

capture images for calibration manually
create xml containing image file names by using image list generator from OpenCV or by using the Matlab script provided below
run calibration using the following parameters
```
singlecamcalibration.exe files.xml -w 9 -h 6 -pt chessboard -o gopro3 -oe
```

Example image list (files.xml) :

```
<?xml version="1.0"?>
<opencv_storage>
<images>
./frames/frame_00001.png
./frames/frame_00059.png
./frames/frame_00126.png
./frames/frame_00181.png
./frames/frame_00353.png
./frames/frame_00439.png
./frames/frame_00478.png
./frames/frame_00560.png
./frames/frame_00589.png
./frames/frame_00646.png
./frames/frame_00695.png
./frames/frame_00959.png
./frames/frame_01062.png
./frames/frame_01153.png
./frames/frame_01461.png
./frames/frame_01516.png
./frames/frame_01672.png
./frames/frame_01759.png
./frames/frame_01822.png
./frames/frame_01868.png
./frames/frame_01917.png
./frames/frame_02115.png
./frames/frame_02199.png
./frames/frame_02292.png
./frames/frame_02731.png
./frames/frame_02875.png
</images>
</opencv_storage>
```

Output : Example camera calibration for GoPro 3 Black (Superview Mode)

```
%YAML:1.0
calibration_time: "Tue May  9 19:23:56 2017"
image_width: 1920
image_height: 1080
board_width: 9
board_height: 6
square_size: 1.
flags: 0
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 8.7984333348949622e+02, 0., 9.6701979597278034e+02, 0.,
       8.8574368328130231e+02, 4.9043015925119653e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ -2.3470770830549290e-01, 6.1921637942502571e-02,
       1.1228509374665811e-04, 1.6030680031137417e-04,
       -7.6287817408728937e-03 ]
avg_reprojection_error: 9.0238007052623903e-01
```
