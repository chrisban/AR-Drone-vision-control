Based off of Puku0x's CV Drone project: github.com/puku0x/cvdrone, which uses a GPL license. For more information on licensing, see the file GPL.txt.

This project's code can be found in the /src/project folder.
Project members: Christopher Ban, David Mercer, Dalin Williams

This Project uses Visual Studio and is intended for Parrot AR Drone's to locate and follow a neon yellow construction hat.

State machine diagram can be viewed here: https://github.com/chrisban/AR-Drone-vision-control/blob/master/stateMachine.jpg
The algorithm designed to control the drone consists of two major parts. The first part consists of a large state machine designed to move through several states that depend on both the drone and its environment in order to control it's actions. The second part of flight control consists of fuzzy logic. This algorithm works by using a fuzzy algorithm to determine how far away from optimal orientation and positioning the drone is at any given time. The result of this algorithm is then used to determine the speed to either turn, strafe, or move the drone in any given direction.

For image processing, we found that utilizing the extra functions included in openCV (Hough lines, canny, etc.) resulted in slower computation and weird video lagging/freezing at times. Because of these issues, the image processing algorithm we used to control the drone was kept fairly simple. We make some assumptions about the environment as well, such as optimal lighting, and no other round and brightly yellow colored objects in the area.

Specifically, as the drone rotates, the video feed's rgb channels are split and thresholding is performed on the green channel. From there the image is eroded, dilated, and smoothed which will result in a black image with the target  highlighted in white. If there is no target that matches the hat's shape in that image, the drone will remain in it's searching state. Otherwise, the drone will move to the target and begin following at an acceptable distance.


Some functions were not added such as finding target using downward facing camera due to low resolution or other limiting factors.
