Based off of Puku0x's CV Drone project: github.com/puku0x/cvdrone, which uses a GPL license. For more information on licensing, see the file GPL.txt.
This project's code can be found in the /src/project folder.
Project members: Christopher Ban, David Mercer, Dalin Williams

This Project uses Visual Studio and is intended for Parrot AR Drone's to locate and follow a neon yellow construction hat.

The state machine has five main modes: 
- initialization states
- searching/locating state
- Moving state
- Following State
- Braking state

On run the AR Drone will take off in the air and begin rotating in place until it locates the hard hat. It recognizes the hard hat with the use of OpenCV functions. As the drone rotates, the video feed's rgb channels are split and thresholding is performed on the green channel. From there the image is eroded, dilated, and smoothed which will result in a black image with the target  highlighted in white. If there is no target that matches the hat's shape in that image, the drone will remain in it's searching state. Otherwise, the drone will move to the target and begin following at an acceptable distance.

Some functions were not added such as finding target using downward facing camera due to low resolution or other limiting factors.
