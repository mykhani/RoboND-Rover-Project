[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[image1]: ./images/detecting_rocks.png
[image2]: ./images/detecting_obstacles.png
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Search and Sample Return Project (solution)

For details regarding setting up the project, please see this [link](https://github.com/udacity/RoboND-Rover-Project/blob/master/README.md)

### Notebook Analysis
#### 1. Detection of Rocks and Obstacles
##### Detecting Rocks
For detecting rocks, I have used the opencv color filtering function for detecting rocks. First off, to use opencv functions, we need to convert the image first from RGB to BGR format. This can be done by flipping back the RGB values in image taken as numpy array. The next step is to convert the image from BGR colorspace to HSV (Hue, Saturation and Value). The reason for chosing HSV colorspace was because of the ease of color selection. The Hue is the basic color while Saturation and Values refers to lights and darks respectively. This means that we can easily detect lighter (less saturation) and darker (higher value) shades of a certain color (Hue). This makes it easier to select the different shades of Rock color. To determine the range of HSV values for rock sample, I opened the rock example image in GIMP (GNU Image Manipulation Program), and using the eye dropper tool, I picked 2 color samples, one with a lightest and other with darkest shade of rock color. One point to consider is that opencv and GIMP, both use different ranges for the values of H, S and V. For instance, in OpenCV, Hue ranges from 0 to 179, Saturation and Value both from 0 to 255. While in GIMP, Hue ranges from 0 to 360 while Saturation and Values both range from 0 to 100. Therefore to work with both, we need to convert values picked from GIMP to OpenCV compatible ones by introducing a scaling factor. Finally, OpenCV's "inRange" function is used to select all the shades of the rock, in between the 2 color points picked earlier. This gives a mask image, consisting of ones where the color criteria is met, effectively pin-pointing the location of rock pixels. 
```python
def detect_rocks(img):
    # convert rgb to bgr for opencv
    img = img[:,:,::-1]
     # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    # Color scaling between GIMP and OpenCV
    gimp2cv2_hue_scale = (179 / 360)
    gimp2cv2_sat_scale = (255 / 100)
    gimp2cv2_val_scale = (255 / 100)
    
    lower_rock = np.array([45 * gimp2cv2_hue_scale, 25 * gimp2cv2_sat_scale, 44 * gimp2cv2_val_scale])
    upper_rock = np.array([54 * gimp2cv2_hue_scale, 255, 255])
    # Threshold the HSV image to get only rock colors
    mask = cv2.inRange(hsv, lower_rock, upper_rock)
    
    return mask
```
Below is the image showing rock detection.
![detecting rocks][image1]
##### Detecting Obstacles
For detecting obstacles, I used the same function as that for detecting navigable terrain with slight change i.e. selecting all the pixels that are not navigable terrain or more specifically, falling below the color threshold for navigable path.
```python
def detect_obstacles(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be below all three threshold values in RGB
    # below_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[below_thresh] = 1
    # Return the binary image
    return color_select
```
Below is the image showing rocks detection.
![detecting obstacles][image2]
#### 2. Image processing

### Autonomous Navigation and Mapping
#### 1. Implementing Perception Step
#### 2. Launching the simulator
#### 3. Results
