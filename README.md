[//]: # (Image References)
[image_0]: ./misc/rover_image.jpg
[image1]: ./images/detecting_rocks.png
[image2]: ./images/detecting_obstacles.png
[image3]: ./images/settings.png
[image4]: ./images/result.png
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
All the image process is performed in function process_image(). Here, the first step is to extract the x,y co-ordinates of the position and yaw angle of the robot in world reference frame. The index of the current image frame is used to map robot telemetry data to a particular frame.
```python
    xpos = data.xpos[data.count]
    ypos = data.ypos[data.count]
    yaw = data.yaw[data.count]
```
The next step is perform the perspective transform, i.e. to convert the robot's first person perspective based camera image to a top view based perspective and them perform path, rock and obstacle detection.
```python
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped, (160, 160, 160))
    rocks_select = detect_rocks(warped)
    obstacles_select = detect_obstacles(warped, (160, 160, 160))
```
Next, the x,y position of individual pixels of navigable path, rocks and obstacles is extracted from the image frame (given in robot body reference frame) and converted to world reference frame.
```python
    # 4) Convert thresholded image pixel values to rover-centric coords
    xpix , ypix = rover_coords(threshed)
    rock_xpix, rock_ypix = rover_coords(rocks_select)
    obstacle_xpix, obstacle_ypix = rover_coords(obstacles_select)
    
    # 5) Convert rover-centric pixel values to world coords
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, xpos, ypos, yaw, 200, 10)
    navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, 200, 10)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, 200, 10)
```
The world map image is populated by marking the obstacle pixels in the Red channel, rock pixels in Green and Navigable path in Blue channel.  
```python
    data.worldmap[obstacle_y_world, obstacle_x_world, 0] = 255    
    data.worldmap[rock_y_world, rock_x_world, 1] = 255
    data.worldmap[navigable_y_world, navigable_x_world, 2] = 255
```
Finally the camera image, the transformed image and world map is combined together into a larger image which is then overlayed on top of output video.
```python
    output_image[0:img.shape[0], 0:img.shape[1]] = img
    # Let's create more images to add to the mosaic, first a warped image
    # Add the warped image in the upper right hand corner
    threshed_3d = np.dstack((threshed * 255, threshed * 0, threshed * 0))
    output_image[0:img.shape[0], img.shape[1]:] = threshed_3d

    # Overlay worldmap with ground truth map
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
    # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
```
### Autonomous Navigation and Mapping
#### 1. Perception Step
Here's the final implementation of perception step.
```python
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
	# These source and destination points are defined to warp the image	
	# to a grid where each 10x10 pixel square represents 1 square meter
	# The destination box will be 2*dst_size on each side
	dst_size = 5 
	# Set a bottom offset to account for the fact that the bottom of the image 
	# is not the position of the rover but a bit in front of it
	# this is just a rough guess, feel free to change it!
	bottom_offset = 6
	image = Rover.img
	xpos = Rover.pos[0]
	ypos = Rover.pos[1]
	yaw = Rover.yaw
	source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
	destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
	warped = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
	threshed = color_thresh(warped, (160, 160, 160))
	rocks_select = detect_rocks(warped)
	obstacles_select = detect_obstacles(warped, (160, 160, 160))
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
	Rover.vision_image[:,:,0] = obstacles_select * 255
	Rover.vision_image[:,:,2] = threshed * 255
    # 5) Convert map image pixel values to rover-centric coords
	xpix , ypix = rover_coords(threshed)
	obstacle_xpix, obstacle_ypix = rover_coords(obstacles_select)
    # 6) Convert rover-centric pixel values to world coordinates
	obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, xpos, ypos, yaw, 200, 10)
	navigable_x_world, navigable_y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, 200, 10)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
		
	Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1    
	Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10

    # 8) Convert rover-centric pixel positions to polar coordinates
	dist, angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
    # Rover.nav_dists = rover_centric_pixel_distances
    # Rover.nav_angles = rover_centric_angles
	Rover.nav_angles = angles
	if rocks_select.any():
		rock_xpix, rock_ypix = rover_coords(rocks_select)
		rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xpos, ypos, yaw, 200, 10)
		
		rock_dist, rock_angle = to_polar_coords(rock_xpix, rock_xpix)
		# find the position of first pixel of rock
		rock_idx = np.argmin(rock_dist)
		rock_x = rock_x_world[rock_idx]
		rock_y = rock_y_world[rock_idx]
		Rover.worldmap[rock_y, rock_x, 1] = 255
		Rover.vision_image[:,:,1] = rocks_select * 255
	else:
		Rover.vision_image[:,:,1] = 0
    
	return Rover
```
The perception step in file perception.py is similar to the process_image() function from the notebook. It is responsible for taking the Rover state containing current camera image and telemetry data, performing image analysis for detecting navigable terrain, obstacles and rocks and returning the Rover state containing all the processing results. First, the Rover's position and orientation in world frame is extracted from the Rover state class object and then image processing is performed (perspective transform, path/obstacle/rock detection) and a world map consisting of the navigable path, rocks and obstacles is constructed.

One major difference from process_image() is that instead of assigning value 255 in the Red and Blue channels for showing obstacles and navigable terrain respectively, the value is incremented each time a pixel is detected as either as an obstacle or navigable path and the navigable path is given more preference. It means that if a same pixel is detected as an obstacle and navigable path, it will be treated as navigable path. 
```python
Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1    
Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10
```
The navigable pixels are then converted to polar co-ordinates, as the pairs of distance and angle from robot front. These pairs are used by decision step to command the rover.

Finally, due to perspective transform, the rocks get detected as beams, instead of points. To convert these beams to points, only the closest rock pixel is treated as the location of a rock sample
#### 2. Decision Step
The decision step function in file decision.py is responsible for sending the throttle and steering commands to rover based on the results of perception step. By default, the bare minimum functionality is provided and as of now, I have kept it unchanged to meet the minimum requirement for a passing submission. The logic behind decision making is very simple, the code first checks if there are sufficient (greater or equal to Rover.stop_forward number) navigable terrain pixels in front of the rover and if yes, it sets the forward movement by sending throttle command. Also it takes the average of all the navigable angles (which tends to lie towards bigger cluster of navigable terrain pixels) and use it to set the steer angle command.
**UPDATE: Now I have added mechanism to get out of stuck loops. The way it works is that the robot keeps track of its movement and saves a snapshot of the last time when it moves more than a threshold. If a certain time passes and the robot has still not covered the threshold distance, despite having the throttle, it means it's stuck somewhere and then it stops and tries to spin to get out of loop. I have also added a threshold for minimum traversable distance infront of robot instead of just relying on the number of angles available. This also helps the robot in avoiding obstacles.

#### 3. Launching the simulator
Below is the image of settings used while launching the simulator. I have used ubuntu 16.04.5 host running on VMWare virtual machine.
![Simulator settings][image3]
#### 4. Results
The rover successully detected most of the rock samples and meets the requirements of mapping at least 40% map with more than 60% fidelity. Although at certain times, the robot gets stuck in rocks, in tight locations and sometimes keep circling in circular loops. I plan to improve the decision step bit more in future and to add the functionality of picking and returning the sample to start location.  ![Final result][image4]
