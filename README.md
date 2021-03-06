### Project Summary
This project is conducted in the Unity simulator, and modeled after the NASA Sample Return Challenge. It will involve image processing, rover decision making, controls, and mapping visualization tasks.

The objective for this project is to autonomously navigate a foreign terrain by processing camera data to calculate open pathways, locate rock samples, and map the surrounding area. 
##### Perspective Transform
We begin the perception pipeline by taking a single image from the Rover camera:
![Original_Image](/assets/Original_Image.png)

This image will then be filtered for field of view that is desired for the rover navigation path, as well as applying a mask:
```
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    return warped, mask

dst_size = 5 
bottom_offset = 6
source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
warped, mask = perspect_transform(grid_img, source, destination)
```
![Perspective_Transform](/assets/Perspective_Transform.png)
##### Color Thresholding
The image within the specified field of view will be filtered by specified gray thresholds to provide a finer resolution of navigable terrain for the Rover:
```
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
                
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

threshed = color_thresh(warped)
plt.imshow(threshed, cmap='gray')
```
![Color_Thresholding](/assets/Color_Thresholding.png)
##### Coordinate Transformations
We need to apply several transforms to go from the Camera frame to the Rover path. These transforms will be a conversion of radial coordinates to Rover space, the Rover space to the World frame, and finally to Rover centric polar coordinates: 
```
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel

# Convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Calculate pixel values in rover-centric coords and distance/angle to all pixels
xpix, ypix = rover_coords(threshed)
dist, angles = to_polar_coords(xpix, ypix)
mean_dir = np.mean(angles)
```
![Coordinate_Transforms](/assets/Coordinate_Transforms.png)
##### Find Rocks!
Part of the objective is to locate rock samples in the terrain. The rocks in this environment have a specific yellow color that is unlike other objects along the terrain, and therefore we can create a color filter to identify these objects:
```
def find_rocks(img, levels):
    rock = ((img[:,:,0] > levels[0]) & (img[:,:,1] > levels[1]) & (img[:,:,2] < levels[2]))
    
    color = np.zeros_like(img[:,:,0])
    color[rock] = 1
    return color

#Define Color Threshold
levels=(110,110, 50)
rockmap = find_rocks(rock_img, levels)
```
![Rock](/assets/Rock.PNG)
### Process Image
Once the coordinate transforms are complete, we can now use the images from the Rover centric frame to apply the mask and use the grayscale filter to determine the navigable path for the Rover:  
```
def process_image(img):
    # Perspective transform and mask
    warped, mask = perspect_transform(img, source, destination)
    
    # Color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    
    # Convert thresholded image pixel values to rover-centric coords
    obs_map = np.absolute(np.float32(threshed) - 1) * mask
    xpix, ypix = rover_coords(threshed)
    
    # Convert rover-centric pixel values to world coords
    world_size = data.worldmap.shape[0]
    scale = 2 * dst_size
    xpos = data.xpos[data.count]
    ypos = data.ypos[data.count]
    yaw = data.yaw[data.count]
    x_world, y_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
    obsxpix, obsypix = rover_coords(obs_map)
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, xpos, ypos, yaw, world_size, scale)
 ```
 ##### Update worldmap
 The project objective is to track the terrain that the Rover has traveled, as well as populating the map with the rock sample locations that are detected:
 ```
    #Update worldmap (to be displayed on right side of screen)
    data.worldmap[y_world, x_world, 2] = 255
    data.worldmap[obs_y_world, obs_x_world, 0] = 255
    nav_pix = data.worldmap[:,:,2] > 0
    data.worldmap[nav_pix, 0] = 0
    
    levels=(110,110, 50)
    rockmap = find_rocks(warped, levels)
    if rockmap.any():
        rock_x, rock_y = rover_coords(rockmap)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world_size, scale)
        data.worldmap[rock_y_world, rock_x_world, :] = 255
    
    output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))
    output_image[0:img.shape[0], 0:img.shape[1]] = img
    warped, mask = perspect_transform(img, source, destination)
    output_image[0:img.shape[0], img.shape[1]:] = warped
```
##### Overlay worldmap with ground truth map
Now that the Worldmap is updated, it will need to be overlayed with the ground truth, or existing known map terrain. This will allow us to determine the percentage of map terrain that the Rover has covered, as well as the fidelity: 
```
    map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
    # Flip map overlay so y-axis points upward and add to output_image 
    output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
    
    return output_image
```
The processing of these images, along with the rover control commands were, stiched together to form a video of the terrain mapping, this video can be found [here](https://github.com/mattlubbers/Udacity-RoboND-Rover/blob/master/Rover_ImageStitching_Video.mp4). 

The full analysis for the sample data image processing can be referenced [here](https://github.com/mattlubbers/Udacity-RoboND-Rover/blob/master/Rover_Lab_Notebook.pdf).
### Process Live Rover Data
The next objective of the project was to take the Rover camera input and process the image to determine control commands to navigate the terrain on the fly! The full Rover image data can be found in the [rover_images](https://github.com/mattlubbers/Udacity-RoboND-Rover/tree/master/rover_images) directory, and a sample image of the live data can be seen below:

![Rover_Data](/rover_images/2018_09_30_02_30_04_746.jpg)
```
def perception_step(Rover):
    dst_size = 5
    bottom_offset = 6
    image = Rover.img
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    destination = np.float32([[image.shape[1] / 2 - dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - bottom_offset],
                              [image.shape[1] / 2 + dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              [image.shape[1] / 2 - dst_size, image.shape[0] - 2 * dst_size - bottom_offset],
                              ])

    # Perspective transform and mask
    warped, mask = perspect_transform(Rover.img, source, destination)

    # Color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshed) - 1) * mask

    # Update Rover.vision_image
    Rover.vision_image[:, :, 2] = threshed * 255
    Rover.vision_image[:, :, 0] = obs_map * 255

    # Convert thresholded image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)

    # Convert rover-centric pixel values to world coords
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    #xpos = data.xpos[data.count]
    #ypos = data.ypos[data.count]
    #yaw = data.yaw[data.count]
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    obsxpix, obsypix = rover_coords(obs_map)
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)

    # Update worldmap (to be displayed on right side of screen)
    Rover.worldmap[y_world, x_world, 2] += 10
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    dist, angles = to_polar_coords(xpix, ypix)
    #nav_pix = Rover.worldmap[:, :, 2] > 0
    Rover.nav_angles = angles
    #data.worldmap[nav_pix, 0] = 0

    levels = (110, 110, 50)
    rockmap = find_rocks(warped, levels)
    if rockmap.any():
        rock_x, rock_y = rover_coords(rockmap)
        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]

        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:, :, 1] = rock_map = 255
    else:
        Rover.vision_image[:, :, 1] = 0
    
    return Rover
```

### Autonomous Mode
Now that the image processing as been adjusted for the live Rover data, we need to add commands for the decision making behavior to determine what the Rover should do!

![Rover_AutonomousMode](/assets/Rover_AutonomousMode.PNG)
##### Longitudinal Controls
First things first, we need to determine if the Rover has a clear path to continue moving forward, or if it should stop for an object:
```
def decision_step(Rover):
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
```
##### Stop! Steer? Move Forward!
If the decision to stop has been made, we can monitor the speed to slow the vehicle, as well as determine if a steering input is necessary to either maneuver around an obstacle or turn around completely in a dead-end:
```
        elif Rover.mode == 'stop':
            # We're not slow enough, keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving, time to turn
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but the path is clear, go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
```
##### Keep Cruisin!
If nothing is in our way, let's keep on the throttle!
```
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
```
### Areas for Improvement
- The rover currently does not have an incintive or penalty to complete the entire terrain mapping coverage
- There is no mechanism of ensuring map quality/fidelity while maneuvering along the path
- Controls can be further refined
- No rock sample collection, only identification and location logging
### Conclusion
The Rover Simulator project was an excellent challenge to familiarize with image processing, decision making, and controls. If there were more time, additional measures would be added to score better on map fidelity and map coverage. However, through the run with the Simulator in Autonomous mode, it was able to identify and locate 5 rocks, with a 71% map fidelity and a 41% map coverage score! The full video of the project results can be found [here](https://github.com/mattlubbers/Udacity-RoboND-Rover/blob/master/Rover_Autonomous_Video.mp4).
