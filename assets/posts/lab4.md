Lab 4
=====

## Overview and Motivations - H. Nichols

The goal of this lab was to park the car a specified distance away from an orange cone and have the car follow a red line. The algorithms needed to solve these problems are implemented using images from the Zed camera instead of the LIDAR sensor used in previous labs. This assignment is fundamental in helping the team develop skills needed to properly use the Zed camera and navigate the robot based only on vision data. Having experience with visual servoing is important as we progress forward in the class so that we can better understand the tradeoffs of using the Zed camera versus LIDAR sensor for autonomous navigation of the race car. 

## Proposed Approach - C. Gadson

The team used visual servoing to approach the problems of parking in front of a cone and following a line. Continuous visual feedback from the Zed camera on the car is used to calculate necessary inputs such as the angle the car must turn in order to be directly facing the cone or following a line, the speed at which the car should travel, and how far the car must drive forward or reverse in order to be the appropriate distance from the cone. The project was split into four separate tasks to make the project more manageable: detect, locate and visualize, and park in front of the cone then follow the red line.

### Initial Setup - C. Gadson

The team split up into sub-teams to implement the code where the number of teammates on each subteam directly correlates with how time consuming or difficult the task seemed: two people worked on detecting the cone, two on estimating cone location, one on parking, and the last teammate on line following. Since a majority of the project is reliant on the cone detection algorithm, this was implemented first in order for the other sections to be developed in parallel thereafter.

There are several different ways to approach the implementation of the lab, and we made decisions that optimized for both efficiency of the code and further development of skills teammates want to gain. For example, a working line following algorithm could have been completed with few modifications of the car parking code. However, we reasoned that developing a line following algorithm from scratch without dependence on any other data besides the camera output may be the most efficient method and would allow each teammate to have a similar amount of development to do - which is something everyone is interested in. The information pipeline for line following is made much shorter by directly taking in images from the camera and only making the most necessary calculations instead of using the same pipeline that the parking algorithm needs. Many decisions similar to this one were made to approach the lab in a way the whole team was comfortable with.

### Technical Approach

#### Detect the Cone - M. Picchini

The first step in the lab is to detect an orange cone and return a bounding box of where the cone lies in the image. We decided to try three different algorithms for cone detection: SIFT + RANSAC, template matching, and color space image segmentation and filtering. After we tested all three, we would decide which algorithm would be the best to integrate into our project.

The SIFT + RANSAC and template matching algorithms were primarily implemented by the staff, while the color space image segmentation and filtering algorithm we wrote from scratch. The SIFT + RANSAC algorithm takes advantage of the features of a given image in order to detect the cone. The template matching algorithm looks for the staff-provided cone template image in each test image every time, best detecting the cone when it is depicted almost identical to the template image. Then since the color segmentation algorithm was written solely by the team, the algorithm will be explained in greater detail.

In order to detect the cone with color space image segmentation and filtering, we rely on the fact that the cone is orange. An RGB image is translated into HSV values in order to provide an easier means of filtering for the orange color of the cone independent of the intensity of lighting in the environment. After this translation, we create a mask of the image where the HSV values are within a certain range of the hue, saturation, and value that represent the color orange we want to filter. (Figure 0) 

<center>
<img src="assets/images/lab4/color-segmentation.png" alt="image goes here" style="width:500px;">

Figure 0: Depicting ranges of HSV values used to filter mask
</center>

The following mask has values of 1 where the pixel is within the desired ranges, and a 0 otherwise (Figure 1). In order to get parts of the cone that were incorrectly undetected, we use a morphological transformation function called closing. However, this resulted in a bounded box that was usually smaller than the ground truth. In order to fix this, we used another function to dilate the mask, and expand the cone. Unfortunately, this dilation expanded some noisy parts of the image (Figure 2).   


<left>
<p style="float: left; font-size: 9pt; text-align: center; width: 45%; margin-right: 1%;"><img src="assets/images/lab4/mask-cone2.png" style="width: 100%">Figure 1:  Initial filtered mask with closing function</p>
</left>

<left>
<p style="float: right; font-size: 9pt; text-align: center; width: 45%; margin-left: 1%;"><img src="assets/images/lab4/cone-mask1.png" style="width: 100%">Figure 2: Final filtered mask - the cone is expanded, but so is noise</p>
</left>

<br><br><br><br><br><br><br><br><br><br><br><br><br>

We then used an opencv function to find the contours of the mask. Using the logic that the cone should be the largest chunk in the image to filter out noise and another function was used to find the largest area contour (Figure 3). After isolating the cone, we find and return the bounding box.

<center>
<img src="assets/images/lab4/cone-countour1.png" alt="image goes here" style="width:500px;">

Figure 3: Modified image where contours are blue, bounding box is red, and ground truth bounding box is green
</center>

#### Locate and Visualize the Cone - J. Aleman

Once the cone is detected in the camera image, we transform that location from a camera 2D point to a location in the real world. A problem with this transformation is that the 2D image is a collapsed space of the 3D world, and is missing the valuable information that the z-axis would provide. This is an example of the distortions and misrepresentations that happen when someone takes a picture with a camera:

<center>
<img src="assets/images/lab4/Snail-left.jpg" alt="image goes here" style="width:250px;">
<img src="assets/images/lab4/Snail-right.jpg" alt="image goes here" style="width:250px;">

Figure 2: Perspective illusion results from collapsing 3D into 2D. Photo Cred: J. Bever
</center>


So in order to transform from 2D back to 3D we need to make some assumptions. In this case, we rely on the fact that both the car and the cone will be on the same plane to estimate positions (z = 0). There are two potential methods, one being to use the camera calibration files (camera matrix) to calculate a 3D light ray using the 2D pixel coordinates. We could determine where the ray intersects the 2D plane on the ground, and that intersection provides the 3D coordinates of the cone. The other method would be to assume that all camera pixels are on a 2D plane. We can then use a homography matrix to transform from 2D plane (the floor) to another 2D plane (camera space) and vice versa. A drawback of the first option is that it relies on a few extra assumptions we were not comfortable with: that the camera is perfectly parallel to the floor and the distance between the camera and the floor can be measured accurately.
Considering this, we chose the second option, drawing a rectangle of known dimensions on the floor. Knowing the 3D coordinates of each vertex with respect to the car frame as well as the pixel coordinates on the camera space, a homography matrix can be calculated:

The pixel can then be converted to a world coordinate using:

$$ \begin{pmatrix} x \\\ y \\\ 1 \end{pmatrix} = \boldsymbol{H} * \begin{pmatrix} u \\\ v \\\ 1 \end{pmatrix} $$

Regardless of which option we chose, there was one pitfall to do calculations. If the cone position was not actually on the floor, both methods would result in invalid coordinates. To avoid this we set up a threshold of values that we would be able to convert from. This was the camera’s horizon line.


<center>
<img src="assets/images/lab4/homography_lab4.jpg" alt="image goes here" style="width:500px;">

Figure 3: Using homography we map a 2D plane into another 2D plane
</center>


#### Park the Car - H. Nichols
The technical approach in programming a cone follower and parker consisted of implementing PID controllers on both the distance and angle from the cone. This section is referred to as cone follower more often than cone parker because it follows the location of the cone and only when it achieves its desired distance does it park.

The algorithm uses the output from locating the cone to determine the distance and angle from the cone. It does this by converting the point location of the cone from cartesian to polar coordinates. With this point in the correct form, a PID control on both of the angle and distance help to move the car to the desired position 1.75 feet from the cone. The following equations describe how the distance error and angle error are calculated, and the PID control is the sum of all of these components.

Error Calculation:
	$$ errorDistance = currentDistance - desiredDistance$$
	$$ errorAngle = currentAngle - desiredAngle$$

Note: the desired angle is always zero because when the car is parked it must be aligned perpendicular to the cone.

For proportional control:
	$$ P\_{distance} = Kp * errorDistance $$
	$$ P\_{angle} = Kp * errorAngle $$
	
For derivative control:
	$$ D\_{distance} = K\_d* (errorDistance - previousDistError) $$
	$$ D\_{angle} = K\_d * (errorAngle - previousAngleError) $$

For integral control:
	$$ I\_{distance} = K\_i * \sum(errorDistance) $$
	$$ I\_{angle} = K\_i * \sum(errorAngle) $$
	

We implement control on both the speed and the steering angle instead of setting an arbitrary speed and only fixing the steering angle because we thought it was important for the car to recognize that its speed should vary depending on its distance to the cone. When far away from the cone, the car moves more swiftly; inversely, when close to the cone but not the desired distance away, the car creeps toward the cone until reaching its end point.

#### Follow the Line - A. Hrabchak
The line following algorithm imports only the zed camera’s image from the racecar. It then erases the top 65 percent of the image so that only the bottom 35 percent is used in the rest of the script. We then are able to detect the red line and create a rectangular box around the detected area using the same method as in the cone detection algorithm. This rectangle is then analyzed, and we extract the horizontal component of its middle point. The difference between this coordinate and the the horizontal middle of the image is the error and proportional term used in our controller. The controller only uses a proportional term as the derivative and integral terms were not necessary at the speed we chose to drive the car.

### ROS Implementation - J. Aleman

As mentioned before, at the beginning of the week, we divided into sub-groups to tackle the different sections of the lab. This task separation is reflected on our ROS architecture. Each one of those tasks has its own node.

<center>
<img src="assets/images/lab4/ros-arch-full.jpg" alt="image goes here" style="width:800px;">

Figure 4: Cone Follower ROS architecture
</center>


The line follower, is implemented separately as this is because estimating the position of the cone in 3D space was not necessary for making a line follower.

Using this ROS architecture had the following advantages:
Each node could be developed independently. So each sub-team had full ownership of it’s node.
It kept each member of the team efficiently contributing code without suffering from merge conflicts
Was easy to debug, as each ROS topic could be probed.


## Experimental Evaluation - J. Aleman

Experimental evaluation was done on a per node basis first. Feeding fake inputs  into the listening ROS topics in VMs. Once each node was working we loaded them into the car for full system testing. The following sections describe our observations during testing.

### Testing Procedure

#### Detect the Cone - C. Gadson
Testing for cone detection involved testing the three detection algorithms against test images and publishing an output image from the car to confirm that the algorithm we implemented accurately finds the cone. We first determined the accuracy of the SIFT + RANSAC, template matching, and color segmentation algorithms by scoring the results of each with the respective IoU metric scores. Our second method of testing included publishing a modified image taken from the Zed camera on the car that includes the bounding box of the detected cone by our algorithm. By listening to the topic to which we were publishing the modified image, we were able to test and confirm that our algorithm was properly bounding the cone as the target rather than another object or space in the image as depicted in the video below (Figure 5).

<center>
<img src="assets/images/lab4/3rd-person-view-of-car-for-cone-detection.JPG" alt="image goes here" style="width:210px;">
<img src="assets/images/lab4/Cone-detection-from-zed-camera.png" alt="image goes here" style="width:500px;">

Figure 5: Output of the cone detection algorithm to confirm functionality
</center>

#### Locate the Cone - R. Chang & J. Aleman
After the homography matrix was implemented, we passed the function a myriad of inputs to determine if there were any that would create invalid results. Once the algorithm to locate the cone was working properly, we tested the car’s estimated cone positions by comparing them to measurements with a measuring tape.

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/camcnGjv_u4" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 1: Cone Position Estimation
</center>

#### Park the Car - H. Nichols
To test the cone parking algorithm, we ran a number of test cases to see where the algorithm fails and/or needs improvement. We tested the code by placing the cone in a variety of angular positions and varying distances from the race car to see if it could accurately navigate to its stopping point. We implemented a proportional-only control at first, and fine tuned that gain empirically by testing the car as described above. The derivative and integral controls were added one after another, refining each gain before adding the next just as we did for the proportional-only control. Adding each term one by one made it easier to appreciate how each term contributed to motion of the car.


#### Follow the Line - A. Hrabchak
Testing the line following algorithm involved running the code on several different shaped taped courses that were set up throughout the building. There was a variety of patterns that the car would follow. The proportional gain was found quickly through this testing.


### Results - A. Hrabchak

The results include both the outcomes of performing our testing procedures as well as the final product.

While testing the separate cone detection algorithms, we found that color space image segmentation and filtering had the best results due to the algorithm’s resilience to changes in lighting and the unusual, inconsistent orange color of the cone. This algorithm matched the ground truth bounding box of the cone nearly perfectly after being tuned, thus we chose this algorithm as the one to integrate into this project. However, it would perform as well if there was a lot of orange in the image, but since orange is an uncommon color in the environment, the algorithm performs well. The template algorithm was the second best, proving somewhat successful when the cone was almost identical to that of the test cone image, but not as accurate for deviations in lighting and slight obstructions. The SIFT + RANSAC was the least effective because the cone does not have very many distinctive features for the algorithm to use to its advantage.

After implementing and testing the cone locating algorithm, we discovered that all values above a certain horizontal pixel line resulted on invalid results. We called this the horizon line and filtered the input to only accept values below it. We found the estimates to be really good when the cone was close to the car but got progressively worse the further the cone was. This makes sense as the resolution of the camera is only so good and when the cone is far away a single pixel error on the cone localization amounts for a large error in position.

Then for the cone parking algorithm, we found that if the cone was not in the Zed camera’s point of view (x value is zero), the cone parking code would throw errors. This was solved by adding an edge case of outputting zero the to Ackermann drive message to bypass this error. In general, the car jerks a bit when it moves towards the cone. As the car accelerates, the bottom bumper sinks towards the ground distorting the homography matrix which leads to the car’s calculated distance not being accurate. Once the car comes to a stop it realizes this error and starts moving again to fix it. The final result after sufficient testing for cone parking are shown in Videos 2 and 3.

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/GorZBxDwMGA" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 2: Cone follower: Demonstration of parking and following
</center>


<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/cqr3-TgrWJ0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 3: Cone follower: Backwards movement demonstration
</center>

Finally, for the line follower we found that because the zed camera is only feeding us an image from the left side of the car, the car would follow a line with its front left tire on the physical line. To ensure that the car had and even field view on each side we added an offset to the error term. The error is now the difference between the horizontal component of the center of the line and a point directly to the right of the middle of the screen. Another issue that we ran into during testing was that if the lines had a very tight turn, the car would lose track of the line and just continue forward instead of turning. Ultimately this was due to the limit of the cars turning radius as even when manually controlling the car, we still could not make such a sharp turn. It is also interesting to note that the color bounds used in the orange cone detection algorithm detect the deep red color of the tape used for line following testing. The final result of this implementation is in Video 4.

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/ztoLsq2yuwA" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 4: Line follower different light conditions
</center>

Overall, both the cone follower and the line follower performed successful after sufficient rounds of testing as depicted above. The cone parking and following algorithm correctly finds the cone, turns toward it, and stops a foot and a half away. The line follower was able to complete the two marked courses in the lab classroom at full speed. The following videos depict the final results.

## Lessons Learned - R. Chang

During this lab, our team found that collaborating and organizing what was to be done allowed us to accomplish the technical aspects of our lab quickly. 


### Technical Conclusions - M. Picchini

This lab taught us the basics of object detection and localization and applying this to useful tasks such as parking and line following.  

We learned about the pros and cons of different algorithms for detecting objects and implemented these algorithms, as well.  We also learned and implemented locating an object in space using homography matrices.  After  accomplishing these tasks, we were able to use these results with previous weeks’ knowledge of PID to accomplish parking and line following.  Some things that caused us to stumble were noise in the background for cone detection using color, setting the PID gains correctly, and dealing with edge cases in the line follower when the car could not turn enough to follow the line.  Overall, we improved the functionality of our car to not only be able to follow walls, but also detect and follow objects and a line


### CI Conclusions

1. Hailey - I found that this lab was much more successful in terms of efficiency and delegation of work than our first lab. From the start we split up the lab into four sections and communicated about what each person would need as an output from another teammate in order to implement their section of the code. This systematic approach was very time efficient as we all were able to work independently and then come together with progress. This method tested our communication skills as each teammate was heavily dependent on another teammate’s work. Though this method was very successful for completion of the lab, in the future I’d like to additionally set aside time for each teammate explain the process of their code as I want a deeper understanding of what my other teammates did to implement their section.

2. Raphael - This lab, we tried immediately splitting the work before even thinking about the technical aspects. We read the lab and decided which parts depended on which, and also figured out the availability of our members throughout the week to decide how to distribute the parts. This allowed us to jump into working much faster than last time. Our improved efficiency allowed us to have the lab working earlier before the deadline than last lab. Our division of work proved to improve the results of our lab significantly. All of us were able to fully devote our time to one aspect of the lab, allowing us to fine tune it and make it more robust. However, this isolated each member to only understand their part of the code. In the future, it would be beneficial for us educationally to set aside extra time to review each other’s code and understand how the system works as a whole.

3. Michael - The collaboration and organization of the lab was much more successful than in the previous week.  After a week of rushing to finish certain parts of the previous lab, we decided upon a new strategy.  We immediately divided the lab and distributed tasks to each person, setting early deadlines for each divided part.  Because each of our parts had some connection to the other parts, we discussed what each part should deliver to the others.  This allowed for everyone to work on their respective parts without having to wait for another part to be finished.   This organization removed a lot of that stress and allowed more to be accomplished.  While this was more efficient, it did segment members to certain tasks and ultimately less involvement and learning for members in certain areas.  In the future labs we will most likely continue this organization and distribution of work, but have internal, hands on, team presentations so each member understands how each part works.

## Future Work - C. Gadson

Although we successfully used visual servoing to complete this lab, there are several improvements that could be made including more accurately detecting the cone by filtering out large objects the same color as the cone, better determining the the distance from the car to the cone when they are more than three meters apart, and refining the PID controllers. Specifically for the line following algorithm, It would be interesting to implement an acceleration cap on the cone following algorithm to reduce error and prevent jerkiness.

<center> Edited by J. Aleman and C. Gadson</center>

