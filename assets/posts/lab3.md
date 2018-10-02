Lab 3
=====

## Overview and Motivations - A. Hrabchak

The two main objectives of this lab were to implement a wall follower and safety controller. The wall follower is designed to have the robot autonomously maintain a certain desired distance from the wall on either its left or right side. The safety controller is used to prevent the car from crashing by forcing a stop when an obstacle is predicted to be in its way. Implementing these two features is essential in building the foundation for our car’s automated driving capabilities. Lab 3 has allowed us to get a hands on experience with taking theory into practice as we transition from simulation to real-time driving as well as learn from the unexpected difficulties that arise.  Our lessons learned in terms of collaboration will help guide us in the next labs to work more efficiently as a team.

## Proposed Approach - R. Chang

Our proposed approach for the wall follower was to use linear regression on the relevant slice of LIDAR data to determine the distance and angle of the wall. A PD controller on the wall distance was to be implemented outputting into the steering angle while driving at a constant speed. For the safety controller, we proposed to account for how quickly the robot is currently driving to determine how close to stop the car from an obstacle. We also proposed to account for the steering angle of the robot to determine what angles to check for obstacles at.

### Initial Setup - R. Chang

The sensor used in this lab is the Hokuyo LIDAR at the front of the robot. The sensor returns distances at an angle range of 270 degrees. A servo is used to control the steering angle and the VESC speed controllers drive the motors.

### Technical Approach - M. Picchini, H. Nichols, R. Chang

To implement the wall follower code, we analyzed the input laser scan data to output a steering angle for the racecar. Our code parses through the LIDAR data and slices it from 15 degrees off the x-axis to 90 degrees perpendicular to the racecar, shown in figure 1. This is achieved by finding the index in the data that corresponds to 15 and 90 degrees. If the car is following the left wall, these indices are added to the midpoint index of the data, and if the car is following a right wall, the indices are subtracted. We then transform this data from polar coordinates into Cartesian x, y, and z coordinates using the read_points method from PointCloud2. The outliers are filtered by removing data points that are at the maximum scan range and checking to only include data in the 15-90 degree window. 
If there is no data in this window, we apply control to the car for it to reorient itself by turning 0.3 radians in the direction it is trying to follow the wall. We then apply a numpy linear algebra method that computes the least-squares solution to find an equation that best fits the data. The proportional control is implemented by using the error in the racecar’s current distance versus the desired distance multiplied by a gain. The derivative control is the angle of the wall relative to the racecar, inverse tangent of the slope, multiplied by a gain. The gains were found by adjusting them in the simulator. The resulting steering angle is the addition of the proportional and derivative control. 

<center> 
<img src="assets/images/lab3/splitting_LIDAR_data.png" alt="Figure 3" style="width:500px;"> 

Figure 1
</center>

The safety controller code is designed as a separate ROS package to dynamically change according to the speed and angle at which we are moving.   It creates a safety box that causes to car to stop if more than a certain amount of points reside within it (Figure 2).  This box can be curved, to represent the projected path of the car.  To accomplish this goal of creating a box and allowing the car to stop, the safety controller first subscribes to the laser scan for data points and the wheels’ angle and speed. The incoming ackermann_msg is used to get the steering angle, and the we additionally subscribe to /vesc/odom to get the velocity of the wheels as determined by the speed controllers.  The angle and speed are used to generate the estimated distance and path the car will take.  The distance “d” the car will take to stop at a certain velocity and deceleration is calculated by using the equation d = v^2/(2a), where “a” is the acceleration, and “v” is the velocity. This distance is then added to a minimum stopping distance in front of the car, so the car stops at an appropriate distance in front of the object and has some buffer room.

<center> 
<img src="assets/images/lab3/racecar_safety_controller.png" alt="Figure 3" style="width:500px;"> 

Figure 2
</center>

If the steering angle is straight, the box is given a length “d” and a width that is the width of the car.  If the steering angle is non-zero, we calculate the curved box by creating three concentric radii using the Ackermann steering geometry (Figure 3), where the radius of the arc travelled by the front wheels is L/sin(theta) and the radius for the rear wheels is L/tan(theta), and L is the length the wheelbase (distance between front and rear wheels). The stopping distance is used as the arc length, and these values (arc radii and arc length) are used to determine the curved box.  Then each point in the laser scan data is checked to see if it is within the box created.  If there are more points than the threshold number of points in the box, the car is stopped.


<center> 
<img src="assets/images/lab3/ackermann_geometry.png" alt="Figure 3" style="width:500px;"> 

Figure 3
</center>

### ROS Implementation - C. Gadson & J. Aleman

We decided to keep each task as a separate ros package. This allowed flexibility for for each contributor to test and modify any file pertaining a rospackage without causing merge conflicts. We ended up with three ROS packages:

* Wall Follower
* Safety Controller
* Test maps

Each one of those has it's own set of parameter files as well as launch files so that they can be fully independent from each other.

Each package has it's own nodes. The various nodes include:

* Joystick teleop control node
* Safety Controller Node
* Wall Follower controller node

<span class="image main">![](assets/images/lab3/ROS_Architecture.png)</span>

The priority levels of the controllers read highest to lowest from right to left. Joystick input overrides the other two controllers, while safety supersedes the wall follower. It is important that the safety controller has precedence over autonomous code in order to avoid collisions and prevent damage to the car. Wall follower output is sent to the safety controller for it to react appropriately. In the situation where the safety controller does not detect a treat for collision, the wall follower data will continue through the pipeline with priority. Then if joystick input is not given, the wall follower data will be published and the race car will move accordingly.

There are also visualization marker topics that were added in order to visualize the data points that the calculations are based on for the wall follower or safety controller respectively. This is valuable when simulating and debugging the code. This is a tool that will be useful to implement moving forward as well.



## Experimental Evaluation - C. Gadson & J. Aleman

### Testing Procedure

Lab 2 showed us how effective testing with the headless simulator was. We decided to keep using it to allow each team member to do contributions even when they were away from the hardware. 
To do this we:

* Tested our wall follower code from lab 2 in real life to validate the simulator
* Used the simulator for developing and testing the basic functionality of:
  * Improved wall follower
  * Safety controller
* Achieved basic functionality on simulator
* Started testing with the racecar

#### Simulation testing

To be able to test different scenarios under simulation, we used three different maps.
* Building 31
* Basement of building 32
* Custom map created just for our tests

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/1yYYhOgdqSM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</center>

#### Race Car testing

The final result of this project includes a successful implementation of the wall follower and safety controller as described previously.  The safety controller and the wall follower compliment one another very well, as the safety controller does a good job of stopping the wall follower when it gets a little overzealous, but does not completely inhibit the wall follower’s movement after stopping.  The best way to depict this is through videos of the race car driving based on the wall follower code, stopping according to the safety controller, and both implementations working in unison.

### Results

The final result of this project includes a successful implementation of the wall follower and safety controller as described previously. The best way to depict this is through videos of the race car driving based on the wall follower code, stopping according to the safety controller, and both implementations working in unison.

Safety Controller and Wall follower running at the same time:

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/VfPTppgZUJY" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</center>

Safety Controller:

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/ihs1-eWknoM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</center>

Wall Follower running as well as visualization:

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/OvRYLPi5LRM" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</center>

## Lessons Learned - A. Hrabchak, M. Picchini

The start of this lab consisted of a long discussion about our objectives and motivations across multiple days. This approach was successful, however, the team would have benefitted from a more systematic approach from the beginning. In the future, our plan of action will be more structured. We learned that we must establish our overall objectives and discuss our proposed technical approach before assigning duties and plunging into the code. 

Creating a physical outline and task list proved to be quite effective throughout the middle of the week as the lab progressed.  In terms of communicating and collaborating with one another,  we tried various different methods that appeared to be successful to different degrees:
* We used google docs in order to organize our ideas and put together our report and website.  This method was effective, It allowed the whole team to see the status of work, and to make changes easily.
* Slack was chosen as our main means of communication.  This helped in setting up meeting times and addressing problems remotely, but fell short in delivering information in a timely manner as not all of the team members are used to checking slack frequently.  For future labs, when a response is needed faster, texting might be the be the better alternative.
* While coding and working on the car, we used tmux sessions in order to work concurrently and be unified in our actions.  This allowed everyone involved to easily peak in on what others are doing, and collaborate quickly when necessary. Overall, we have established a general means of planning, communication, and collaboration that will guide us for future labs.


### Technical Conclusions - H. Nichols

The technical and collaborative aspects of this lab were very dependent on each other. Even though we split up the work, it was important for all of us to communicate effectively so that every team member understood the technical implementation of the algorithms. This collaboration made it easier to debug and converge on the ideal solution.

In developing the wall following code, we processed and filtered the LIDAR data to the desired range of 15-90 degrees, as we didn’t feel it was necessary for performance to include a larger range. We ran into issues with the code if there was no wall in this window so we instructed the car to turn and keep scanning for a wall if this happens. This ensures our car will keep turning until a wall is found. Granted if the car is in open space it will travel in circles, but we didn’t feel this was a case we would run into. We have not tested this end case as much as we would like to but plan to continue to improve this aspect. Otherwise, the code performed to expectations and did not have any bugs that we discovered. We did attempt to implement a more standard type of PID controller by using the change in error, rather than the angle, but we found that the angle implementation worked better. We also began adding an integral term in the code. We plan to use and tune this parameter for future labs if necessary. In improving the wall following code, we noticed the racecar turns corners more cleanly now than it did when we began the lab.
 
We calculated the stopping distance for the racecar along any line or arc. The car was given some buffering room so that if we needed to stop, the car would stop right before the obstacle instead of crashing into it as we stop. We additionally filtered through the data coming through the crash window so that the car wouldn’t be too sensitive and stop to noise. Our implementation of the safety controller is robust for objects in its direct field of view but is not best suited for when the car is turning around a corner as the field of view is blocked. The car may not be able to stop if there is an obstacle directly behind the corner. It would be interesting to further implement the algorithm to swerve around these obstacles instead of outright stopping every time. In addition, we also struggled to tune the stopping algorithm of the car. The car moves forward, the safety box extends out and detects an object, so it comes to a stop. At this time, a new safety box is created while the car is stopped so the car proceeds to move forward. However, the car quickly realizes it will hit the detected object but it can’t stop before it hits wall. We plan to continue to improve this algorithm so this doesn’t happen.
 
In developing our algorithms, we found that it was additionally helpful to implement markers to visualize the safety controller. This was helpful for debugging and testing.
 
We evaluated the process utilization load at 30% and after additionally running our nodes, the load slightly increased to 33%. Our nodes are small in comparison so we decided to not focus on optimization yet.

This lab was successful in terms of individual and team development of skills. Some team members learned how to code better, whereas others became more proficient in ROS. In addition, we all spoke about how to organize our code better in terms of structure and comments so that any team member is able to follow along.


### CI Conclusions

1. Allie - At our first team meeting, each member shared their individual motivations and goals for this semester. This discussion proved to be very useful for the division of tasks and reinforced that while we may have come from different backgrounds, we are all here for similar reasons. We also reviewed each other schedules in order to determine potential meetings time outside of the allotted lab blocks. We were slow to create a concrete plan about our approach for the lab which resulted in finishing rather close to the deadline. We do not anticipate this being a problem in the future as now we know to create our task list and determine our approach as an essential first step. 

2. Carissa - This first lab together as a team started off a bit rocky, but we were able to organize ourselves and learn from our initial mistakes. At the beginning of this lab, I was somewhat hesitant about how to get started as a team; we were all going at our own pace through the beginning of the lab in an unstructured manner. The next time we met, we quickly realized that we needed to break up the work in a reasonable way and get going in order to meet our deadline. One thing I improved at during the week is speaking up when I think we’re not on the same page or when I’m confused about the direction we’re going in. I’m looking forward to our next project together now that we have agreed on an organized procedure for approaching and completing labs.

3. John - Getting the team organized on the first day was challenging, We tried to divide the work and jump into technical work form day one but it took a few days to get to know each member and how the like to contribute. In retrospect, I would love to have started with getting to know the team better before we started assigning tasks. Everybody’s strengths were very different even though our objectives were very similar. Everyone wants to program a lot and learn as much of as possible. Once we knew how each member likes to contribute the efficiency of the team improved and we were able to get a lot more done.

## Future Work - A. Hrabchak

Our wall follower algorithm works successfully on the racecar, however we still feel we can keep on improving it. We plan to add and tune an integral term into our PD controller. The implementation of an integral term will reduce steady state error. We didn’t experience steady state issues during this lab but we believe it will become more relevant in the future.

In case of the safety controller currently decelerates and stops the car when it predicts a crash into an obstacle. This is an adequate approach for ensuring the car’s safety but is not a permanent solution. In the future we would like for the car to not only stop in the face of danger but navigate and find a solution route to avoid the obstacle.

