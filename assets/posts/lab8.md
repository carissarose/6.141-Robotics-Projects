Lab 6
=====

## Overview and Motivations - [C. GADSON]

The goals of this lab include planning trajectories in a grid map from specified start and end poses, follow a predefined trajectory with the racecar, and demonstrate real-time path planning and execution. The particle filter localization implementation from the last lab used in conjunction with a pure pursuit control and path planner in this lab are necessary for this assignment. This project is valuable because it will be needed for the timed race challenge that the class will have in a week. By having a successful implementation this week, the team will be able to more easily make improvements to perform well in the race.

## Proposed Approach - [A. HRABCHAK]

In order to accomplish the objective of planning a path for the car, the team decided to implement the Rapidly-exploring Random Trees* (RRT*)  algorithm and use a piecewise representation of the trajectory. The RRT* algorithm randomly draws a space-filling tree in the available search area and chooses the most efficient path as the final trajectory. 

The trajectory tracker then takes in this piecewise, generates a lookahead point, and then follows this point using PD control.  

### Initial Setup - [A. HRABCHAK]

The team initially split up the lab into the following main tasks to be accomplished: implement the path planning algorithm, implement the trajectory tracker, and modify the safety controller to ensure more reliability. All of these tasks had the ability to be developed independently of one another given that enough time was left at the end to integrate and test the entire system as necessary. Therefore, the majority of the team was able to work in parallel in order to complete all the required tasks. 

There are several different ways to approach the implementation of a path planning algorithm and a trajectory planning algorithm. The team made its decision to use the sampling-based algorithm RRT* for path planning it had a significantly faster runtime than the other approaches. The other initial decision that needed to be made in order to ensure consistency between algorithms was how to represent the trajectory. The team ultimately chose to use a piecewise in order to define the path due to simplicity.

### Technical Approach - [J. ALEMAN, R. CHANG]

Pure Pursuit Controller
The pure pursuit controller allows the racecar to follow trajectories. For our case, we use piecewise trajectories as they have the simplest representation method.
At a given time, the racecar doesn’t know where in the trajectory it is. It controls it’s steering based on how far to either side a lookahead point that lies on the trajectory is.

To calculate the lookahead point, first, we find the closest line segment on the piecewise trajectory. Then, we find where this line segment intersects with a lookahead radius r. This is the lookahead point.

There are some edgecases to handle:
* If the line segment end point is within the lookahead radius, the next line segment is used.
* If the radius is shorter than the minimum distance to the trajectory, The lookahead point is a vector perpendicular to the trajectory at distance r
* The equation for a line intersecting a circle has two solutions. 

<center>
<img src="assets/images/lab6/lookahead-diagram.jpg" alt="image goes here" style="width:80%;">

Figure 1: Lookahead diagram to calculate curvature
</center>

Once the lookahead point is found, we can calculate what is the curvature that would allow the racecar to hit that point when it turns its front wheels and set the steering command by that amount. [1]

Particle Filter Optimizations
Our particle filter from the previous lab, allowed us to estimate the position of the car reliably on hallways with lots of features but not so well on long hallways. We had to improve the localization node in order to use the pure pursuit controller propely.
We achieved this doing two things:
* Increased the noise on the motion model
* Squashed the sensor model a bit more
This allowed the particle filter to rely more on the motion model for the featureless hallways but it would still have trouble at certain points if plenty of new obstacles were found.

We turned to SLAM for this. Using google’s cartographer we generated a new map that included some of the new obstacles and features we can find today in the basement of building 32:

<center>
<img src="assets/images/lab6/basement_fixed.png" alt="image goes here" style="width:80%;">

Figure 2: Updated map to include new obstacles
</center>

Planning - RRT*
To find a trajectory from the robot’s current location (from the particle filter) to a goal point through the hallways, we used a path planning algorithm called RRT* (rapidly-exploring random tree). RRT* is a sampling-based algorithm that randomly builds a tree from the start point, avoiding obstacles, until the goal point is reached.

The basic idea of RRT* is as follows. Starting with just the start point in the tree as the root node, a point is randomly sampled in the entire map, and the nearest existing node in the tree is found. A new node is added to the tree at a fixed distance from that node, in the direction of the randomly sampled point, if the path is collision-free. This process is repeated until a node close to the goal is added, and then the path is found by tracing back the tree. This basic algorithm is known as RRT.

<center>
<img src="assets/images/lab6/basement_fixed.png" alt="image goes here" style="width:80%;">

Figure 3: ROS architecture for particle filter
</center>

RRT* expands upon this algorithm to optimize the path for length. Every time a new node is created, instead of connecting it to the nearest node, other nearby nodes are considered, and it is connected to the one that would create the lowest cost to reach that new node. In addition, the tree is “rewired” by once again considering nodes in a neighborhood around the new node. If going through the new node to the neighboring node is lower cost than the original cost of reaching the neighboring node, the neighboring node is disconnected from its original parent and the new node becomes its parent.

To check for obstacles between two points, we realized that similar functionality had already been implemented in rangelibc used in the last lab. We used rangelibc to raycast from one of the points towards the other point. If the raycast distance is less than the distance between the points, there is an obstacle.

To avoid paths going too close to the walls, we also applied morphological dilation to the map to expand the walls and force the path closer to the middle of the hallways.

### ROS Implementation - [R. CHANG]

<center>
<img src="assets/images/lab6/ROS-architecture.jpg" alt="image goes here" style="width:80%;">

Figure 4: RRT Diagram
</center>

The new code for this lab is contained in two ROS nodes, the path planner (/path_planner) and path follower (/pure_pursuit). Both are subscribed to the localization odometry topic from last lab’s particle filter node. The /path_planner node receives the robot location data, and waits for a clicked point message from rviz which specifies the goal point. It plans a trajectory as a polygon (list of points) and publishes it for /pure_pursuit. It uses the robot localization to make the robot follow the trajectory, and drives the car by publishing to the VESC.

## Experimental Evaluation - [H. NICHOLS]

Experimental evaluation was performed using an iterative testing process. The initial objective in our evaluation of the success of our pure pursuit algorithm was to have the car effectively use a lookahead point to follow a generated path. This process involved tuning parameters to reduce oscillation in following the trajectory. Our next success checkpoint was the effective implementation of the path planner. This testing process involved tuning of parameters and the search algorithms to ensure the planned trajectory is permissible. The remainder of the lab was dedicated to improving the particle filter from the past lab to optimize the car’s ability to locate itself in the map.

### Testing Procedure - [H. NICHOLS]

The evaluation procedure involved both iterative testing within the rviz simulator to initially test the workingness of the algorithm and on the physical racecar to tune the parameters further. 

The majority of the preliminary testing relied on the simulator. To test the pure pursuit algorithm, the only parameter that required tuning was the lookahead radius. This value was deemed optimized in the simulator once the car accurately followed an arbitrarily generated path, minimizing the average distance from the desired path.  The testing of the path planner algorithm also relied heavily on rviz. The evaluation of success of the RRT* algorithm was measured in three capacities: one being that the generated path stayed within the confines of the hallway, two that the vehicle started and ended at the selected points, and three being the time it took to generate the path. The preliminary experimental evaluations were performed in the simulator to reduce the chance for risk of collision on the racecar. 

After the lookahead distance for the pure pursuit and these two criteria for the path planner were met, the latter portion of the lab relied on visualization tools supplied by the racecar run in real time. In real time, the lookahead distance was sensitive to the speed of the car, so iterative testing was completed to find the optimal value for the parameter given a certain speed. This value was found through trial and error. In addition, there were issues with the map as the path planner would generate paths through obstacles that were in the tunnels but not in the simulated map. This information prompted us to overlay the original map supplied with a SLAM-generated map to get a more accurate map for the racecar to plan its trajectories in. With this new map, parameters were tuned in the particle filter so that the car could effectively plan and generate a path while identifying and avoiding real-life obstacles.


### Results - [M. PICCHINI]

The RRT* and pure pursuit algorithms generally worked well.  RRT* finds a path most of the time, but the time it takes does vary considerably.  However, this result is expected with the RRT* algorithm since it is non-deterministic and by chance, the algorithm can fail at finding a path, or also take a very long time at finding a path.  In the following table, we can see how long the algorithm took to find a trajectory based on the platform. The long tests go from one end of the basement to the other and the short tests just around a single corner, as seen in the picture below.

<center>
<img src="assets/images/lab6/table.png" alt="image goes here" style="width:80%;">

Figure 5: Different runtimes based on platform
</center>

<center>
<img src="assets/images/lab6/rrt.png" alt="image goes here" style="width:80%;">

Figure 6: Depiction of RRT in simulation
</center>

The long tests can vary by up to 30%, which is a significant amount variance.  

The pure pursuit results for following a generated path was very consistent.  The following plot shows the results of the pure pursuit algorithm following a path in the tunnels.

<center>
<iframe width="560" height="315" src="https://www.youtube.com/watch?v=1GuazayyhlU" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 1: Path Planning and Trajectory Tracking Demonstration
</center>

<left>
<p style="float: left; font-size: 9pt; text-align: center; width: 45%; margin-right: 1%;"><img src="assets/images/lab6/error.png" style="width: 100%">Figure 7</p>
</left>
<left>
<p style="float: right; font-size: 9pt; text-align: center; width: 45%; margin-left: 1%;"><img src="assets/images/lab6/trajectory-to-follow.png" style="width: 100%">Figure 8: Depiction of trajectory</p>
</left>

<br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>

Overall, the pure pursuit algorithm has a very low average error over the whole path.  On straight lines, the algorithm does exceptionally well, converging quickly, and staying on the path consistently.  When the car approaches a turn that is too tight for its steering, it does struggle to minimize the overall loss that could otherwise be achieved by turning earlier or some other strategy.  However, for turns that the car is able to actually make, there is very little loss.  The error on tight turns should not necessarily be seen as the pure pursuit’s fault, but more so the path’s fault.  The ideal path should not tell the car to turn tighter than possible, as the car does have its physical limitations.


## Lessons Learned - [A. HRABCHAK]

In this lab, the team kept a much better pace then with previous labs. The tasks were divided early, and lab time was productive in collaboration and asking questions. One helpful ritual that proved its use once again in this lab was the sketching of the expected ROS architecture as a group before splitting apart. This not only helped team members understand the objectives of the lab, but it also helped show the eventual system integration as a whole. The team also made more use of the established trello board during this lab. The trello board proved to be useful for some team members while others did not seem to find it as useful and were less active in updating the status of their tasks. Moving forward, it is important that if the team is going to benefit from using a tool such as trello, all team members must commit equally to ensure that the board is always accurate. 

### Technical Conclusions - [M. PICCHINI]

The concepts of path planning and trajectory tracking were pretty straight forward.  There were hardly any major conflicts when implementing the pure pursuit and RRT* algorithms.  When testing on the simulator, with the exception of typical small bug fixes, the code ran as expected and very smoothly.  The RRT* algorithm was able to generate a decent path in the tunnels, and the pure pursuit algorithm was able to follow this path.

While everything went smoothly on the theoretical side, problems started to arise when testing in a real environment.  The main problem arose when testing our pure pursuit algorithm in the real environment .  The pure pursuit algorithm relies on the particle filter outputting a correct pose of the car.  However, the particle filter was having problems localizing in certain parts of the basement where new objects that lined the walls that were not accounted for in the standard map we used.  In order to solve this problem, we attempted to recreate the entire map with SLAM.  Unfortunately, we could not get the map to converge completely, so we then attempted to modify the standard map with photoshop to account for the new objects. To accomplish this modification, we used the lidar scanned data of the new basement obstacles and superimposed them with the standard map.  However, this also proved to be unsuccessful, as the objects were very complex and difficult to model accurately. We then tried tuning our particle filter, introducing more noise and relying more on the motion model, but again, we could not conquer the altered hallway.  While debugging, we also found that the suspension of the car was affecting the lidar scan, angling the scan down more than it should have and reading the floor at certain angles.  To fix this, we physically adjusted the suspension.  

Ultimately, we were not able to maneuver this modified hallway very well.  However, in the process, the particle filter became more robust in other areas of the tunnels.  Also, in the areas that the particle filter was able to output the correct results, the pure pursuit was able to correctly follow the planned path.

This lab taught us that simulations are very helpful in quickly testing and developing, the ideal environment may not carry very well over to the real environment.  While it is important to have an implementation working in an ideal environment, it is just as important to test in real life and have a system robust to the real environment.  However, through all of these problems, the team learned how to follow and generate a path given start and end points and a map of the known surroundings, ultimately adding another crucial aspect to the ability of the autonomous car.

### CI Conclusions

1. Hailey - Our collaboration and communication in this week’s lab was better than last week’s. We spent close to an entire lab period walking through the ROS architecture of how all the nodes and topics piece together. This helped us explicitly define how each working element of this lab fits in with another piece. In addition, our communication via our slack group was much better this week as everyone was very present in communicating their status whether it was their availability to meet up or their progressions on technical aspects. This week we did a great job of completing our technical deliverables but in the future, I think it would be beneficial for our group to progressively work on the lab report and presentation throughout the week instead of saving it until the very end.

2. Michael - Compared to last lab where scheduling and responsiveness was almost non-existent due to spring break, this lab our communication and collaboration was very efficient.  Every group member was very responsive and contributive.  This resulted in assigning and completing parts of our lab relatively early.  The efficiency also allowed for more group members to understand more aspects of the lab and result in more contribution to making our path follower work better.  The work was better spread out over the group as well, with less of a rush on trying to finish the technical aspects.  We were also able to start the presentation earlier, but not as early as we would have liked.  This late start to the lab report can be addressed in the future by knowing when the results of our technical aspects is enough and we should move on to documentation. 

3. Raphael - This lab was completely much more smoothly than last lab, due to not having spring break or sick team members. Everyone was much more available so we spent a lot more time working together in person, which made communicating and dividing tasks easier. Everyone was able to work on a relatively independent part of the lab. However, since there were only two main parts of the lab (planning and following), this meant that only two team members worked significantly on the “new” parts, which may have been a problem for team learning. The increased efficiency allowed us to still get the lab working on time despite being more technically challenging. Despite this, we still pushed the lab report and presentation to the last two days, which is still not desirable.

## Future Work - [C. GADSON]

While we got the core functionality of the lab working there are several aspects to optimize on:
* Pure Pursuit algorithm currently uses a fixed speed. We would like to make it adapt the speed of the car and the lookahead radius based on the path curvature. This will help us for the race.
* Our particle filter still gets the car lost in certain cases. We need to squash the sensor model more so we can rely on the motion model to get it through hallways without running into particle depletion issues.
* RRT* algorithm takes too long. We will profile it to understand where it’s taking time and make it run faster.

References
[1] https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

