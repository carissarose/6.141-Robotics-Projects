Final Challenge
=====

## Overview and Motivations - [H. NICHOLS]

The goal of the urban mobility challenge is to autonomously plan and execute a parallel parking maneuver and to autonomously unpark and exit the parking lot. The algorithm must be able to execute this plan and maneuver while avoiding collisions within the allotted two minutes in order to be successful. We were motivated to choose the urban mobility challenge as our final project as we were interested in the application of core concepts that we had already learned in lecture and implemented in previous labs. We wanted to pursue a final project that challenged us to find an optimal solution with minimal space for error. This required us to look deeper into the material we had learned in lecture to come up with creative solutions to consistently plan, park and unpark the racecar.

## Proposed Approach

### Initial Setup - [R. CHANG]

When deciding on the overall approach to use, we attempted to find an approach that would be a good balance of new and old methods, so that we could learn new things while also being able to build upon our past work in the class. The goal of the challenge was to autonomously have the car maneuver into a parking space, and leave the parking space, within two minutes in the presence of obstacles in a map. This called for path planning and trajectory following. We settled on using modified path planner and follower algorithms to complete the task of parallel parking among obstacles. To do this, we would modify our code from the race to optimize for complex maneuvers in tight spaces.

### Technical Approach

#### Path Planner - [J. ALEMAN, R. CHANG]

The path planning algorithm is a modified RRT algorithm that uses Reeds-Shepp curves instead of straight lines. Reeds-Shepp curves are curves that are defined between a start and end pose, using combinations of only straight lines and fixed curvature curves, while allowing for driving in both directions; this exactly reflects the capabilities of an Ackermann drive car.

Our approach to path planning has four key insights that led to its success: bidirectional RRT with Reeds-Shepp curves, smart sampling, configuration space stacking for obstacle checking, and path pruning. Each of these insights are described below.

We used “The Open Motion Planning Library” (OMPL) [1] to generate the shortest-length Reeds-Shepp curves given two poses. This returns a list of discretized poses along the path. Our algorithm first tries to generate a path between the start and end points directly. If this path is obstacle free, it is returned as the path. Otherwise, it enters the RRT algorithm. The RRT algorithm was rewritten to be bidirectional, meaning that is generates two trees from the start and end poses. It samples a point, determines the closest points in both the start tree and goal tree, and connects it to the closest point (using Reeds-Shepp length as the metric) in each tree with a Reeds-Shepp curve if it is obstacle free. If a point is able to be connected to both trees at once, the two trees are joined into the final path.

<center>
<img src="assets/images/fc/rrtgeneration.gif" alt="image goes here" style="width:50%;">

Figure 1: Bidirectional RRT Generation
</center>

RRT with uniform sampling was not enough to find feasible paths for parking in some of the tightest spots, so we do a more intelligent form of sampling. Instead of randomly generating points to add to each end of the tree, each sample point has higher probability of being sampled around the start and goal points (gaussian distribution around the start and goal). This dramatically increased the probability of finding a trajectory as well as making the algorithm quicker to find it:

<center>
<img src="assets/images/fc/sigma.png" alt="image goes here" style="width:50%;">

Table 1: RRT Gaussian Sigma
</center>

<center>
<img src="assets/images/fc/pose_sampling.png" alt="image goes here" style="width:50%;">

Figure 2: Pose Sampling
</center>

Each time a point is sampled and a reed-shepp curve from the closest node to it is generated, we must ensure that it is obstacle free. To do this, we check each of the poses along the trajectory against the configuration space of the robot. For previous labs, the configuration space assumed the robot was a large circle and dilated the map assuming that configuration. For parking on tight spots, this proved to be too conservative. To solve this, we made a stack of configuration spaces, one for each possible angle the robot can be found at and used the true size of the robot. This allowed to check if each pose on a trajectory would be obstacle free considering both its position and orientation.

<center>
<img src="assets/images/fc/c_space.gif" alt="image goes here" style="width:50%;">

Figure 3: Configuration Space
</center>

Last, since RRT is random, it can often generate complex paths that are difficult to follow or inefficient. To solve this, we designed a pruning algorithm to clean up the path. It starts by taking the last pose on the trajectory and trying to connect it to the first. If it’s unsuccessful it takes the one previous to last, and if unsuccessful, the one before it, and so on, until a path between the start and a pose on the trajectory can be connected. Once such a pose is found, the pruning algorithm will try to connect it to the last pose on the trajectory again and start stepping backwards from it until a new pose is found and the process is repeated. The path is considered pruned when a feasible path between the a pose and the goal can be achieved.

<center>
<img src="assets/images/fc/pathpruning.gif" alt="image goes here" style="width:50%;">

Figure 4: Path Pruning
</center>

#### Path Follower - [H. NICHOLS]

The path following algorithm is a modified pure pursuit that allows the racecar to follow trajectories. The trajectory used for this path following algorithm is a discretized array of poses. Given an array of poses that represent the discretized path, the algorithm first finds an associated direction with each pose by checking whether the next path pose is in front or behind the current path pose. If the next path pose is in front, we assign the current path pose with forward direction, and if the next path pose is behind, we assign the current path pose with backwards direction.

<center>
<img src="assets/images/fc/pp_v4.jpg" alt="image goes here" style="width:50%;">

Figure 5: Associating Directions with Pose Array
</center>

To find the lookahead point, we find the index of the closest path pose to the car. We add this index to an arbitrary lookahead index – in our case it was six indices. This value was optimized through the testing phase. The algorithm then finds the necessary curvature to reach the lookahead point and commands the steering angle of the car. 

<center>
<img src="assets/images/fc/pp_v2.jpg" alt="image goes here" style="width:40%;">

Figure 6: Lookahead Point for Segments
</center>

We parse through all the directions associated with the poses and find the cusp poses - the regions in the path that contain a change in direction. The cusp poses are always marked with forward direction. At the cusp, a line is projected from the cusp pose and the lookahead point slides along this line.
 
<center>
<img src="assets/images/fc/pp_v3.jpg" alt="image goes here" style="width:30%;">

Figure 7: Lookahead Point for Cusp Poses
</center>

Once the car pose is defined to be in front of the cusp pose, the lookahead point will switch to the new segment. By knowing these cusp poses, we can define each segment to be regions of uninterrupted direction. The old segment is marked complete and the new segment can be treated as its own path.

<center>
<img src="assets/images/fc/pp_v1.jpg" alt="image goes here" style="width:30%;">
 
 Figure 8: Car Pose Crossing Cusp Pose
</center>


<center>
<img src="assets/images/fc/pp_v5.jpg" alt="image goes here" style="width:30%;">
 
 Figure 9: Transition of Lookahead Point to New Segment
</center>


To enable the car to move backwards we inverted the car controller and did all calculations and measurements as if it was driving forwards to avoid having to adjust for the car dynamics.  This proved to be non problematic in practice.

### ROS Implementation - [A. HRABCHAK]

<center>
<img src="assets/images/fc/final-ros-arch.jpg" alt="image goes here" style="width:50%;">

Figure 10: ROS Architecture
</center>

In order to complete the urban mobility challenge, a few nodes, represented in light blue in Figure 1., were modified. The new trajectory planner node now subscribes to two poses, one goal parked pose and one exit and end pose. The planner uses the car’s odometry from localization as the starting pose. It then publishes a trajectory as a PoseArray and changes the cars state to notify the trajectory follower that it can begin following a path. The trajectory follower subscribes to the trajectory topic as well as the localized odometry given by the particle filter. The particle filter was only slightly modified to tune it from the previous challenge. The trajectory follower then drives the car by publishing an Ackermann message. Once the car has successfully parked, it changes the state of the car again to notify the trajectory planner to generate a new path to exit the lot.

Something interesting to note is that the modified visualizations are on a separate node on purpose. This is so that we can choose to run them or not depending on the circumstance. In some cases, adding too much visualization slowed down our algorithms and sometimes visualization messages would make rosbags too large. Keeping them on separate notes gives us a lot of flexibility for debugging without impacting the performance of the code we care for.




## Experimental Evaluation 

### Testing Procedure - [A. HRABCHAK]

An iterative evaluation process was used to thoroughly and effectively test both the path planning and pure pursuit algorithms. Initially, the algorithms were tested in simulation using rviz before real world testing on the physical car. To begin, simple trajectories were created on the open basement map used in previous labs. Here, the car was tested on its ability to follow cusps in the trajectory and change from forward motion to backwards motion. Once these basic paths were consistently successfully completed, the car was then given trajectories that parallel parked it in unusually large parking spots to allow for small errors. At this point, the car had to follow multiple cusps in succession. More complicated maps were used to test the path planners ability to create trajectories in which many obstacles were present and parking spaces were smaller. See examples below.

<left>
<p style="float: left; font-size: 9pt; text-align: center; width: 45%; margin-right: 1%;"><img src="assets/images/fc/modmap1.png" style="width: 110%">Figure 11: Map Example 1</p>
</left>
<left>
<p style="float: right; font-size: 9pt; text-align: center; width: 45%; margin-left: 1%;"><img src="assets/images/fc/modmap2.png" style="width: 110%">Figure 12: Map Example 2</p>
</left>

<br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>

With each new map and smaller parking spot, the path planner and trajectory follower were tested in simulation before tuning on the actual car. The majority of real world testing took place on the same general map that was used during the final challenge but with obstacles in different locations. 


### Results - [A. HRABCHAK]

The RRT algorithm and pure pursuit used for this challenge were generally successful. The path planner is able to find a path and optimize that path to the desired 0.9 m parking spot in 11.7 seconds on average. The consistency of the path planner was the greatest obstacle. Because of RRT’s random searching nature, the time to find a trajectory varied greatly, from 0.011 seconds to an indefinite amount of time. This ultimately caused the majority of issues on demonstration day. The pure pursuit algorithm was fairly successful and accurate when following the planned trajectory. Error in car position and orientation relative to the trajectory was minimized as best possible, as seen the graphs below. There are a few execeptions where the team believes the error was recorded to be greater due to inaccuracies in the localization algorithm.

<center>
<img src="assets/images/fc/groundtruthhurts.png" alt="image goes here" style="width:50%;">

Figure 13: Car Path (red) vs. Planned Trajectory (blue)
</center>
<center>
<img src="assets/images/fc/carpatherror.png" alt="image goes here" style="width:50%;">

Figure 14: Car Distance Error
</center>
<center>
<img src="assets/images/fc/orientation_error.png" alt="image goes here" style="width:50%;">

Figure 15: Car Orientation Error
</center>

Localization was extremely important in this challenge. The path planner algorithm only added a small buffer in the car dimensions in order to optimize running time and allow the car to reach small parking locations. A localization error of even a few centimeters would strongly affect whether or not the car hit an obstacle. The particle filter used previously was modified because of the small area and low speeds that the car experienced in this challenge. Even with these modifications, the localization still is not perfect and contributes to the cars deviation from the provided trajectory.

During testing, the total time to plan, park, and unpark was 44 seconds on average. On demonstration day, a similar map was given and the car completed the course in exactly 2 minutes. This result was uncharacteristically long and can be attributed to less points searched by the path planning algorithm in the region of the car caused by a misunderstanding between TA input and the code.


## Lessons Learned 

### Technical Conclusions - [M. PICCHINI]

In this lab, our team learned how general algorithms such as RRT and pure pursuit, could be tailored and optimized to perform well on more specific tasks such as parallel parking.  Rather than struggle to develop completely novel ideas for certain problems, most problems can be broken down into basic steps that can be solved with slight modifications to currently well developed solutions that are readily available.  In the case of parallel parking, our team noticed that both RRT and pure pursuit could be modified in such a way.  For RRT, the original straight line, forward path planning with completely random sampling that we used for racing, was modified to instead use bidirectional Reeds-Shepp curves and smart point sampling.  The bidirectional Reeds-Shepp curves allowed for generated paths that were actually possible for the car to follow and generated paths that allowed the car to maneuver into tight spots - which is crucial and desired for parallel parking, but not so much for racing.  The smart sampling was tailored to finding paths out of tight spaces which is important to parallel parking, as opposed to over long distances which is important to racing.  For pure pursuit, we learned that the algorithm could be adapted to handle directional changes, as well as near perfect path following.  

During the lab, our team encountered many bumps in the road.  The car not following the path, extremely complicated path generations, and slow path generation runtime were some of the more notable problems that arose.  However, with a lot testing and redesigning, these problems were able to be overcome.  Particle filter, pure pursuit, and speed tuning for the specific environment solved the problem of the car deviating from the path.  Smarter pruning allowed for complicated paths to become more simple, even when the whole path could not successfully be pruned. Slow path generation runtime was first improved by smarter sampling which was then followed by a low resolution path finder that found paths quickly, aiding the high resolution path finder.

Overall, our team improved the capability of the car to not only find and follow paths that only considered forward movements and allowed for large error, but also tight paths that required bidirectional movements. Our team also improved on understanding how algorithms can be adapted and implemented to perform specific tasks that they were not specifically designed for. 

### CI Conclusions - [M. PICCHINI]

The collaboration of our team these past few weeks has allowed us to complete the lab in a timely and efficient manner.   By planning ahead and giving a lot of buffer room on what our group needed to accomplish with a Gantt chart, tasks were able to be distributed and organized effectively between the members of the team.  Multiple tasks were able to be performed concurrently, even though they relied on one another.  This was able to be accomplished because the dependencies and expectations were decided at the start.  Each task was also given to no less than two group members, providing multiple views on a current task.  This planning of tasks early and having multiple members on each task allowed for the technical aspects to be of higher quality and easy integration into the current system.   More focus was able to be given on the actual implementation of the lab rather than the logistics behind it, resulting in a lot more getting accomplished than in our previous lab experiences.

# References
[1] The Open Motion Library https://ompl.kavrakilab.org/

