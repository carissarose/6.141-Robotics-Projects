Lab 5
=====

## Overview and Motivations - [R. CHANG] 

The goal of this lab was to allow the car to localize (determine its location) in the Stata Center basement. The algorithm we developed is called a particle filter, which uses LIDAR data and odometry data from the VESC to calculate the most likely location of the robot on a map. Localization is an important fundamental task in robotics, as higher levels of autonomy (i.e. planning) require knowledge of the location of the robot. For the race car, knowledge of its location in the hallways can allow us to further develop algorithms to drive around the hallways.

## Proposed Approach - [M. PICCHINI]

Our group chose to implement a particle filter, in order to determine the car’s orientation and position in its environment. The more accurate wheel odometry was chosen as the input to the motion model as opposed to the IMU, and the known tunnel map and laser scan data were inputs to the sensor model.  The particle filter was responsible for taking in the motion and sensor input and producing a best guess pose at which the car resides.  The particle filter was required to perform under real time operation to allow the car to run smoothly.  As a result, appropriate optimizations were made when programming to meet this benchmark.  In order to complete the project, the work was divided into appropriate categories for team members to complete.

### Initial Setup - [M. PICCHINI]

Our team chose to split up the project into the following categories in order to delegate the tasks among our team members appropriately: development, debugging/testing, and data collection.  This was necessary due to the nature of the assignment, as well as the availability of the team members. Since each category relied on the previous category’s completion, we got started on the project early in order to meet the deadline.    The development group would be responsible for a basic implementation of the particle filter – albeit not necessarily fast or accurate.  The debugging/testing group would be responsible for correcting errors in the code and tuning certain parameters to minimize the particle filter’s error when compared with the ground truth, as well as optimizing the code to perform during real time operation.  Finally, the data collection group would collect and analyze data from the particle filter to ensure that everything is working as expected and the results are up to par.
 
Unlike the previous project where some code was a black box to group members working on other parts of the project, this distribution of work seemed to be effective in allowing all group members to work with and understand the main algorithms at the core of the particle filter.  Each group is required to understand and modify the code to complete their task, which not only contributes to each group member learning, but also contributes to the quality of the final product since multiple people are looking over the same code.


### Technical Approach - [R. CHANG]

The idea behind the particle filter is to represent possible locations of the car as “particles”, or a set of poses associated with a probability weight of how likely it is for the car to be at that pose. It is an iterative algorithm that converges over time.

Each iteration consists of three main steps: (re)sampling the particles, updating/moving them according to a motion model, and assigning probabilities to each one according to a sensor model. The number of particles is fixed (let it equal _m_) and experimentally determined, and they are initialized in a particular way. There are two ways to initialize particles: they can be distributed evenly throughout the map (no prior information at all about the robot’s location) or all started at a known location (if the robot’s approximate starting location is known). Convergence is not guaranteed if the particles are distributed throughout the map, so we took the second approach for initialization. This required code to receive an initial pose input from rviz and set all the particles to that pose with equal weights (_1/m_).

#### Resampling the Particles

The first step is to resample the set of particles according to the previous probability weights. Particles are drawn (with replacement) _m_ times randomly from the current set of particles, according to their probabilities. Thus, higher weight particles are more likely to be drawn multiple times, and lower weight particles are likely to not be drawn at all, throwing away improbable robot poses. This new set of particles is used as the current set of particles for the next steps.

#### Motion Model

The poses of all the particles are then updated according to how the car moved in the last iteration, plus some noise to account for inaccuracies in the odometry data. Odometry data from the VESC is used to update the poses. The data comes as an odometry message with poses and twists (velocities) and we chose to use deltas in pose as it was more accurate. The returned pose is x, y, and orientation coordinates in some arbitrary coordinate space, so the deltas in x and y are first rotated by the orientation to get deltas in local robot space. Each particle, i, is then updated as follows:

$$x\_{i, t} = x\_{i, t-1} + \Delta x \cos(\theta\_{i, t-1}) + \Delta y \sin(\theta\_{i, t-1}) + \mathcal{N}(0, \sigma\_x^2)$$
$$y\_{i, t} = y\_{i, t-1} - \Delta x \sin(\theta\_{i, t-1}) + \Delta y \cos(\theta\_{i, t-1}) + \mathcal{N}(0, \sigma\_y^2)$$
$$\theta\_{i, t} = \theta\_{i, t-1} + \Delta\theta + \mathcal{N}(0, \sigma\_\theta^2)$$

The noisiness of the model, sigma, is experimentally determined. The effect of the motion model is that the particles disperse and diverge over time due to the noise.

#### Sensor Model

The likelihood of each particle is determined using a sensor model, which gives the conditional probability of the current laser scan given the pose of the particle. The closer the particle aligns with the true location of the robot, the more likely it is for the observed laser scan to match the “simulated” laser scan that would occur from the particle. Each laser scan consists of many measurements, so we downsample these (by an experimentally determined amount) and compute probabilities for each range individually and multiply them together. We precompute the sensor model (conditional probabilities) and store it into a table.

<left>
<p style="float: left; font-size: 9pt; text-align: center; width: 45%; margin-right: 1%;"><img src="assets/images/lab5/Figure_3.png" style="width: 100%">Figure 1:  Probability Distribution </p>
</left>
<left>
<p style="float: right; font-size: 9pt; text-align: center; width: 45%; margin-left: 1%;"><img src="assets/images/lab5/Figure_1.png" style="width: 100%">Figure 2: Precomputed Sensor Model</p>
</left>

<br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>



The formula consists of a few parts. The main part is a Gaussian distribution which peaks the likelihood of the observation to be the highest if it matches the “ground truth”, which is the simulated laser scan from a particle. A slight ramp is added to ranges less than the ground truth to account for possible obstacles in the way. A spike is added at the maximum LIDAR range to account for out-of-range scans. Finally, a small constant is added to the whole thing to account for other unlikely events. The whole table is normalized, and this is done for each possible ground truth value. The result is a 2 dimensional table.

During the algorithm, the simulated laser scans from each particle are computed using rangelibc, a fast raycasting library. For each particle, the simulated scan is compared to the observed scan and a probability is looked up using the precomputed table. This probability becomes the weight for that particle, which is used for the resampling step in the next iteration. This has the effect of removing particles that are far from the robot’s true location and making the particles converge to the robot’s true location. Finally, the expected value of the particles is returned as the inferred pose of the robot.



### ROS Implementation - [H. NICHOLS]

The ROS architecture in this lab is structured around the simulation.

For the simulation, the primary node /particle_filter collects from the lidar data and odometry data and publishes an odometry message with the inferred pose to /pf/pose/odom. This output message, along with the /vesc/odom data is compiled and analyzed in the /log node to create error plots. The ground truth estimate is extracted from the /vesc/odom topic and the inferred pose is collected from the /pf/pose/odom message. The /log node parses through these topics and writes to a CSV file to compare the ground truth to the estimated pose of the robot. 

<center>
<img src="assets/images/lab5/ros_lab5.png" alt="image goes here" style="width:80%;">

Figure 3: ROS architecture for particle filter
</center>

When running the particle filter on the racecar, the ROS architecture changes slightly as it will only subscribe to the odometry and laserscan data received by the racecar instead of the simulation and publish a similar odometry message of inferred pose.

## Experimental Evaluation - [A. HRABCHAK]

Experimental evaluation was done using an agile testing process. The initial objective was to be able to run our particle filter and obtain results, no matter how inaccurate. The remainder of the lab was devoted to tuning parameters and optimizing the filter to generate the best possible localization. 

### Testing Procedure - [C. GADSON]

The general testing procedure relied on the simulator to visualize the particles produced by the code. The algorithm was first debugged by each method written, then parameters were fine-tuned accordingly. 
Although many different methods are used in this particle filter, the general way each one was tested and debugged is the same. For each function, the team visualized the output and confirmed that the results were expected. For example, when testing the motion model, the simulator was used to check that the particles moved in an expected fashion of moving forward while also diverging based on the input from the controller and the set noise values. Once each method was tested and debugged in this way, the specific parameters could be determined.
Parameters were tuned iteratively by first making large changes to each variable then slowly converging on a final value. The success of the parameter changes was determined through the team’s simulation testing and the staff-provided test cases that give numerical results based on localization accuracy.


### Results - [A. HRABCHAK]

Our particle filter was able to localize our car with a maximum of about 0.65 meters of error. This maximum error occurs when the car is driving down a long hallway with not many features to detect. As the car turns corners and navigates through tighter conditions, the localization hovers between 0 and 0.3 meters of error. The trends of these results are expected as the error will be greater when there are less features in the cars surrounding area. These results are shown in the figure and video below. 

<center>
<img src="assets/images/lab5/particle_filter_00_error.png" alt="image goes here" style="width:60%;">

Figure 4: Particle Filter Error
</center>


<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/r10aZ1MS8gw" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 1: Particle Filter Error
</center>

The following video shows the car driving throughout the tunnels alongside the real time localization in RViz. Once again it is clear that the particle cloud disperses during the periods of time when there are not many features in the environment. When there are many features available, the accuracy of the localization is much greater. 

<center>
<iframe width="560" height="315" src="https://www.youtube.com/embed/R4ilJe3sg44" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

Video 2: Particle Filter Demonstration
</center>

During testing, we found that we needed to add an offset to the particle filter data collected as it is centered around the lidar sensor and not the car itself. This is shown in the image below. The axis on the left shows the particle filter data, and the axis on the right shows the true axis of the car. Each time that the particles are initialized, they are initialized with a slight offset as seen by this axis on the left. The offset is shown in the particle filter distance error graph at the start. The car is not moving at the beginning, and the particles are initialized with approximately 0.3 m of error. 


<left>
<p style="float: left; font-size: 9pt; text-align: center; width: 45%; margin-right: 1%;"><img src="assets/images/lab5/lidarerror.png" style="width: 100%">Figure 5:  Initial filtered mask with closing function</p>
</left>
<left>
<p style="float: right; font-size: 9pt; text-align: center; width: 45%; margin-left: 1%;"><img src="assets/images/lab5/graphwithlidarerror.png" style="width: 100%">Figure 6: Final filtered mask - the cone is expanded, but so is noise</p>
</left>

<br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>

We also found that the parameters that worked best with the autograder were different than the parameters that worked best in our simulation. Specifically, the x variance, y variance, and x mean worked better in our simulations when they were a bit smaller than the values submitted to the autograder. 

## Lessons Learned 

###CI/Collaborative Conclusions - [H. NICHOLS]

This lab was off to a slow start for our team due to multiple team members being bogged down by midterms and the spring flu. Although we were off to a slow start, we continued to stay in touch about our availability to meet. For this lab, it was helpful to receive the grading and feedback from the staff on past labs so that we know how to improve moving forward. Per Jane’s advice, we set up a Trello account and linked it with our Slack messaging system so that team members can see exactly which deliverables are due and when. We found the process of breaking down the tasks more useful than the physical separation between to do, doing, and done. In future labs, our team will continue to keep a running list of deliverables. We improved as a team this week in communicating about our external struggles and their impact on the progress of the lab, but given this information, we can continue to work on realistic timelines for ourselves.

### Technical Conclusions - [J. ALEMAN]

The concept of the particle filter localization was simple to understand but there were many pitfalls to solve along the way:
* The motion model was very sensitive so different noise parameters were necessary for every platform (racecar, simulator, autograder)
* The sensor model encountered numerical errors under certain cases.
    * Zero probability for a single laser range was to be avoided as the combined probability for a particle would multiply all values including 0.
    * Small probabilities would become 0 if not handled correctly too

The sensor model was the more robust of the two, but it was interesting to see how the car’s pose estimate would change depending on which term we relied on more. 
When we assumed a very bad motion model (making the noise really high) then the estimate was primarily driven by the sensor model. Viceversa, if the noise was really small (the motion model being perfect) then the pose estimate would be driven by the motion model alone. At the end, each model is good certain scenarios so we had to carefully tune both to make the estimate good at all times:

* The sensor model was good at estimating the position of the car when there were a lot of features to track on the lidar data.
* The motion model was good at tracking the car movement when in hallways or other areas where there weren’t many features to track


### CI Conclusions

1. Allie - This lab definitely started out slowly for the team as this time of the semester tends to be very busy for students. This resulted in our collaboration and communication dominating how efficient we were as a group in completing the necessary tasks. The trello board helped to breakdown the tasks and assign these tasks to team members. It was helpful to have a visualization of what people were currently working on and what people still had to accomplish. Everyone was up to date with each others progress at all times. Our communication was great during the week leading up to spring break, however the communication about wrapping up the report and presentation during and right after break was rather terrible. In the future, I think this will be less of a problem because we are back on a regular school schedule. 
2. John - This lab tested how well our team structure and organization worked under unexpected circumstances. Not only were each members busy due to midterms, but many got very sick at the same time. The result was a very slow and uncoordinated start for the lab. At the end we managed to get organized again but it took a lot of effort and sorting through confusion to get the lab done. This gives us some good pointers on how to improve for future labs. Better coordination from the beginning and more communication will be a priority from now on.
3. Carissa - In addition to the points already mentioned regarding struggles with communication, the structure of the lab made it a little more difficult than the labs so far. The lab challenged us to find our specific route we would take to complete the lab; there weren’t many clear-defined tasks and there wasn’t a given procedure to take. As a product of this and our lack of clear communication throughout, the way we worked on and finished this lab was fairly disorganized. Teammates worked on parts of the lab while we were together and just picked up where the last left off. Not parallelizing the work and not having the whole team on the same page the whole time prevented us from being as successful as we could’ve been with this lab.

## Future Work - [M. PICCHINI]

In addition to wall following, line following, cone detection, and parking, the car is now capable of localization.  These skills in which the car is acquiring are crucial to the skills needed for practical autonomous operation.  Combining these capabilities in the future will improve the robustness of racing the car effectively and safely.  Localization in particular adds the ability to be aware of our location in an environment so the car could work with more information when determining possible paths.  Our group will continue to use skills we have learned to work more effectively and efficiently on improving the autonomous driving capability of our car and the future additions to come.
