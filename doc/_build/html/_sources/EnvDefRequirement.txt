Input/Output
------------

Following are the requirements for input environment json file.

- resolution: Canvas dimension n. eg: n=500 --> means 0-500, 0-500 along x-axis and y-axis respectively.
- initial_state: x, y of the rear axle center at initial state. eg: [45, 45].
- initial_orientation: Orientation angle in degree at initial state. eg: 45.
- goal_state: x, y of the rear axle center at goal state. eg: [480, 50].
- goal_orientation : Orientation angle in degree at goal state. eg: 90.
- random_points: Number of random point for the algorithm. eg: 5000.
- theta_norm: [x, y, theta] normalization vector for the distance function. Usually x and y be given equal weight and a constant for theta term. eg: [1, 1, 5].
- tolerance: Allowed error tolerance for distance function. eg: 1.
- geom_skip_factor: Used for drawing curves. This number defines the number of lines used while drawing curves. eg: value 1 would yield a better looking curve than value 5. However at a cost of memory.
- collision_check_skip_factor: Frequency of collision detection while performing the integration. eg: value 1 would yield higher number of collision detection, while value 4 would skip at an interval of 4 while doing the integration. The former one would be a 4 times computational intensive than the later one.
- total_steps_for_complete_integration: Defines the h value in the 4th order Runge-Kutta integration. Higher value yield better accuracy in integration and consequently better overall accuracy.
- obstacles: Defines a list of obstacles objects as polygon. Following is the method to define obstacles in the environment.
    - shape: Has to be "polygon" for all obstacles
    - property: where the vertex data goes.
        - vertices: define the x, y coordinate as a list. eg: [[400, 80], [400, 0], [100, 0], [100, 80]]
- car: meta data of car robot
    - linear_velocity: Velocity at which the car goes linearly. eg: 1.
    - max_turning_angle: Maximum angle between the axis along the car length and the front wheel when it is turned at its maximum.
    - front_axle_back_axle_distance: Distance between the front axle and rear axle of the car.
    - dimension: Defines the car dimension when its position (center of rear axle) is super imposed on 0, 0 of euclidean plane. Provide the xy coordinates as a list similar to obstacles-->property-->vertices.
     
Sample definition for input file is as follows.  

.. include:: EnvRRTStarRS10.json
    :literal:
    
The output generated will have all the details similar to that of input json and additionally the tree details and path details will be embedded. Extra attributes are as follows.

- control: Control input
- tree_lines: Tree as a list of line segments
- path_lines: Final path as a list of line segments

Following is a sample output file while running the RRT* for 10 iterations (To limit number of lines in this display).
    
.. include:: EnvRRTStarRS10_10_graph.json
    :literal:  
