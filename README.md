### Hello Robot Stretch3 Demo for Autonomous Operation in the Microfabrication Cleanroom

#### High Level Overview
The aim of this project is to present a minimum viable demo that showcases the Stretch 3's autonomous operation in the microfabrication cleanroom as a basis for proposing further research into the use of autonomous mobile manipulators in a microfabrication cleanroom environment.

The current scope of the project is producing a video that shows a complete and successful iteration of wafer etching process using the Oxford RIE80 etcher. A complete and successful single iteration comprises of the Stretch3 being able to open/close the RIE80, navigate between stations, and deposit/withdraw a wafer from a station and the machine. In theory, by showing a single, somewhat smart/autonomous approach to the problem, we have a basis to show that such operation could be extended to as many iterations as needed and for completely autonomous operation.

#### Current Approach
We are currently working on the demo in a relatively fixed location and setup, i.e. the MCB cleanroom RIE80 area, and as such our approach to the problem is combining a series of fixed, coarse grained actions with fine grained error correction loops between fixed operation steps.

For interaction with the RIE80 and the wafers to be etched, we have attached a vacuum pen to the gripper using a CAD printed tool. While this helps solve the issue of being able to deposit and withdraw wafers, it means our gripper camera view is obstructed and is something to keep in mind (elaborated upon later).

For example, since we know roughly how far the station is from the machine, we can move the robot roughly some distance (using the existing Hello Robot Stretch3 body library) from the machine to the station and then correct for orientation and distance offshoots (e.g. drift) after the fact.

As it stands, our current approach for error correction boils down to the use of Aruco tags at key locations that can be used to localise the robot and correct any errors before performing the necessary next steps in the iteration. I.e. instead of implementing some form of continuous error correction and trajectory calculation throughout the process, we have opted for a means of fixing the robot's position and setting it up to carry out a predefined task at each step in the process; as long as the robot starts in the same position and orientation before starting a subtask, the actions performed in the subtask will be the same (e.g. extend arm out by X meters, down by Y meters, etc.). Though somewhat of a naive approach, for the purposes of a demo and given the nature of the Stretch3 body library, we feel that this is a good enough approach for creating a full demo that showcases a level of autonomy and physical capability that can then be improved upon and optimised when moving towards actual day-to-day use development.

The Stretch3 is equipped with two cameras, a head mounted D435 camera and a gripper mounted D405 camera, that we use to perform positional error correction and manipulation tasks-- the former is used for movement to and alignment with the RIE80 and wafer station, while the latter is used for fine grained manipulation tasks like twisting the dial and pressing the button on the RIE80 to operate it.

#### Examples of Error Correction Using the Head and Gripper Mounted Cameras
Camera error correction is done using the existing camera and aruco helpers found in the [Hello Robot Stretch3 Visual Servoing Library](https://github.com/hello-robot/stretch_visual_servoing), and majority of the logic behind the gripper mounted manipulation tasks is reusing and reworking majority of the servoing logic.

Head Mounted D435 Camera
- Given an Aruco tag pasted in line with a station (e.g. wafer station) with some positional offset from the desired target (e.g. 25 cm in front of the tag = 5 cm in front of the station, specified using the tag ID and an offset config file), the head mounted cam can get the X, Y, Z error of the tag relative to its camera frame.
- Using this positional error vector alongside tag information such as it's unit Z vector in camera frame coordinates, we are able to calculate the rotational error (needed to face the tag head on) and horizontal or depth error (depending on if moving to a station or aligning with a station) of the tag.

Gripper Mounted D405 Camera
- Given the obstruction of the center of the gripper camera's point of view by the vacuum pen, any fine grain manipulation that needs to be performed by the Stretch3 hinges on using bi-lateration with two Aruco tags on the left and right of the object to be manipulated.
- Similar to the head mounted camera, we use the positional errors calculated in the camera frame and adjust our position based on those values and the distance to our target; however, in the case of the gripper we take our target point to be the midpoint (or thereabouts depending on the operation, as it changes slightly due to tool offsets for different tasks) of the two tags.

#### The Important Scripts
We are currently working on 5 scripts that govern the entire demo-- complete_demo.py, twist_and_adjust.py, button_and_adjust.py, base_alignment.py, and station_navigation.py

**will fill in this section later, need to clean up codebase**

#### Further Considerations and Future Changes