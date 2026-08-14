### Hello Robot Stretch3 Demo for Autonomous Operation in the Microfabrication Cleanroom

#### High Level Overview
The aim of this project is to present a minimum viable demo that showcases the Stretch 3's autonomous operation in the microfabrication cleanroom as a basis for proposing further research into the use of autonomous mobile manipulators in a microfabrication cleanroom environment.

The current scope of the project is producing a video that shows a complete and successful iteration of the wafer etching process using the Oxford RIE80 etcher. A complete and successful single iteration comprises the Stretch3 being able to open/close the RIE80, navigate between stations, and deposit/withdraw a wafer from a station and the machine. In theory, by showing a single, somewhat smart/autonomous approach to the problem, we have a basis to show that such operation could be extended to as many iterations as needed and for completely autonomous operation.

#### Current Approach
We are currently working on the demo in a relatively fixed location and setup, i.e. the MCB cleanroom RIE80 area, and as such our approach to the problem is combining a series of fixed, coarse grained actions with fine grained error correction loops between fixed operation steps.

For interaction with the RIE80 and the wafers to be etched, we have attached a vacuum pen to the gripper using a CAD printed tool. While this helps solve the issue of being able to deposit and withdraw wafers, it means our gripper camera view is obstructed and is something to keep in mind (elaborated upon later).

For example, since we know roughly how far the station is from the machine, we can move the robot roughly some distance (using the existing Hello Robot Stretch3 body library) from the machine to the station and then correct for orientation and distance offshoots (e.g. drift) after the fact.

As it stands, our current approach for error correction boils down to the use of ArUco tags at key locations that can be used to localise the robot and correct any errors before performing the necessary next steps in the iteration. I.e. instead of implementing some form of continuous error correction and trajectory calculation throughout the process, we have opted for a means of fixing the robot's position and setting it up to carry out a predefined task at each step in the process; as long as the robot starts in the same position and orientation before starting a subtask, the actions performed in the subtask will be the same (e.g. extend arm out by X meters, down by Y meters, etc.). Though somewhat of a naive approach, for the purposes of a demo and given the nature of the Stretch3 body library, we feel that this is a good enough approach for creating a full demo that showcases a level of autonomy and physical capability that can then be improved upon and optimised when moving towards actual day-to-day use development.

Information about the names and dimensions of the ArUco tags can be found in `autonomous_demo/aruco_marker_info.yaml`.

The Stretch3 is equipped with two cameras, a head mounted D435 camera and a gripper mounted D405 camera, that we use to perform positional error correction and manipulation tasks-- the former is used for movement to and alignment with the RIE80 and wafer station, while the latter is used for fine grained manipulation tasks like twisting the dial and pressing the button on the RIE80 to operate it.

#### Examples of Error Correction Using the Head and Gripper Mounted Cameras
Camera error correction is done using the existing camera and ArUco helpers found in the [Hello Robot Stretch3 Visual Servoing Library](https://github.com/hello-robot/stretch_visual_servoing), and most of the logic behind the gripper-mounted manipulation tasks is reusing and reworking the existing servoing logic.

Head Mounted D435 Camera
- Given an ArUco tag pasted in line with a station (e.g. wafer station) with some positional offset from the desired target (e.g. 25 cm in front of the tag = 5 cm in front of the station, specified using the tag ID and an offset config file), the head mounted cam can get the X, Y, Z error of the tag relative to its camera frame.
- Using this positional error vector alongside tag information such as its unit Z vector in camera frame coordinates, we are able to calculate the rotational error (needed to face the tag head on) and horizontal or depth error (depending on if moving to a station or aligning with a station) of the tag.

Gripper Mounted D405 Camera
- Given the obstruction of the center of the gripper camera's point of view by the vacuum pen, any fine grain manipulation that needs to be performed by the Stretch3 hinges on using bi-lateration with two ArUco tags on the left and right of the object to be manipulated.
- Similar to the head mounted camera, we use the positional errors calculated in the camera frame and adjust our position based on those values and the distance to our target; however, in the case of the gripper we take our target point to be the midpoint (or thereabouts depending on the operation, as it changes slightly due to tool offsets for different tasks) of the two tags.

#### The Important Scripts
There are currently 5 key scripts used in the demo: `main.py`, `base_alignment.py`, `twist_and_adjust.py`, `button_and_adjust.py`, and `station_navigation.py`.

## `main.py`
- `main.py` is the demo orchestration script that imports the other four scripts/subroutines as modules and sequences them into a complete demo script.
- The demo sequence takes place in `main()` and uses defined helper functions for movement, rotation, and manipulation
- Currently `main()` does the following:
    - try to instantiate the Stretch3 robot object from the `stretch_body` package
    - check if a debug flag has been set *in code* and if so run the `debug_test()` routine specified by the arguments and return after completion
    - if the debug flag is not set, then the main body while loop will be run and perform $i$ iterations of the demo (as specified *in code*)
    - **TODO: update the main script to take CLI arguments to determine whether to run in debug mode and how many iterations to run**
- `main.py` has the following helper functions:
    - `default_gripper(robot)`: homes/resets the gripper position
    - `move(robot, distance)`: moves the robot `distance` meters forward
    - `rotate(robot, angle)`: rotates the robot `angle` radians *counter-clockwise*
    - `wafer_action(robot, at_tray, put_down)`: robot performs a manipulation action based on whether it is at the RIE80 tray or the wafer station and whether it is a pick up or put down action *(default values are at_tray=`True`, put_down=`False`)*

## `base_alignment.py`
- `base_alignment.py` is the script that aligns the robot in-line with two specified ArUco tags using its own base mounted ArUco tags; in the demo it is primarily used to align the robot perpendicularly with the RIE80 using two floor ArUco tags #9 and #10.
- `run(robot)` points the head-mounted D435 camera down and starts an iterative alignment loop. Each iteration averages several marker detections, calculates the robot's position and angular error, and applies a limited rotation and forward or backward movement.
- Alignment succeeds when the position and angle errors fall within the configured thresholds, or stops after the maximum number of attempts.

## `twist_and_adjust.py`
- `twist_and_adjust.py` uses the gripper-mounted D405 camera to align the gripper with the RIE80 dial using the two ArUco tags on either side of it.
- Once the gripper is close enough, the script performs a fixed sequence of wrist rotations and arm movements to open or close the dial, as selected by the `op` argument passed to `run()`.
- The script also restores the robot to its starting pose after completing the operation.
- `twist_and_adjust.py` has the following main functions:
    - `recenter_robot(robot)`: moves the head, arm, lift, wrist, and gripper into the starting pose used for dial operation
    - `run(robot, exposure, op)`: performs visual alignment and then runs the fixed dial-opening or dial-closing sequence *(default values are exposure=`low`, op=`close`)*

## `button_and_adjust.py`
- `button_and_adjust.py` uses the gripper-mounted D405 camera to align the gripper with the RIE80 button using the surrounding ArUco tags and tags mounted on the gripper fingers.
- Once aligned, it extends the arm to press and hold the button before retracting the arm.
- `button_and_adjust.py` has the following main functions:
    - `recenter_robot(robot)`: moves the robot into the starting pose used for button alignment
    - `run(robot, exposure)`: performs the visual alignment, button press, hold, and retraction sequence *(default exposure=`low`)*

## `station_navigation.py`
- `station_navigation.py` uses the head-mounted D435 camera to locate a named station ArUco tag, such as `tray` or `wafer_station`, and correct the robot's position and orientation relative to it.
- The `horizontal_align` argument determines whether the robot approaches the station by correcting its forward distance or aligns laterally with it after turning to face the station.
- `station_navigation.py` has the following main functions:
    - `recenter_robot(robot)`: moves the head and manipulator out of the way and into the starting pose used for navigation
    - `run(robot, tag_name, exposure, horizontal_align)`: searches for `tag_name` and corrects the robot's orientation and position until the configured tolerances are met *(default values are exposure=`low`, horizontal_align=`True`)*

#### V1.0 Testing Build Trials and Results
Located [here](https://docs.google.com/spreadsheets/d/1fxswzhHGRKX3I80u3gj_Lc9I5HO02BGpnYSGhEmqL74/edit?usp=sharing)

#### Further Considerations and Future Changes

- Further considerations for the existing approach are as follows:
    - Implement some form of autonomous docking and charging so that the robot can start, wait, and end at the docking station (can eliminate the battery life issue if recipes are long enough on average)
    - Consider a leaner and more consolidated approach for the helper scripts-- there is a lot of overlap and redundancy in each script (it would be good to make each script leaner, but it also may be possible to reduce the number of total scripts (e.g. station, button, and dial might be combined into a single script))

- Future changes:
    - Increase the generalisability of the demo by either getting rid of hardcoded fine grain motion values OR implementing some generally configurable parameters.
        - the former is probably better given that the latter still implies some level of similarity between the environments the robot is being used in.
    - Refactor the entire demo and procedure to use ROS2 and leverage things like TF2 and Nav2, which would allow for easier transform management, navigation, and ultimately greater flexibility.
    - Work on and develop some general/standardised hardware interface which would allow for the separation of robot development and autonomous routine development.
        - i.e. given some standardised tool/hardware interface, in theory a developed autonomous routine should be able to extend to any general mobile manipulator, not just the stretch3, given that it can interface with the standard tools correctly.