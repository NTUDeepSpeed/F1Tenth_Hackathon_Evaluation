#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np

# from gap_finder.pid import PID
from evaluation.pid import PID

# reference: https://github.com/f1tenth/f1tenth_labs_openrepo/blob/main/f1tenth_lab4/README.md
# reference: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html


class GapFinderAlgorithm:
    """
    This class implements the gap finder algorithm. The algorithm takes in a list of ranges from a LiDAR scan and
    returns a twist dictionary that will move the car to the deepest gap in the scan after drawing safety bubbles.
    params:
        - disparity_bubble_diameter: the diameter of the safety bubble
        - view_angle: the angle of the field of view of the LiDAR as a cone in front of the car
        - coeffiecient_of_friction: the coeffiecient of friction of the car used for speed calculation
        - disparity_threshold: the threshold for marking a disparity in the scan i.e. an edge
        - lookahead : the maximum distance to look ahead in the scan i.e ranges more than this are set to this value
        - speed_kp: the proportional gain for the speed PID controller
        - steering_kp: the proportional gain for the steering PID controller
        - wheel_base: the distance between the front and rear axles of the car
        - speed_max: the maximum speed of the car
        - visualise: a boolean to generate visualisation markers
    """

    def __init__(
        self,
        disparity_bubble_diameter=0.4,
        safety_bubble_diameter=0.4,
        front_bubble_diameter=0.33,
        minimum_bubble_diameter=0.33,
        view_angle=3.142,
        coeffiecient_of_friction=0.71,
        disparity_threshold=0.6,
        lookahead=3.0,
        speed_kp=1.0,
        steering_kp=1.2,
        wheel_base=0.324,
        speed_max=10.0,
        speed_min=1.0,
        speed_low_pass_filter=0.8,
        steering_low_pass_filter=0.8,
        visualise=False,
        bin_number=7,
    ):
        # Tunable Parameters
        self.disparity_bubble_diameter = disparity_bubble_diameter =0.28 [m]
        self.safety_bubble_diameter = safety_bubble_diameter = 0.8 # [m]
        self.front_bubble_diameter = front_bubble_diameter = 0.25
        self.minimum_bubble_diameter = minimum_bubble_diameter = 0.22
        self.view_angle = view_angle = 3.14  # [rad]
        self.coeffiecient_of_friction = coeffiecient_of_friction
        self.lookahead = 5.5 #[m]
        self.disparity_threshold = disparity_threshold = 0.5 # [m]
        self.wheel_base = wheel_base  # [m]
        self.speed_max = speed_max = 10.5 # [m/s]
        self.speed_min = speed_min = 1.2
        self.front_bubble_diameter = front_bubble_diameter
        self.bin_number = bin_number = 8
        self.speed_low_pass_filter = speed_low_pass_filter = 0.7
        self.steering_low_pass_filter = steering_low_pass_filter = 0.75
        # Controller Parameters
        self.speed_pid = PID(Kp=0.9,Ki=0.0, Kd = 0.0)
        self.speed_pid.set_point = self.speed_max
        self.steering_pid = PID(Kp=1.5, Ki = 0.0, Kd=0.12)
        self.steering_pid.set_point = 0.0
        
        # Feature Activation
        self.do_mark_sides = True
        self.do_min_filter = True
        self.do_limit_lookahead = True
        self.do_mark_disparity = True
        self.do_mark_minimum = True
        self.do_bin_speed = True
        self.do_speed_low_pass_filter = True
        self.do_steering_low_pass_filter = True
        # Visualisation
        self.visualise = visualise
        self.safety_markers = {"range": [0.0], "bearing": [0.0]}
        self.goal_marker = {"range": 0.0, "bearing": 0.0}
        # Internal Variables
        self.initialise = True
        self.center_priority_mask = None
        self.fov_bounds = None
        self.middle_index = None
        self.last_speed = 0.0
        self.last_steering = 0.0
        

    def update(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        angle_increment = scan_msg.angle_increment

        # --- Compute front clearance early for alignment ---
        middle_index = ranges.shape[0] // 2
        front_clearance = ranges[middle_index]
        if front_clearance == 0.0:
            front_clearance = np.min(ranges)  # fallback if LIDAR returns 0

        if self.initialise:
            # lookahead
            if self.lookahead is None:
                self.lookahead = scan_msg.range_max
            # middle index for front of car
            self.middle_index = ranges.shape[0] // 2
            # field of view bounds
            view_angle_count = self.view_angle // angle_increment
            lower_bound = int((ranges.shape[0] - view_angle_count) / 2)
            upper_bound = int(lower_bound + view_angle_count)
            self.fov_bounds = [lower_bound, upper_bound + 1]
             #center priority mask
            #anges_right = ranges[lower_bound : self.middle_index]
            #anges_left = ranges[self.middle_index : upper_bound + 1]
            #ask_right = np.linspace(0.95, 1.0, ranges_right.shape[0])
            #ask_left = np.linspace(1.0, 0.95, ranges_left.shape[0])
            #elf.center_priority_mask = np.concatenate((mask_right, mask_left))
            self.center_priority_mask = np.ones(1000)
            self.initialise = False
        # --- AUTO-YAW CORRECTION FOR INITIAL SPAWN ---
        #if not hasattr(self, "spawn_aligned"):
            #self.spawn_aligned = False
            #self.spawn_timer = 0.0

        #if not self.spawn_aligned:
        # Estimate heading based on left/right LiDAR average
            #left_mean = np.mean(ranges[:self.middle_index])
            #right_mean = np.mean(ranges[self.middle_index:])
    
        # Simple proportional steering to center the car
            #alignment_steering = 0.5 * (left_mean - right_mean)  # tweak 0.5 if needed
            #alignment_speed = min(self.speed_max * 0.5, front_clearance)  # slow while aligning
    
            # Mark as done if roughly centered
            #if abs(left_mean - right_mean) < 0.05:  # 5 cm tolerance
                #self.spawn_aligned = True
    
                # Return early with alignment command
            #self.last_speed = alignment_speed
            #self.last_steering = alignment_steering
            #return {"speed": alignment_speed, "steering": alignment_steering}

        ### LIMIT LOOKAHEAD ##
        if self.do_limit_lookahead:
            ranges[ranges > self.lookahead] = self.lookahead
            modified_ranges = ranges.copy()
        else:
            modified_ranges = ranges.copy()
            self.lookahead = scan_msg.range_max

        ### FIND FRONT CLEARANCE ###
        front_clearance = ranges[self.middle_index]  # single laser scan
        if front_clearance != 0.0:  # mean of safety bubble of front scan
            arc = angle_increment * ranges[self.middle_index]
            radius_count = int(self.front_bubble_diameter / arc / 2)
            front_clearance = np.mean(
                ranges[
                    self.middle_index - radius_count : self.middle_index + radius_count
                ]
            )

        ### MIN FILTER ###
        if self.do_min_filter:
            for i, r in enumerate(ranges):
                arc = angle_increment * r
                radius_count = int(self.minimum_bubble_diameter / arc / 2)
                if i < radius_count:
                    the_min = np.min(ranges[: i + radius_count + 1])
                elif i > ranges.shape[0] - radius_count:
                    the_min = np.min(ranges[i - radius_count :])
                else:
                    the_min = np.min(ranges[i - radius_count : i + radius_count + 1])
                modified_ranges[i] = the_min

        ### LIMIT FIELD OF VIEW ###
        limited_ranges = modified_ranges[self.fov_bounds[0] : self.fov_bounds[1]]
        limited_modified_ranges = modified_ranges[
            self.fov_bounds[0] : self.fov_bounds[1]
        ]

        marked_indexes = []
        ### MARK LARGE DISPARITY###
        if self.do_mark_disparity:
            for i in range(1, limited_ranges.shape[0]):
                if (
                    abs(limited_ranges[i] - limited_ranges[i - 1])
                    > self.disparity_threshold
                ):
                    if limited_ranges[i] < limited_ranges[i - 1]:
                        r = limited_ranges[i]
                        arc = angle_increment * r
                    else:
                        r = limited_ranges[i - 1]
                    arc = angle_increment * r
                    radius_count = int(self.disparity_bubble_diameter / arc / 2)
                    lower_bound = i - radius_count
                    upper_bound = i + radius_count + 1
                    marked_point = [lower_bound, upper_bound, r]
                    marked_indexes.append(marked_point)

        ### MARK MINIMUM ###
        if self.do_mark_minimum:
            r = np.min(limited_ranges)
            i = np.argmin(limited_ranges)
            arc = angle_increment * r
            radius_count = int(self.safety_bubble_diameter / arc / 2)
            lower_bound = i - radius_count
            upper_bound = i + radius_count + 1
            marked_point = [lower_bound, upper_bound, r]
            marked_indexes.append(marked_point)

        ### APPLY MARKS ###
        for mark_point in marked_indexes:
            limited_modified_ranges[mark_point[0] : mark_point[1]] = mark_point[2]

        ### PRIORITISE CENTER OF SCAN ###
        limited_modified_ranges *= self.center_priority_mask 
        # --- Dynamic lookahead and corner speed scaling ---
        min_range = np.min(limited_ranges)

        # scale down speed for tight corners
        if min_range < 1.8:          # narrow gap -> slow down
            speed_factor = 0.45       # aggressive slow
            self.lookahead = 2.5     # reduce lookahead to avoid oversteer
        elif min_range < 3.0:
            speed_factor = 0.75
            self.lookahead = 3.8
        else:
            speed_factor = 1.05       # straight -> full speed
            self.lookahead = 5.5

        # apply speed scaling
        init_speed = (front_clearance / self.lookahead) * self.speed_max * speed_factor
        speed = self.speed_pid.update(init_speed)
        ### FIND DEEPEST GAP ###
        max_gap_index = np.argmax(limited_modified_ranges)
        goal_bearing = angle_increment * (max_gap_index - limited_ranges.shape[0] // 2)

        ### FIND TWIST ###
        init_steering = np.arctan(
            goal_bearing * self.wheel_base
        )  # using ackermann steering model
        steering = self.steering_pid.update(init_steering)
        

        # init_speed = np.sqrt(10 * self.coeffiecient_of_friction * self.wheel_base / np.abs(max(np.tan(abs(steering)),1e-16)))
        # init_speed = front_clearance/self.lookahead * min(init_speed, self.speed_max)
        #init_speed = front_clearance / self.lookahead * self.speed_max
        curvature = abs(steering)

        if curvature < 0.12:
            init_speed = self.speed_max
        elif curvature < 0.28:
            init_speed = 0.9 * self.speed_max
        elif curvature < 0.5:
            init_speed = 0.75 * self.speed_max
        else:
            init_speed = 0.55 * self.speed_max
        speed = self.speed_pid.update(init_speed)

        ### BIN SPEED ###
        if self.do_bin_speed:
            bins = np.linspace(self.speed_min, self.speed_max, self.bin_number)
            for bin_speed in bins:
                if speed < bin_speed:
                    speed = bin_speed
                    break

        ### LOW PASS FILTER ###
        if self.do_speed_low_pass_filter:
            speed = (
                self.speed_low_pass_filter * speed
                + (1 - self.speed_low_pass_filter) * self.last_speed
            )
        if self.do_steering_low_pass_filter:
            steering = (
                self.steering_low_pass_filter * steering
                + (1 - self.steering_low_pass_filter) * self.last_steering
            )
        
        self.last_speed = speed
        self.last_steering = steering

        ackermann = {"speed": speed, "steering": steering}
        print(speed)

        ### VISUALISATION ###
        if self.visualise:
            # Visualise Modified Ranges
            self.safety_scan_msg = scan_msg
            scan_msg.ranges = modified_ranges.tolist()
            # Visualise Marked Ranges
            self.safety_markers["range"].clear()
            self.safety_markers["bearing"].clear()
            for i_range in marked_indexes:
                bearing = angle_increment * (i_range[0] - ranges.shape[0] // 2)
                self.safety_markers["range"].append(i_range[1])
                self.safety_markers["bearing"].append(bearing)
            # Visualise Goal
            self.goal_marker["range"] = modified_ranges[max_gap_index]
            self.goal_marker["bearing"] = goal_bearing

        return ackermann

    def get_bubble_coord(self):
        m = []
        for i, r in enumerate(self.safety_markers["range"]):
            x = r * np.cos(self.safety_markers["bearing"][i])
            y = r * np.sin(self.safety_markers["bearing"][i])
            m.append([x, y])
        return m

    def get_goal_coord(self):
        x = self.goal_marker["range"] * np.cos(self.goal_marker["bearing"])
        y = self.goal_marker["range"] * np.sin(self.goal_marker["bearing"])
        return [x, y]

    def get_safety_scan(self):
        return self.safety_scan_msg


class GapFinderNode(Node):
    """
    ROS2 Node Class that handles all the subscibers and publishers for the gap finder algorithm.
    It abstracts the gap finder algorithm from the ROS2 interface.
    The only things to tune or change are in the sections:
    - ROS2 PARAMETERS
    - SPEED AND STEERING LIMITS
    - GAP FINDER ALGORITHM
    """

    def __init__(self):
        ### ROS2 PARAMETERS ###
        self.hz = 50.0  # [Hz]
        self.timeout = 1.0  # [s]
        self.visualise = True
        scan_topic = "scan"
        # drive_topic = "/nav/drive"
        drive_topic = "drive"

        ### SPEED AND STEERING LIMITS ###
        # Speed limits
        self.max_speed = 10.0  # [m/s]
        self.min_speed = 1.0  # [m/s]
        # Acceleration limits
        self.max_acceleration = None  # [m/s^2]
        # Steering limits
        self.max_steering = 0.75  # [rad]

        ### GAP FINDER ALGORITHM ###
        self.gapFinderAlgorithm = GapFinderAlgorithm(
            disparity_bubble_diameter=0.4,
            safety_bubble_diameter=1,
            front_bubble_diameter=0.33,
            minimum_bubble_diameter=0.33,
            view_angle=3.142,
            coeffiecient_of_friction=0.71,
            disparity_threshold=0.6,
            lookahead=None,
            speed_kp=1.0,
            steering_kp=1.2,
            wheel_base=0.324,
            speed_max=10.0,
            speed_min=1.0,
            speed_low_pass_filter=0.8,
            steering_low_pass_filter=0.8,
            visualise=True,
            bin_number=7,
        )
        

        ### ROS2 NODE ###
        super().__init__("gap_finder")
        # Scan Subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 1
        )
        self.scan_subscriber  # prevent unused variable warning
        self.scan_ready = False
        self.last_scan_time = self.get_time()
        # Drive Publisher
        self.drive_msg = AckermannDriveStamped()
        self.last_drive_msg = AckermannDriveStamped()
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 1
        )
        self.last_drive_time = self.get_time()
        # Viz Publishers
        if self.visualise:
            self.init_visualisation()
        # Timer
        self.timer = self.create_timer(1 / self.hz, self.timer_callback)

    

    def init_visualisation(self):
        # Safety Viz Publisher
        self.bubble_viz_publisher = self.create_publisher(
            MarkerArray, "/safety_bubble", 1
        )
        self.bubble_viz_msg = Marker()
        self.bubble_viz_msg.header.frame_id = "/ego_racecar/base_link"
        self.bubble_viz_msg.color.a = 1.0
        self.bubble_viz_msg.color.r = 1.0
        self.bubble_viz_msg.scale.x = self.gapFinderAlgorithm.disparity_bubble_diameter
        self.bubble_viz_msg.scale.y = self.gapFinderAlgorithm.disparity_bubble_diameter
        self.bubble_viz_msg.scale.z = self.gapFinderAlgorithm.disparity_bubble_diameter
        self.bubble_viz_msg.type = Marker.SPHERE
        self.bubble_viz_msg.action = Marker.ADD
        # Goal Viz Publisher
        self.gap_viz_publisher = self.create_publisher(Marker, "/goal_point", 1)
        self.goal_viz_msg = Marker()
        self.goal_viz_msg.header.frame_id = "/ego_racecar/base_link"
        self.goal_viz_msg.color.a = 1.0
        self.goal_viz_msg.color.g = 1.0
        self.goal_viz_msg.scale.x = 0.3
        self.goal_viz_msg.scale.y = 0.3
        self.goal_viz_msg.scale.z = 0.3
        self.goal_viz_msg.type = Marker.SPHERE
        self.goal_viz_msg.action = Marker.ADD
        # Laser Viz Publisher
        self.laser_publisher = self.create_publisher(LaserScan, "/safety_scan", 1)

    def get_time(self):
        """
        Returns the current time in seconds
        """
        return (
            self.get_clock().now().to_msg().sec
            + self.get_clock().now().to_msg().nanosec * 1e-9
        )

    def scan_callback(self,scan_msg):
        self.scan_ready = True
        self.scan_msg = scan_msg
        self.last_scan_time = self.get_time()
         


    def publish_drive_msg(self, drive={"speed": 0.0, "steering": 0.0}):
        self.drive_msg.drive.speed = float(drive["speed"])
        self.drive_msg.drive.steering_angle = float(drive["steering"])
        self.drive_publisher.publish(self.drive_msg)
        self.last_drive_msg = self.drive_msg
        self.last_drive_time = self.get_time()

    def publish_viz_msgs(self):
        safety_scan = self.gapFinderAlgorithm.get_safety_scan()
        bubble_coord = self.gapFinderAlgorithm.get_bubble_coord()
        goal_coord = self.gapFinderAlgorithm.get_goal_coord()
        laser_viz_msg = safety_scan
        bubble_array_viz_msg = MarkerArray()

        for i, coord in enumerate(bubble_coord):
            self.bubble_viz_msg = Marker()
            self.bubble_viz_msg.header.frame_id = "/ego_racecar/base_link"
            self.bubble_viz_msg.color.a = 1.0
            self.bubble_viz_msg.color.r = 1.0
            self.bubble_viz_msg.scale.x = (
                self.gapFinderAlgorithm.disparity_bubble_diameter
            )
            self.bubble_viz_msg.scale.y = (
                self.gapFinderAlgorithm.disparity_bubble_diameter
            )
            self.bubble_viz_msg.scale.z = (
                self.gapFinderAlgorithm.disparity_bubble_diameter
            )
            self.bubble_viz_msg.type = Marker.SPHERE
            self.bubble_viz_msg.action = Marker.ADD
            self.bubble_viz_msg.id = i
            self.bubble_viz_msg.pose.position.x = coord[0]
            self.bubble_viz_msg.pose.position.y = coord[1]
            bubble_array_viz_msg.markers.append(self.bubble_viz_msg)

        self.goal_viz_msg.pose.position.x = goal_coord[0]
        self.goal_viz_msg.pose.position.y = goal_coord[1]

        self.bubble_viz_publisher.publish(bubble_array_viz_msg)
        self.gap_viz_publisher.publish(self.goal_viz_msg)
        self.laser_publisher.publish(laser_viz_msg)

    def timer_callback(self):
        if self.scan_ready:
            ### UPDATE GAP FINDER ALGORITHM ###
            drive = self.gapFinderAlgorithm.update(self.scan_msg)

            ### APPLY SPEED AND STEERING LIMITS ###
            # steering limits
            drive["steering"] = np.sign(drive["steering"]) * min(
                np.abs(drive["steering"]), self.max_steering
            )
            # speed limits
            drive["speed"] = max(drive["speed"], self.min_speed)
            drive["speed"] = min(drive["speed"], self.max_speed)
            # acceleration limits
            if self.max_acceleration is not None:
                dt = self.get_time() - self.last_drive_time
                d_speed = drive["speed"] - self.last_drive_msg.drive.speed
                if abs(d_speed) > self.max_acceleration * dt:
                    # accelerate
                    if drive["speed"] > self.last_drive_msg.drive.speed:
                        drive["speed"] = (
                            self.last_drive_msg.drive.speed + self.max_acceleration
                        )
                    # decelerate
                    else:
                        drive["speed"] = (
                            self.last_drive_msg.drive.speed - self.max_acceleration
                        )
            # drive["speed"] = 0.0

            ### PUBLISH DRIVE MESSAGE ###
            self.publish_drive_msg(drive)
            self.last_time = self.get_time()

            #### PUBLISH VISUALISATION MESSAGES ###
            if self.visualise:
                self.publish_viz_msgs()

        ### TIMEOUT ###
        if (self.get_time() - self.last_scan_time) > self.timeout:
            # self.scan_ready = False
            pass


def main(args=None):
    rclpy.init(args=args)

    gapFinder = GapFinderNode()

    rclpy.spin(gapFinder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gapFinder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()