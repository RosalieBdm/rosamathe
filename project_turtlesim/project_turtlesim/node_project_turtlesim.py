
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from typing import Tuple, Iterable

from project_interfaces.msg import Trajectory

ANGULAR_THRESHOLD = 0.15
ANGULAR_DRIFT_THRESHOLD = 0.25
DISTANCE_THRESHOLD = 0.1
RADIUS_VISIBILITY = 1

LINEAR_SPEED_DEFAULT = 2.0
ANGULAR_SPEED_COEF = 0.5

STOP = 0
WALK = 1
TURN = 2


class RobotTurtlesim(Node):
    """Class of the robots using Turtlesim tool with ros2

    Args:
        Node (_type_): a ros2 node.
    """

    def __init__(self) -> None:
        super().__init__('node_robot')
        
        self.declare_parameters('', [                                                           # declare the parameters to be given to the node by the launcher
            ('id', -1),
            ('checkpoints', [])
        ])

        self.id = self.get_parameter('id').get_parameter_value().integer_value                  # get the id number from the parameters
        
        # the list of checkpoints is given as a list of floats/doubles in the launch file (no list of tuples type available)
        tmp_list_checkpoints = self.get_parameter('checkpoints').get_parameter_value().double_array_value
        # create the list of checkpoints from this array of doubles
        self.list_checkpoints = [(tmp_list_checkpoints[i], tmp_list_checkpoints[i+1]) for i in range(0, len(tmp_list_checkpoints), 2)]

        self.subscriber_turtlesim_pose = self.create_subscription(
            Pose,                                                                               # message type
            f'/turtlesim1/turtle{self.id}/pose',                                                # topic to subscribe to
            self.turtlesim_pose_callback,                                                       # callback to handle messages
            10                                                                                  # queue size
        )
        self.publisher_motors_control = self.create_publisher(
            Twist,                                                                              # message type
            f'/turtlesim1/turtle{self.id}/cmd_vel',                                             # topic to publish to
            10                                                                                  # queue size
        ) 

        self.subscriber_robot_trajectories = self.create_subscription(
            Trajectory,                                                                         # message type
            '/robot/trajectory',                                                                # topic to subscribe to
            self.robot_trajectories_callback,                                                   # callback to handle the messages
            10                                                                                  # queue size
        )
        
        self.publisher_robot_trajectories = self.create_publisher(
            Trajectory,                                                                         # message type
            '/robot/trajectory',                                                                # topic to publish to     
            10                                                                                  # queue size
        )
                
        self.index: int = 0
        self.checkpoint_to_reach: Tuple[float, float] = self.list_checkpoints[self.index]       # set the next checkpoint coordinates to reach
        self.angle_robot_to_checkpoint: float = None                                            # set the robot orientation to the checkpoint
        self.coordinates: Tuple[float, float] = (0.0, 0.0)                                      # set the position of the robot
        self.orientation: float = 0.0                                                           # set the robot orientation
        self.shifting_force: float = 0.0                                                        # set the value of the Zeghal shifting force
        self.state = WALK
        
    def turtlesim_pose_callback(self, msg: Pose):
        
        self.coordinates = msg.x, msg.y
        self.angle_robot_to_checkpoint = calculate_alignment_with_checkpoint(msg, self.checkpoint_to_reach)
        self.orientation = msg.theta
        
        if (checkpoint_reached(self.coordinates, self.checkpoint_to_reach)):
            self.get_logger().info("Checkpoint reached.")

            # assign a new checkpoint
            self.index += 1
            self.checkpoint_to_reach = self.list_checkpoints[self.index % len(self.list_checkpoints)]
            
        else :
            self.set_state()

            motors_command = Twist()
            motors_command.linear.x = 0.0
            motors_command.linear.y = 0.0
            motors_command.linear.z = 0.0
            motors_command.angular.z = 0.0

            if self.state == WALK:
                self.get_logger().info("Robot is aligned with checkpoint.")
                motors_command.linear.x = LINEAR_SPEED_DEFAULT
            elif self.state == TURN:
                self.get_logger().info("Robot is not aligned with checkpoint.")
                motors_command.angular.z = self.angle_robot_to_checkpoint * ANGULAR_SPEED_COEF
                
            self.publisher_motors_control.publish(motors_command)
        
        trajectory_msg = Trajectory()
        trajectory_msg.id = self.id
        trajectory_msg.x, trajectory_msg.y = self.coordinates
        trajectory_msg.px, trajectory_msg.py = self.checkpoint_to_reach

        # Publish the trajectory message to the topic dedicated to them
        self.publisher_robot_trajectories.publish(trajectory_msg)

    def robot_trajectories_callback(self, msg: Trajectory):
        if msg.id != self.id:
            self.shifting_force = 0
            if (neighbor_detection(self.coordinates, (msg.x, msg.y)) and detect_possible_collision((msg.x, msg.y), (msg.px, msg.py), self.coordinates, self.checkpoint_to_reach)):
                self.get_logger().warning(f"Robot-{msg.id} and Robot-{self.id} might have a collision.")
                pass#self.shifting_force = calculate_zeghal_shift_force(msg.orientation, (msg.x, msg.y), self.orientation, self.coordinates)

    def set_state(self) -> None:
        if self.shifting_force != 0.0:
            self.state = STOP
        elif self.state == STOP:
            self.state = WALK
        elif abs(self.angle_robot_to_checkpoint) < ANGULAR_THRESHOLD and self.state == TURN:
            self.state = STOP
        elif abs(self.angle_robot_to_checkpoint) > ANGULAR_DRIFT_THRESHOLD and self.state == WALK:
            self.state = TURN
        else:
            self.state = WALK

        self.get_logger().info(f"Robot state: {self.state=}")
                

def generate_reset_motors_command() -> Twist:
    command = Twist()
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.z = 0.0

    return command


def checkpoint_reached(coords_robot: Tuple[float, float], coords_checkpoint: Tuple[float, float]) -> bool:
    """Check if a checkpoint has been reached based on the coordinates of a checkpoint and the coordinates of a robot.

    Args:
        coords_robot (Tuple[float, float]): the cartesian coordinates of a robot
        coords_checkpoint (Tuple[float, float]): the cartesian coordinates of a robot's checkpoint to reach

    Returns:
        bool: True if the checkpoint has been reached, else False
    """
    distance = math.sqrt((coords_robot[0] - coords_checkpoint[0])**2 + (coords_robot[1] - coords_checkpoint[1])**2)
    print(f"Distance robot -> checkpoint = {distance:.2f}")
    return distance < DISTANCE_THRESHOLD


def ccw(A,B,C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def detect_possible_collision(coords_r1: Tuple[float, float], coords_dest_r1: Tuple[float, float], coords_r2: Tuple[float, float], coords_dest_r2: Tuple[float, float]) -> bool:
    """Detect if two robots are susceptible to have a collision based on their positions and trajectories.

    Args:
        coords_r1 (Tuple[float, float]): the coordinates of the first robot
        coords_dest_r1 (Tuple[float, float]): the coordinates of the destination of the second robot
        coords_r2 (Tuple[float, float]): the coordinates of the second robot
        coords_dest_r2 (Tuple[float, float]): the coordinates of the destination of the second robot

    Returns:
        bool: True if a collision might occure, else False
    """
    return ccw(coords_r1, coords_r2, coords_dest_r2) != ccw(coords_dest_r1, coords_r2, coords_dest_r2) and ccw(coords_r1, coords_dest_r1, coords_r2) != ccw(coords_r1, coords_dest_r1, coords_dest_r2)


def calculate_zeghal_shift_force(orientation_r1: float, coords_r1: Tuple[float, float], orientation_r2: float, coords_r2: Tuple[float, float]) -> float:
    angle_r1_r2 = calculate_alignment_between_robots(orientation_r1, coords_r1, orientation_r2, coords_r2) # angle du vecteur robot-voisin avec comme référentiel l'orientation du robot
    angle_orientation_r1_r2 = orientation_r2 - orientation_r1 #différence entre l'orientation du robot et l'orientation du voisin 
    return angle_r1_r2 - angle_orientation_r1_r2

def calculate_alignment_between_robots(orientation_r1: float, coords_r1: Tuple[float, float], orientation_r2: float, coords_r2: Tuple[float, float]) -> float:
    pass

def calculate_alignment_with_checkpoint(robot_pose: Pose, coords_robot_dest: Tuple[float, float]) -> float:
    #calculer
    robot_orientation = robot_pose.theta
    vector_robot_to_checkpoint = [coords_robot_dest[0] - robot_pose.x, coords_robot_dest[1] - robot_pose.y]
    if (vector_robot_to_checkpoint[0] == 0):
        return -1
    bearing_dest = math.atan(vector_robot_to_checkpoint[1]/vector_robot_to_checkpoint[0] ) # angle par rapoport à l'axe x autour de l'axe z

    if ((robot_pose.x - coords_robot_dest[0]) > 0):
        bearing_dest += math.pi
    
    if bearing_dest < 0 :
        bearing_dest += 2*math.pi

    if robot_orientation < 0:
        robot_orientation += 2*math.pi

    print(f"Bearing checkpoint: {math.degrees(bearing_dest):.2f}° / Bearing robot: {math.degrees(robot_orientation):.2f}°")
    if (bearing_dest - robot_orientation) > math.pi:
        return bearing_dest - robot_orientation - 2*math.pi
    elif (bearing_dest - robot_orientation) < - math.pi:
        return bearing_dest - robot_orientation + 2*math.pi
    return bearing_dest - robot_orientation# angle depuis l'orientation du robot vers le vecteur point-robot 

def neighbor_detection(coords_r2: Tuple[float, float], coords_r1: Tuple[float, float]) -> bool:
    """Determine if a robot is in the visibility radius of another robot.

    Args:
        coords_r2 (Tuple[float, float]): the cartesian coordinates of a robot
        coords_r1 (Tuple[float, float]): the cartesian coordinates of a robot

    Returns:
        bool: True if the robots see each other, else False
    """
    return math.sqrt((coords_r2[0] - coords_r1[0])**2 + (coords_r2[1] - coords_r1[1])**2) < RADIUS_VISIBILITY


def euler_from_quaternion(quaternion) -> Tuple[float, float, float]:
    x = quaternion.x 
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * ( w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z= math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def main(args=None):
    rclpy.init(args=args)

    node = RobotTurtlesim()

    rclpy.spin(node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()