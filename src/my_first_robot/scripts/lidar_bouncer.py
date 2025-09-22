#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # We are now using LaserScan messages

class LidarBouncer:
    def __init__(self):
        # Initialize the node
        rospy.init_node('lidar_bouncer_node', anonymous=True)
        
        # Create a publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Create a subscriber to the /scan topic
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Register the shutdown hook
        rospy.on_shutdown(self.shutdown_hook)
        
        # Set the loop rate
        self.rate = rospy.Rate(10) # 10hz
        
        rospy.loginfo("Lidar Bouncer node started. Robot will now autonomously avoid walls.")

    def scan_callback(self, scan_data):
        """
        This function is called every time a new /scan message is received.
        This is the "THINK" part of our robot.
        """
        # The scan_data.ranges is an array of 360 distance measurements.
        # The measurement at index 0 is directly in front of the robot.
        front_distance = scan_data.ranges[0]
        
        # Define a safety threshold. If a wall is closer than this, we turn.
        safety_threshold = 0.3 # 30 centimeters
        
        # Create a velocity command message
        move_cmd = Twist()
        
        # --- Decision Logic ---
        if front_distance > safety_threshold:
            # If the way ahead is clear, move forward.
            rospy.loginfo(f"Path is clear. Distance: {front_distance:.2f}m. Moving forward.")
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
        else:
            # If a wall is too close, stop and turn left.
            rospy.loginfo(f"Obstacle detected! Distance: {front_distance:.2f}m. Turning.")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.5 # Turn left at 0.5 rad/s
            
        # Publish the command
        self.velocity_publisher.publish(move_cmd)

    def shutdown_hook(self):
        """Publishes a final stop command before shutting down."""
        rospy.loginfo("Node is shutting down. Sending stop command.")
        stop_cmd = Twist()
        self.velocity_publisher.publish(stop_cmd)
        rospy.sleep(1)

    def run(self):
        """The main loop of the node."""
        # rospy.spin() is needed to keep the node alive and allow callbacks to be triggered.
        rospy.spin()

# Main entry point of the script
if __name__ == '__main__':
    try:
        bouncer = LidarBouncer()
        bouncer.run()
    except rospy.ROSInterruptException:
        pass
