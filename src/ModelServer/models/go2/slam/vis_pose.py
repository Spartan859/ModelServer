import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from threading import Thread

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.odom_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Store trajectory data - no longer limit data quantity, save complete trajectory
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        self.start_point_marked = False  # Mark whether start point has been processed
        
        # Initialize plotting
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        # Main trajectory line (blue)
        self.line, = self.ax.plot([], [], 'b-', label='Trajectory')
        # Start point trajectory (green) - only show a small segment near start
        self.start_line, = self.ax.plot([], [], 'g-', linewidth=2, label='Start Segment')
        # Start point marker
        self.start_marker, = self.ax.plot([], [], 'go', markersize=8, label='Start Point')
        # Current robot position and orientation
        self.robot_marker, = self.ax.plot([], [], 'ro', label='Robot Position')
        self.direction_line, = self.ax.plot([], [], 'r-', label='Robot Direction')
        
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.set_title('Robot Trajectory (Start Point in Green)')
        self.ax.legend()
        self.ax.grid(True)
        
        # Set up animation update
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        
        self.get_logger().info('Trajectory visualization node started. Will save complete trajectory and mark start point.')

    def odom_callback(self, msg):
        """Process received odometry data"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate heading angle (theta) from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Calculate yaw angle (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        theta = np.arctan2(siny_cosp, cosy_cosp)
        
        # Store all data without quantity limits
        self.x_data.append(x)
        self.y_data.append(y)
        self.theta_data.append(theta)

    def update_plot(self, frame):
        """Update the plot"""
        if len(self.x_data) > 0:
            # Update main trajectory line (all trajectory, blue)
            self.line.set_data(self.x_data, self.y_data)
            
            # Process start point marker (only set when first data arrives)
            if not self.start_point_marked:
                self.start_point_marked = True
                # Mark start position
                self.start_marker.set_data(self.x_data[0], self.y_data[0])
                # Show a small segment near start in green (e.g., first 5 points)
                start_points = min(5, len(self.x_data))
                self.start_line.set_data(self.x_data[:start_points], self.y_data[:start_points])
            
            # Update current robot position marker
            self.robot_marker.set_data(self.x_data[-1], self.y_data[-1])
            
            # Update robot direction indicator line
            theta = self.theta_data[-1]
            dir_length = 0.5  # Length of direction line
            x_dir = self.x_data[-1] + dir_length * np.cos(theta)
            y_dir = self.y_data[-1] + dir_length * np.sin(theta)
            self.direction_line.set_data([self.x_data[-1], x_dir], [self.y_data[-1], y_dir])
            
            # Automatically adjust axis range
            self.ax.relim()
            self.ax.autoscale_view()
        
        return self.line, self.start_line, self.start_marker, self.robot_marker, self.direction_line

def main(args=None):
    rclpy.init(args=args)
    
    # Create visualization node
    visualizer = TrajectoryVisualizer()
    
    # Run ROS loop in a separate thread
    thread = Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    thread.start()
    
    try:
        # Display the plot
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown node
        visualizer.destroy_node()
        rclpy.shutdown()
        thread.join()

if __name__ == '__main__':
    main()