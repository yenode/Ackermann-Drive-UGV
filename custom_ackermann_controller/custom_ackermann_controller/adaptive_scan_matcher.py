#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Float64
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class AdaptiveScanMatcherNode(Node):
    def __init__(self):
        super().__init__('adaptive_scan_matcher')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('max_iterations', 15)
        self.declare_parameter('epsilon_xy', 0.0001)
        self.declare_parameter('epsilon_theta', 0.0001)
        self.declare_parameter('max_correspondence_distance', 0.3)
        self.declare_parameter('outlier_threshold', 0.05)
        self.declare_parameter('min_laser_range', 0.1)
        self.declare_parameter('max_laser_range', 10.0)
        self.declare_parameter('downsample_factor', 2)
        self.declare_parameter('min_feature_threshold', 10)
        self.declare_parameter('quality_threshold', 0.3)
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.max_iterations = self.get_parameter('max_iterations').get_parameter_value().integer_value
        self.epsilon_xy = self.get_parameter('epsilon_xy').value
        self.epsilon_theta = self.get_parameter('epsilon_theta').value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.min_laser_range = self.get_parameter('min_laser_range').value
        self.max_laser_range = self.get_parameter('max_laser_range').value
        self.downsample_factor = self.get_parameter('downsample_factor').get_parameter_value().integer_value
        self.min_feature_threshold = self.get_parameter('min_feature_threshold').get_parameter_value().integer_value
        self.quality_threshold = self.get_parameter('quality_threshold').get_parameter_value().double_value
        
        # State variables
        self.last_scan = None
        self.last_pose = None
        self.last_time = None
        self.scan_matching_quality = 0.0
        self.feature_count = 0
        self.matching_confidence = 1.0
        self.linear_velocity = 0.0  # vx
        self.angular_velocity = 0.0  # vtheta
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/scan_matching_odometry/odom', 10)
        self.quality_pub = self.create_publisher(Float64, '/scan_matching/quality', 10)
        self.confidence_pub = self.create_publisher(Float64, '/scan_matching/confidence', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Adaptive Scan Matcher Node initialized')

    def preprocess_scan(self, scan_msg):
        """Preprocess laser scan data"""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter by range
        valid_mask = (ranges >= self.min_laser_range) & (ranges <= self.max_laser_range)
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        
        # Remove infinite and NaN values
        finite_mask = np.isfinite(valid_ranges)
        valid_ranges = valid_ranges[finite_mask]
        valid_angles = valid_angles[finite_mask]
        
        # Downsample
        if self.downsample_factor > 1:
            indices = np.arange(0, len(valid_ranges), self.downsample_factor)
            valid_ranges = valid_ranges[indices]
            valid_angles = valid_angles[indices]
        
        # Convert to Cartesian coordinates
        points = np.column_stack([
            valid_ranges * np.cos(valid_angles),
            valid_ranges * np.sin(valid_angles)
        ])
        
        return points, valid_ranges, valid_angles

    def detect_features(self, points):
        """Detect features in point cloud for quality assessment"""
        if len(points) < 3:
            return 0
        
        features = 0
        
        # Detect corners using angle changes
        for i in range(1, len(points) - 1):
            p1, p2, p3 = points[i-1], points[i], points[i+1]
            
            # Calculate vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Calculate angle between vectors
            dot_product = np.dot(v1, v2)
            norms = np.linalg.norm(v1) * np.linalg.norm(v2)
            
            if norms > 0:
                cos_angle = np.clip(dot_product / norms, -1, 1)
                angle = np.arccos(cos_angle)
                
                # If angle change is significant, it's a feature
                if angle < 2.6:  # Less than 150 degrees
                    features += 1
        
        return features

    def assess_scan_quality(self, points):
        """Assess the quality of scan for matching"""
        if len(points) < 5:
            return 0.0, 0
        
        # Count features
        feature_count = self.detect_features(points)
        
        # Calculate point density
        if len(points) > 1:
            distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
            avg_point_spacing = np.mean(distances)
            density_score = min(1.0, 0.1 / max(avg_point_spacing, 0.01))
        else:
            density_score = 0.0
        
        # Calculate coverage
        angles = np.arctan2(points[:, 1], points[:, 0])
        angle_range = np.max(angles) - np.min(angles)
        coverage_score = min(1.0, angle_range / (2 * np.pi))
        
        # Feature density score
        feature_density = feature_count / len(points)
        feature_score = min(1.0, feature_density * 100)
        
        # Overall quality
        quality = (density_score * 0.3 + coverage_score * 0.3 + feature_score * 0.4)
        
        return quality, feature_count

    def simple_icp(self, source_points, target_points, initial_pose):
        """Simplified ICP implementation"""
        if len(source_points) < 3 or len(target_points) < 3:
            return initial_pose, 0.0

        try:
            current_pose = initial_pose.copy()

            for iteration in range(self.max_iterations):
                # Transform source points using current pose
                cos_theta = np.cos(current_pose[2])
                sin_theta = np.sin(current_pose[2])
                rotation_matrix = np.array([[cos_theta, -sin_theta],
                                            [sin_theta, cos_theta]])

                transformed_source = (rotation_matrix @ source_points.T).T
                transformed_source[:, 0] += current_pose[0]
                transformed_source[:, 1] += current_pose[1]

                # Find correspondences
                correspondences = []
                distances = []

                for i, point in enumerate(transformed_source):
                    dists = np.linalg.norm(target_points - point, axis=1)
                    min_idx = np.argmin(dists)
                    min_dist = dists[min_idx]

                    if min_dist < self.max_correspondence_distance:
                        correspondences.append((i, min_idx))
                        distances.append(min_dist)

                if len(correspondences) < 3:
                    break

                # Remove outliers
                distances = np.array(distances)
                median_dist = np.median(distances)
                outlier_threshold = median_dist + 2 * np.std(distances)

                good_correspondences = []
                for i, (src_idx, tgt_idx) in enumerate(correspondences):
                    if distances[i] < outlier_threshold:
                        good_correspondences.append((src_idx, tgt_idx))

                if len(good_correspondences) < 3:
                    break

                # Calculate transformation from matched correspondences
                src_matched = np.array([source_points[src_idx] for src_idx, _ in good_correspondences])
                tgt_matched = np.array([target_points[tgt_idx] for _, tgt_idx in good_correspondences])

                # Calculate centroids
                src_centroid = np.mean(src_matched, axis=0)
                tgt_centroid = np.mean(tgt_matched, axis=0)

                # Center the points
                src_centered = src_matched - src_centroid
                tgt_centered = tgt_matched - tgt_centroid

                # Calculate rotation using SVD
                H = src_centered.T @ tgt_centered
                U, _, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T

                # Ensure proper rotation matrix
                if np.linalg.det(R) < 0:
                    Vt[-1, :] *= -1
                    R = Vt.T @ U.T

                # Calculate translation
                t = tgt_centroid - R @ src_centroid

                # Extract angle from rotation matrix
                theta = np.arctan2(R[1, 0], R[0, 0])

                # Update pose incrementally
                delta_pose = np.array([t[0], t[1], theta])
                current_pose += delta_pose

                # Check convergence
                if (np.abs(delta_pose[0]) < self.epsilon_xy and
                    np.abs(delta_pose[1]) < self.epsilon_xy and
                    np.abs(delta_pose[2]) < self.epsilon_theta):
                    break

            # After iterations, compute a matching score based on nearest neighbors
            # Transform source_points by final pose
            cos_theta = np.cos(current_pose[2])
            sin_theta = np.sin(current_pose[2])
            rotation_matrix = np.array([[cos_theta, -sin_theta],
                                        [sin_theta, cos_theta]])
            final_transformed = (rotation_matrix @ source_points.T).T
            final_transformed[:, 0] += current_pose[0]
            final_transformed[:, 1] += current_pose[1]

            final_distances = []
            for point in final_transformed:
                dists = np.linalg.norm(target_points - point, axis=1)
                min_dist = np.min(dists)
                if min_dist < self.max_correspondence_distance:
                    final_distances.append(min_dist)

            if final_distances:
                matching_score = 1.0 / (1.0 + float(np.mean(final_distances)))
            else:
                matching_score = 0.0

            return current_pose, matching_score

        except Exception as e:
            self.get_logger().warning(f'ICP iteration failed: {e}')
            return initial_pose, 0.0

    def scan_callback(self, scan_msg):
        """Process incoming laser scan"""
        # Preprocess scan
        points, ranges, angles = self.preprocess_scan(scan_msg)
        
        if len(points) < 5:
            self.get_logger().warn('Too few valid laser points for scan matching')
            return
        
        # Assess scan quality
        quality, feature_count = self.assess_scan_quality(points)
        self.scan_matching_quality = quality
        self.feature_count = feature_count
        
        # Publish quality metrics
        quality_msg = Float64()
        quality_msg.data = quality
        self.quality_pub.publish(quality_msg)
        
        # Determine matching confidence based on quality
        if quality < (self.quality_threshold or 0.3) or feature_count < (self.min_feature_threshold or 10):
            self.matching_confidence = 0.1  # Low confidence
            self.get_logger().debug('Low scan quality, reducing scan matching confidence')
        else:
            self.matching_confidence = min(1.0, quality * 2.0)
        
        # Publish confidence
        confidence_msg = Float64()
        confidence_msg.data = self.matching_confidence
        self.confidence_pub.publish(confidence_msg)
        
        # Get current time
        current_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9
        
        # Perform scan matching if we have a previous scan
        if self.last_scan is not None and self.last_pose is not None and self.last_time is not None:
            # Calculate time difference
            dt = current_time - self.last_time
            
            if dt > 0.01:  # At least 10ms between scans
                # Simple motion prediction as initial guess
                initial_pose = self.last_pose.copy()
                
                # Perform ICP
                new_pose, matching_score = self.simple_icp(points, self.last_scan, initial_pose)
                
                # Calculate velocity from pose change
                delta_x = new_pose[0] - self.last_pose[0]
                delta_y = new_pose[1] - self.last_pose[1]
                delta_theta = new_pose[2] - self.last_pose[2]
                
                # Normalize angle to [-pi, pi]
                while delta_theta > np.pi:
                    delta_theta -= 2 * np.pi
                while delta_theta < -np.pi:
                    delta_theta += 2 * np.pi
                
                # Calculate linear velocity (magnitude)
                self.linear_velocity = np.sqrt(delta_x**2 + delta_y**2) / dt
                
                # Determine direction (forward/backward) based on heading
                movement_angle = np.arctan2(delta_y, delta_x)
                heading_diff = movement_angle - self.last_pose[2]
                while heading_diff > np.pi:
                    heading_diff -= 2 * np.pi
                while heading_diff < -np.pi:
                    heading_diff += 2 * np.pi
                
                # If moving backward (heading diff > 90 degrees), make velocity negative
                if abs(heading_diff) > np.pi / 2:
                    self.linear_velocity = -self.linear_velocity
                
                # Calculate angular velocity
                self.angular_velocity = delta_theta / dt
                
                # Update confidence based on matching score
                self.matching_confidence *= matching_score
                
                # Create and publish odometry message
                self.publish_scan_odometry(scan_msg.header, new_pose, self.matching_confidence)
                
                self.last_pose = new_pose
                self.last_time = current_time
        else:
            # Initialize with identity pose
            self.last_pose = np.array([0.0, 0.0, 0.0])
            self.last_time = current_time
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        
        # Store current scan for next iteration
        self.last_scan = points

    def publish_scan_odometry(self, header, pose, confidence):
        """Publish scan matching odometry"""
        odom_msg = Odometry()
        odom_msg.header = header
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Set pose
        odom_msg.pose.pose.position.x = pose[0]
        odom_msg.pose.pose.position.y = pose[1]
        odom_msg.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Set covariance based on confidence
        base_pos_var = 0.01 / max(confidence, 0.1)
        base_orient_var = 0.02 / max(confidence, 0.1)
        
        odom_msg.pose.covariance = [
            base_pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, base_pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, base_orient_var
        ]
        
        # Set velocity from calculated values
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        # Set velocity covariance based on confidence
        base_vel_var = 0.1 / max(confidence, 0.1)
        base_ang_vel_var = 0.2 / max(confidence, 0.1)
        
        odom_msg.twist.covariance = [
            base_vel_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, base_ang_vel_var
        ]
        
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveScanMatcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()