"""
Stanley Controller for Lane Keeping in CARLA
Implements lateral control using cross-track error and heading error
"""

import numpy as np
import carla


class StanleyController:
    """
    Stanley lateral controller for path tracking.
    
    The controller computes steering based on heading error and 
    cross-track error from the desired path.
    """
    
    def __init__(self, k=1.0, wheelbase=2.875):
        """
        Initialize Stanley controller.
        
        Args:
            k (float): Cross-track error gain
            wheelbase (float): Vehicle wheelbase length (meters)
        """
        self.k = k
        self.wheelbase = wheelbase
        self.target_speed = 30.0  # km/h
        
    def find_closest_waypoint(self, vehicle_location, waypoints):
        """
        Find the closest waypoint to the vehicle.
        
        Args:
            vehicle_location: Current vehicle location (carla.Location)
            waypoints: List of waypoints representing the path
            
        Returns:
            tuple: (closest_waypoint, index, cross_track_error)
        """
        min_distance = float('inf')
        closest_waypoint = None
        closest_idx = 0
        
        for i, waypoint in enumerate(waypoints):
            distance = vehicle_location.distance(waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_waypoint = waypoint
                closest_idx = i
        
        return closest_waypoint, closest_idx, min_distance
    
    def compute_cross_track_error(self, vehicle_transform, closest_waypoint):
        """
        Compute the cross-track error (lateral deviation from path).
        
        Args:
            vehicle_transform: Current vehicle transform (carla.Transform)
            closest_waypoint: Closest waypoint on the path
            
        Returns:
            float: Cross-track error (positive = right of path, negative = left)
        """
        # Vehicle position and heading
        vehicle_location = vehicle_transform.location
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        # Waypoint position
        waypoint_location = closest_waypoint.transform.location
        
        # Vector from vehicle to waypoint
        dx = waypoint_location.x - vehicle_location.x
        dy = waypoint_location.y - vehicle_location.y
        
        # Transform to vehicle's local coordinate frame
        # Cross-track error is the lateral component
        cross_track_error = -dx * np.sin(vehicle_yaw) + dy * np.cos(vehicle_yaw)
        
        return cross_track_error
    
    def compute_heading_error(self, vehicle_transform, closest_waypoint, next_waypoint):
        """
        Compute the heading error (difference between vehicle and path heading).
        
        Args:
            vehicle_transform: Current vehicle transform (carla.Transform)
            closest_waypoint: Closest waypoint on the path
            next_waypoint: Next waypoint on the path (for computing path heading)
            
        Returns:
            float: Heading error in radians
        """
        # Vehicle heading
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        # Path heading (from closest to next waypoint)
        if next_waypoint is not None:
            dx = next_waypoint.transform.location.x - closest_waypoint.transform.location.x
            dy = next_waypoint.transform.location.y - closest_waypoint.transform.location.y
            path_yaw = np.arctan2(dy, dx)
        else:
            # Use waypoint's own heading if no next waypoint
            path_yaw = np.radians(closest_waypoint.transform.rotation.yaw)
        
        # Compute heading error (normalize to [-pi, pi])
        heading_error = path_yaw - vehicle_yaw
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        return heading_error
    
    def compute_steering(self, vehicle_transform, vehicle_speed, waypoints):
        """
        Compute steering angle using Stanley controller formula.
        
        Args:
            vehicle_transform: Current vehicle transform (carla.Transform)
            vehicle_speed: Current vehicle speed (m/s)
            waypoints: List of waypoints representing the path
            
        Returns:
            float: Steering angle in radians (normalized to [-1, 1] for CARLA)
        """
        vehicle_location = vehicle_transform.location
        
        # Find closest waypoint
        closest_waypoint, closest_idx, _ = self.find_closest_waypoint(
            vehicle_location, waypoints
        )
        
        # Get next waypoint for heading calculation
        next_waypoint = None
        if closest_idx + 1 < len(waypoints):
            next_waypoint = waypoints[closest_idx + 1]
        
        # Compute heading error
        heading_error = self.compute_heading_error(
            vehicle_transform, closest_waypoint, next_waypoint
        )
        
        # Compute cross-track error
        cross_track_error = self.compute_cross_track_error(
            vehicle_transform, closest_waypoint
        )
        
        # Stanley control law
        # steering = heading_error + arctan(k * cross_track_error / speed)
        speed = max(vehicle_speed, 0.1)  # Avoid division by zero
        cross_track_term = np.arctan(self.k * cross_track_error / speed)
        
        steering_angle = heading_error + cross_track_term
        
        # Normalize to [-1, 1] for CARLA (assuming max steering ~70 degrees)
        max_steering_angle = np.radians(70)
        normalized_steering = np.clip(steering_angle / max_steering_angle, -1.0, 1.0)
        
        return normalized_steering
    
    def run_step(self, vehicle, waypoints):
        """
        Execute one control step.
        
        Args:
            vehicle: CARLA vehicle actor
            waypoints: List of waypoints representing the path
            
        Returns:
            carla.VehicleControl: Control command for the vehicle
        """
        vehicle_transform = vehicle.get_transform()
        vehicle_speed = self.get_speed(vehicle)
        
        # Compute steering
        steering = self.compute_steering(vehicle_transform, vehicle_speed, waypoints)
        
        # Simple speed control
        if vehicle_speed < self.target_speed / 3.6:  # Convert km/h to m/s
            throttle = 0.5
            brake = 0.0
        else:
            throttle = 0.0
            brake = 0.3
        
        # Create control command
        control = carla.VehicleControl()
        control.steer = float(steering)
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        
        return control
    
    @staticmethod
    def get_speed(vehicle):
        """
        Get vehicle speed in m/s.
        
        Args:
            vehicle: CARLA vehicle actor
            
        Returns:
            float: Speed in m/s
        """
        velocity = vehicle.get_velocity()
        return np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
