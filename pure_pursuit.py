"""
Pure Pursuit Controller for Lane Keeping in CARLA
Implements geometric path tracking using lookahead distance
"""

import numpy as np
import carla


class PurePursuitController:
    """
    Pure Pursuit lateral controller for path tracking.
    
    The controller computes steering angle based on a geometric arc
    to a lookahead point on the desired path.
    """
    
    def __init__(self, lookahead_distance=3.0, wheelbase=2.875):
        """
        Initialize Pure Pursuit controller.
        
        Args:
            lookahead_distance (float): Distance ahead to target point (meters)
            wheelbase (float): Vehicle wheelbase length (meters)
        """
        self.lookahead_distance = lookahead_distance
        self.wheelbase = wheelbase
        self.target_speed = 30.0  # km/h
        
    def find_lookahead_point(self, vehicle_location, vehicle_transform, waypoints):
        """
        Find the lookahead point on the path.
        
        Args:
            vehicle_location: Current vehicle location (carla.Location)
            vehicle_transform: Current vehicle transform (carla.Transform)
            waypoints: List of waypoints representing the path
            
        Returns:
            carla.Location: The lookahead point, or None if not found
        """
        # Get vehicle's forward vector
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        min_distance = float('inf')
        closest_waypoint_idx = 0
        
        # Find closest waypoint
        for i, waypoint in enumerate(waypoints):
            distance = vehicle_location.distance(waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_waypoint_idx = i
        
        # Search forward from closest waypoint for lookahead point
        for i in range(closest_waypoint_idx, min(closest_waypoint_idx + 50, len(waypoints))):
            waypoint = waypoints[i]
            distance = vehicle_location.distance(waypoint.transform.location)
            
            if distance >= self.lookahead_distance:
                return waypoint.transform.location
        
        # If no point found at exact lookahead distance, return furthest point
        if closest_waypoint_idx + 30 < len(waypoints):
            return waypoints[closest_waypoint_idx + 30].transform.location
        else:
            return waypoints[-1].transform.location
    
    def compute_steering(self, vehicle_transform, target_location):
        """
        Compute steering angle using Pure Pursuit algorithm.
        
        Args:
            vehicle_transform: Current vehicle transform (carla.Transform)
            target_location: Target lookahead point (carla.Location)
            
        Returns:
            float: Steering angle in radians (normalized to [-1, 1] for CARLA)
        """
        # Vehicle position and heading
        vehicle_location = vehicle_transform.location
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        # Vector from vehicle to target
        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        
        # Transform to vehicle's local coordinate frame
        local_x = dx * np.cos(vehicle_yaw) + dy * np.sin(vehicle_yaw)
        local_y = -dx * np.sin(vehicle_yaw) + dy * np.cos(vehicle_yaw)
        
        # Compute curvature (Pure Pursuit formula)
        ld_squared = local_x**2 + local_y**2
        curvature = 2.0 * local_y / ld_squared if ld_squared > 0.01 else 0.0
        
        # Compute steering angle
        steering_angle = np.arctan(curvature * self.wheelbase)
        
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
        vehicle_location = vehicle_transform.location
        
        # Find lookahead point
        target_location = self.find_lookahead_point(
            vehicle_location, vehicle_transform, waypoints
        )
        
        if target_location is None:
            # No valid target, return zero control
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
        
        # Compute steering
        steering = self.compute_steering(vehicle_transform, target_location)
        
        # Simple speed control
        current_speed = self.get_speed(vehicle)
        if current_speed < self.target_speed / 3.6:  # Convert km/h to m/s
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
