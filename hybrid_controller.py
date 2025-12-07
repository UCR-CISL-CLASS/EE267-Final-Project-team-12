"""
Hybrid Controller for Lane Keeping in CARLA
Intelligently combines Pure Pursuit and Stanley controllers based on driving conditions
"""

import numpy as np
import carla
from pure_pursuit import PurePursuitController
from stanley import StanleyController


class HybridController:
    """
    Hybrid lateral controller that switches between Pure Pursuit and Stanley
    based on path curvature, speed, and tracking error.
    
    Strategy:
    - Use Stanley for sharp turns and precise tracking (high curvature)
    - Use Pure Pursuit for straight roads and comfort (low curvature)
    - Blend smoothly between controllers to avoid discontinuities
    """
    
    def __init__(self, 
                 pp_lookahead=3.0,
                 stanley_k=0.5,
                 curvature_threshold=0.05,
                 blend_zone=0.02,
                 mode='adaptive'):
        """
        Initialize Hybrid controller.
        
        Args:
            pp_lookahead (float): Lookahead distance for Pure Pursuit
            stanley_k (float): Gain for Stanley controller
            curvature_threshold (float): Curvature above which to prefer Stanley
            blend_zone (float): Curvature range for smooth blending
            mode (str): 'adaptive', 'switching', or 'blending'
        """
        # Initialize both controllers with optimal parameters
        self.pure_pursuit = PurePursuitController(lookahead_distance=pp_lookahead)
        self.stanley = StanleyController(k=stanley_k)
        
        # Hybrid control parameters
        self.curvature_threshold = curvature_threshold
        self.blend_zone = blend_zone
        self.mode = mode
        
        # Adaptive parameters
        self.speed_adaptive = True
        
        # Logging
        self.active_controller = "Hybrid"
        self.blend_weight = 0.5
        self.current_curvature = 0.0
        
    def estimate_path_curvature(self, vehicle_location, waypoints, lookahead_points=10):
        """
        Estimate the upcoming path curvature.
        
        Args:
            vehicle_location: Current vehicle location
            waypoints: List of waypoints
            lookahead_points: Number of points ahead to consider
            
        Returns:
            float: Estimated curvature (1/radius)
        """
        # Find closest waypoint
        min_distance = float('inf')
        closest_idx = 0
        
        for i, waypoint in enumerate(waypoints):
            distance = vehicle_location.distance(waypoint.transform.location)
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        # Get lookahead points
        end_idx = min(closest_idx + lookahead_points, len(waypoints) - 1)
        
        if end_idx - closest_idx < 3:
            return 0.0  # Not enough points to estimate curvature
        
        # Extract waypoint positions
        points = []
        for i in range(closest_idx, end_idx):
            wp = waypoints[i].transform.location
            points.append([wp.x, wp.y])
        
        points = np.array(points)
        
        # Fit a circle through the points to estimate curvature
        # Simple approximation: use change in heading over distance
        if len(points) < 3:
            return 0.0
        
        # Calculate heading changes
        dx = np.diff(points[:, 0])
        dy = np.diff(points[:, 1])
        headings = np.arctan2(dy, dx)
        
        # Curvature = change in heading / arc length
        if len(headings) > 1:
            heading_change = np.abs(np.diff(headings))
            # Normalize heading change to [-pi, pi]
            heading_change = np.arctan2(np.sin(heading_change), np.cos(heading_change))
            
            distances = np.sqrt(dx[:-1]**2 + dy[:-1]**2)
            total_distance = np.sum(distances)
            
            if total_distance > 0.1:
                curvature = np.sum(np.abs(heading_change)) / total_distance
                return curvature
        
        return 0.0
    
    def compute_blend_weight(self, curvature, speed=None):
        """
        Compute blending weight between controllers.
        
        Args:
            curvature (float): Path curvature
            speed (float): Vehicle speed (optional)
            
        Returns:
            float: Weight for Stanley (0=Pure Pursuit, 1=Stanley)
        """
        # Base weight on curvature
        # weight = 0 -> Pure Pursuit (straight roads)
        # weight = 1 -> Stanley (sharp turns)
        
        if curvature < self.curvature_threshold - self.blend_zone:
            # Low curvature: Pure Pursuit
            weight = 0.0
        elif curvature > self.curvature_threshold + self.blend_zone:
            # High curvature: Stanley
            weight = 1.0
        else:
            # Blend zone: smooth transition
            # Linear interpolation
            weight = (curvature - (self.curvature_threshold - self.blend_zone)) / (2 * self.blend_zone)
            weight = np.clip(weight, 0.0, 1.0)
        
        # Adjust for speed if adaptive mode
        if self.speed_adaptive and speed is not None:
            # At higher speeds, prefer Pure Pursuit for smoothness
            # At lower speeds, Stanley is fine
            speed_factor = np.clip(speed / 15.0, 0.0, 1.0)  # Normalize to ~15 m/s
            weight = weight * (1.0 - 0.3 * speed_factor)  # Reduce Stanley weight at high speed
        
        return weight
    
    def adaptive_lookahead(self, speed):
        """
        Compute speed-adaptive lookahead distance for Pure Pursuit.
        
        Args:
            speed (float): Vehicle speed in m/s
            
        Returns:
            float: Lookahead distance in meters
        """
        base_lookahead = 2.0
        speed_factor = 0.3  # Additional meters per m/s
        max_lookahead = 6.0
        
        lookahead = base_lookahead + speed * speed_factor
        return min(lookahead, max_lookahead)
    
    def adaptive_stanley_gain(self, speed):
        """
        Compute speed-adaptive gain for Stanley controller.
        
        Args:
            speed (float): Vehicle speed in m/s
            
        Returns:
            float: Stanley gain K
        """
        # Lower gain at higher speeds to reduce oscillation
        if speed < 5.0:
            return 1.0
        elif speed < 10.0:
            return 0.7
        else:
            return 0.5
    
    def run_step(self, vehicle, waypoints):
        """
        Execute one control step using hybrid approach.
        
        Args:
            vehicle: CARLA vehicle actor
            waypoints: List of waypoints representing the path
            
        Returns:
            carla.VehicleControl: Control command for the vehicle
        """
        vehicle_transform = vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_speed = self.get_speed(vehicle)
        
        # Estimate path curvature
        curvature = self.estimate_path_curvature(vehicle_location, waypoints)
        self.current_curvature = curvature
        
        # Adapt controller parameters based on speed
        if self.speed_adaptive:
            self.pure_pursuit.lookahead_distance = self.adaptive_lookahead(vehicle_speed)
            self.stanley.k = self.adaptive_stanley_gain(vehicle_speed)
        
        # Mode 1: Hard switching based on curvature
        if self.mode == 'switching':
            if curvature > self.curvature_threshold:
                self.active_controller = "Stanley"
                return self.stanley.run_step(vehicle, waypoints)
            else:
                self.active_controller = "Pure Pursuit"
                return self.pure_pursuit.run_step(vehicle, waypoints)
        
        # Mode 2: Smooth blending
        elif self.mode == 'blending':
            # Get control from both controllers
            pp_control = self.pure_pursuit.run_step(vehicle, waypoints)
            stanley_control = self.stanley.run_step(vehicle, waypoints)
            
            # Compute blend weight
            weight = self.compute_blend_weight(curvature, vehicle_speed)
            self.blend_weight = weight
            
            # Blend steering commands
            blended_steering = (1 - weight) * pp_control.steer + weight * stanley_control.steer
            
            # Use Pure Pursuit's speed control (it's simpler and works well)
            control = carla.VehicleControl()
            control.steer = float(np.clip(blended_steering, -1.0, 1.0))
            control.throttle = pp_control.throttle
            control.brake = pp_control.brake
            control.hand_brake = False
            control.manual_gear_shift = False
            
            # Update active controller for logging
            if weight < 0.3:
                self.active_controller = "Pure Pursuit (blended)"
            elif weight > 0.7:
                self.active_controller = "Stanley (blended)"
            else:
                self.active_controller = "Hybrid (50/50)"
            
            return control
        
        # Mode 3: Adaptive (default) - Smart switching with conditions
        else:  # mode == 'adaptive'
            # Decision logic
            # Use Stanley if:
            # 1. High curvature, OR
            # 2. Low speed AND moderate curvature
            
            use_stanley = False
            
            if curvature > self.curvature_threshold + self.blend_zone:
                use_stanley = True
            elif curvature > self.curvature_threshold and vehicle_speed < 8.0:
                # At low speeds, Stanley is better even for moderate curves
                use_stanley = True
            
            if use_stanley:
                self.active_controller = "Stanley"
                self.blend_weight = 1.0
                return self.stanley.run_step(vehicle, waypoints)
            else:
                self.active_controller = "Pure Pursuit"
                self.blend_weight = 0.0
                return self.pure_pursuit.run_step(vehicle, waypoints)
    
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
    
    def get_controller_info(self):
        """
        Get current controller state information for logging.
        
        Returns:
            dict: Controller state information
        """
        return {
            'active_controller': self.active_controller,
            'blend_weight': self.blend_weight,
            'curvature': self.current_curvature,
            'pp_lookahead': self.pure_pursuit.lookahead_distance,
            'stanley_k': self.stanley.k
        }
