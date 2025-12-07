"""
Extended Experiment Runner with Hybrid Controller
Runs experiments comparing Pure Pursuit, Stanley, and Hybrid controllers
"""

import glob
import os
import sys
import time
import json
import numpy as np
from datetime import datetime

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from pure_pursuit import PurePursuitController
from stanley import StanleyController
from hybrid_controller import HybridController


class ExtendedExperimentRunner:
    """
    Manages CARLA simulation and runs extended controller experiments.
    Includes hybrid controller and enhanced logging.
    """
    
    def __init__(self, host='localhost', port=2000):
        """
        Initialize experiment runner.
        
        Args:
            host (str): CARLA server host
            port (int): CARLA server port
        """
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = None
        self.vehicle = None
        self.spawn_point = None
        self.waypoints = []
        
    def setup_world(self, town='Town01', weather=carla.WeatherParameters.ClearNoon):
        """
        Setup the CARLA world and load the map.
        
        Args:
            town (str): CARLA town/map name
            weather: Weather parameters
        """
        print(f"Loading world: {town}")
        self.world = self.client.load_world(town)
        self.world.set_weather(weather)
        
        # Set synchronous mode for deterministic simulation
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        self.world.apply_settings(settings)
        
        print("World setup complete")
        
    def generate_waypoints(self, distance=2.0):
        """
        Generate waypoints along the road using CARLA map.
        
        Args:
            distance (float): Distance between waypoints
        """
        print("Generating waypoints...")
        carla_map = self.world.get_map()
        self.waypoints = carla_map.generate_waypoints(distance)
        print(f"Generated {len(self.waypoints)} waypoints")
        
    def spawn_vehicle(self, vehicle_type='vehicle.tesla.model3'):
        """
        Spawn the ego vehicle at a spawn point.
        
        Args:
            vehicle_type (str): Blueprint name for the vehicle
        """
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(vehicle_type)
        
        # Get spawn points
        spawn_points = self.world.get_map().get_spawn_points()
        self.spawn_point = spawn_points[0]  # Use first spawn point
        
        # Spawn vehicle
        self.vehicle = self.world.spawn_actor(vehicle_bp, self.spawn_point)
        print(f"Spawned vehicle: {vehicle_type}")
        
        # Let the vehicle settle
        for _ in range(10):
            self.world.tick()
        
    def cleanup(self):
        """
        Clean up CARLA actors and restore settings.
        """
        if self.vehicle is not None:
            self.vehicle.destroy()
            print("Vehicle destroyed")
        
        if self.world is not None:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
    
    def run_experiment(self, controller, experiment_name, duration=60.0):
        """
        Run a single experiment with a given controller.
        
        Args:
            controller: Controller instance
            experiment_name (str): Name for this experiment
            duration (float): Experiment duration in seconds
            
        Returns:
            dict: Logged metrics
        """
        print(f"\n{'='*60}")
        print(f"Running experiment: {experiment_name}")
        print(f"{'='*60}")
        
        # Reset vehicle to spawn point
        self.vehicle.set_transform(self.spawn_point)
        
        # Apply brake to stop vehicle (CARLA 0.9.15 compatible)
        control = carla.VehicleControl()
        control.brake = 1.0
        control.throttle = 0.0
        self.vehicle.apply_control(control)
        
        # Wait for vehicle to settle
        for _ in range(20):
            self.world.tick()
        
        # Data logging
        metrics = {
            'experiment_name': experiment_name,
            'timestamp': datetime.now().isoformat(),
            'lateral_errors': [],
            'heading_errors': [],
            'steering_angles': [],
            'speeds': [],
            'timestamps': [],
            'positions': [],
            'curvatures': [],  # New: track curvature
            'active_controllers': [],  # New: for hybrid controller
            'blend_weights': [],  # New: for hybrid controller
        }
        
        start_time = time.time()
        step = 0
        
        try:
            while time.time() - start_time < duration:
                # Get control command from controller
                control = controller.run_step(self.vehicle, self.waypoints)
                self.vehicle.apply_control(control)
                
                # Tick simulation
                self.world.tick()
                
                # Log metrics
                vehicle_transform = self.vehicle.get_transform()
                vehicle_location = vehicle_transform.location
                
                # Find closest waypoint for error calculation
                min_distance = float('inf')
                closest_waypoint = None
                for waypoint in self.waypoints:
                    distance = vehicle_location.distance(waypoint.transform.location)
                    if distance < min_distance:
                        min_distance = distance
                        closest_waypoint = waypoint
                
                if closest_waypoint is not None:
                    # Lateral error (cross-track error)
                    lateral_error = min_distance
                    
                    # Heading error
                    vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
                    waypoint_yaw = np.radians(closest_waypoint.transform.rotation.yaw)
                    heading_error = waypoint_yaw - vehicle_yaw
                    heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
                    
                    # Log data
                    metrics['lateral_errors'].append(lateral_error)
                    metrics['heading_errors'].append(np.degrees(heading_error))
                    metrics['steering_angles'].append(control.steer)
                    
                    velocity = self.vehicle.get_velocity()
                    speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                    metrics['speeds'].append(speed)
                    
                    metrics['timestamps'].append(time.time() - start_time)
                    metrics['positions'].append({
                        'x': vehicle_location.x,
                        'y': vehicle_location.y,
                        'z': vehicle_location.z
                    })
                    
                    # Log hybrid controller info if available
                    if hasattr(controller, 'get_controller_info'):
                        info = controller.get_controller_info()
                        metrics['curvatures'].append(info.get('curvature', 0.0))
                        metrics['active_controllers'].append(info.get('active_controller', 'N/A'))
                        metrics['blend_weights'].append(info.get('blend_weight', 0.0))
                    else:
                        metrics['curvatures'].append(0.0)
                        metrics['active_controllers'].append(experiment_name)
                        metrics['blend_weights'].append(0.0)
                
                step += 1
                
                # Print progress
                if step % 50 == 0:
                    print(f"Step {step}, Time: {time.time() - start_time:.2f}s")
        
        except KeyboardInterrupt:
            print("\nExperiment interrupted by user")
        
        # Compute summary statistics
        metrics['summary'] = {
            'mean_lateral_error': np.mean(metrics['lateral_errors']),
            'max_lateral_error': np.max(metrics['lateral_errors']),
            'std_lateral_error': np.std(metrics['lateral_errors']),
            'mean_abs_heading_error': np.mean(np.abs(metrics['heading_errors'])),
            'max_abs_heading_error': np.max(np.abs(metrics['heading_errors'])),
            'steering_smoothness': np.std(np.diff(metrics['steering_angles'])) if len(metrics['steering_angles']) > 1 else 0,
            'mean_speed': np.mean(metrics['speeds']),
            'total_steps': step,
            'total_time': time.time() - start_time,
            'mean_curvature': np.mean(metrics['curvatures']) if metrics['curvatures'] else 0.0,
        }
        
        print(f"\nExperiment complete: {experiment_name}")
        print(f"Mean Lateral Error: {metrics['summary']['mean_lateral_error']:.3f} m")
        print(f"Mean Heading Error: {metrics['summary']['mean_abs_heading_error']:.3f}°")
        print(f"Steering Smoothness (std): {metrics['summary']['steering_smoothness']:.4f}")
        
        return metrics
    
    def save_metrics(self, metrics, filename):
        """
        Save metrics to JSON file.
        
        Args:
            metrics (dict): Metrics dictionary
            filename (str): Output filename
        """
        os.makedirs('results', exist_ok=True)
        filepath = os.path.join('results', filename)
        
        with open(filepath, 'w') as f:
            json.dump(metrics, f, indent=2)
        
        print(f"Metrics saved to: {filepath}")


def main():
    """
    Main function to run all experiments including hybrid controllers.
    """
    runner = ExtendedExperimentRunner()
    
    try:
        # Setup
        runner.setup_world(town='Town01')
        runner.generate_waypoints(distance=2.0)
        runner.spawn_vehicle()
        
        all_metrics = []
        
        # Experiment 1: Pure Pursuit with different lookahead distances
        print("\n" + "="*60)
        print("PURE PURSUIT ABLATION STUDY")
        print("="*60)
        
        lookahead_distances = [2.0, 3.0, 5.0]
        for ld in lookahead_distances:
            controller = PurePursuitController(lookahead_distance=ld)
            metrics = runner.run_experiment(
                controller,
                f"PurePursuit_Ld{ld}",
                duration=60.0
            )
            all_metrics.append(metrics)
            runner.save_metrics(metrics, f'pure_pursuit_ld{ld}.json')
            time.sleep(2)
        
        # Experiment 2: Stanley with different gains
        print("\n" + "="*60)
        print("STANLEY CONTROLLER ABLATION STUDY")
        print("="*60)
        
        k_values = [0.5, 1.0, 2.0]
        for k in k_values:
            controller = StanleyController(k=k)
            metrics = runner.run_experiment(
                controller,
                f"Stanley_K{k}",
                duration=60.0
            )
            all_metrics.append(metrics)
            runner.save_metrics(metrics, f'stanley_k{k}.json')
            time.sleep(2)
        
        # Experiment 3: Hybrid Controllers (NEW!)
        print("\n" + "="*60)
        print("HYBRID CONTROLLER EXPERIMENTS")
        print("="*60)
        
        # Hybrid Mode 1: Switching
        print("\nTesting Hybrid Controller - Switching Mode")
        controller = HybridController(
            pp_lookahead=3.0,
            stanley_k=0.5,
            curvature_threshold=0.05,
            mode='switching'
        )
        metrics = runner.run_experiment(
            controller,
            "Hybrid_Switching",
            duration=60.0
        )
        all_metrics.append(metrics)
        runner.save_metrics(metrics, 'hybrid_switching.json')
        time.sleep(2)
        
        # Hybrid Mode 2: Blending
        print("\nTesting Hybrid Controller - Blending Mode")
        controller = HybridController(
            pp_lookahead=3.0,
            stanley_k=0.5,
            curvature_threshold=0.05,
            mode='blending'
        )
        metrics = runner.run_experiment(
            controller,
            "Hybrid_Blending",
            duration=60.0
        )
        all_metrics.append(metrics)
        runner.save_metrics(metrics, 'hybrid_blending.json')
        time.sleep(2)
        
        # Hybrid Mode 3: Adaptive (with speed adaptation)
        print("\nTesting Hybrid Controller - Adaptive Mode")
        controller = HybridController(
            pp_lookahead=3.0,
            stanley_k=0.5,
            curvature_threshold=0.05,
            mode='adaptive'
        )
        metrics = runner.run_experiment(
            controller,
            "Hybrid_Adaptive",
            duration=60.0
        )
        all_metrics.append(metrics)
        runner.save_metrics(metrics, 'hybrid_adaptive.json')
        time.sleep(2)
        
        # Save combined results
        runner.save_metrics(all_metrics, 'all_experiments_extended.json')
        
        print("\n" + "="*60)
        print("ALL EXPERIMENTS COMPLETE")
        print("="*60)
        print(f"Total experiments run: {len(all_metrics)}")
        print(f"Results saved in './results/' directory")
        print("\nExperiments included:")
        print("  - 3 Pure Pursuit variants")
        print("  - 3 Stanley variants")
        print("  - 3 Hybrid controller modes")
        print(f"  Total: {len(all_metrics)} experiments")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        runner.cleanup()


if __name__ == '__main__':
    main()
