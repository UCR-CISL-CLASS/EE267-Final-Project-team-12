#!/usr/bin/env python3
"""
Experiment Runner with Camera View Integration
Compatible with actual Pure Pursuit, Stanley, and Hybrid controller implementations
"""

import sys
import time
import json
import carla
import numpy as np
from pathlib import Path

# Import your actual controllers
from pure_pursuit import PurePursuitController
from stanley import StanleyController
from hybrid_controller import HybridController

# Import visualization with camera support
from visualization_with_camera import VisualizationHUD


class ExperimentRunnerWithCamera:
    """Run lane keeping experiments with camera visualization."""
    
    def __init__(self, enable_viz=True):
        """Initialize the experiment runner."""
        self.client = None
        self.world = None
        self.vehicle = None
        self.camera = None
        self.camera_data = None
        self.viz = None
        self.enable_viz = enable_viz
        
        # Results storage
        self.results_dir = Path("results")
        self.results_dir.mkdir(exist_ok=True)
        
    def setup_carla(self):
        """Connect to CARLA and setup the world."""
        print("Connecting to CARLA...")
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        
        self.world = self.client.get_world()
        
        # Set synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        self.world.apply_settings(settings)
        
        print("✓ Connected to CARLA")
    
    def spawn_vehicle(self):
        """Spawn the ego vehicle."""
        print("Spawning vehicle...")
        
        # Get spawn points
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        # Get vehicle blueprint
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('model3')[0]
        
        # Spawn vehicle
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        time.sleep(0.5)
        
        print(f"✓ Spawned vehicle at {spawn_point.location}")
    
    def setup_camera(self):
        """Setup RGB camera on the vehicle."""
        print("Setting up camera...")
        
        # Get camera blueprint
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        
        # Set camera attributes
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '90')
        
        # Camera transform (behind and above vehicle)
        camera_transform = carla.Transform(
            carla.Location(x=-5.5, z=2.8),
            carla.Rotation(pitch=-15)
        )
        
        # Spawn camera
        self.camera = self.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=self.vehicle
        )
        
        # Register callback
        self.camera.listen(lambda image: self._process_camera(image))
        
        print("✓ Camera setup complete")
    
    def _process_camera(self, image):
        """Process camera image callback."""
        # Convert to numpy array
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # BGR to RGB
        
        self.camera_data = array
    
    def get_waypoints_ahead(self, num_waypoints=50, distance=2.0):
        """
        Get waypoints ahead of vehicle using CARLA's map.
        
        Args:
            num_waypoints: Number of waypoints to get
            distance: Distance between waypoints (meters)
            
        Returns:
            List of carla.Waypoint objects
        """
        current_map = self.world.get_map()
        vehicle_location = self.vehicle.get_location()
        
        # Get current waypoint
        waypoint = current_map.get_waypoint(vehicle_location)
        
        # Collect waypoints ahead
        waypoints = [waypoint]
        for _ in range(num_waypoints - 1):
            waypoint_list = waypoint.next(distance)
            if waypoint_list:
                waypoint = waypoint_list[0]
                waypoints.append(waypoint)
            else:
                break
        
        return waypoints
    
    def run_experiment(self, controller_name, controller_params, duration=60):
        """
        Run a single experiment with camera visualization.
        
        Args:
            controller_name: Name of controller ('pure_pursuit', 'stanley', 'hybrid')
            controller_params: Dictionary of controller parameters
            duration: Experiment duration in seconds
        """
        print(f"\n{'='*60}")
        print(f"Running: {controller_name} with params {controller_params}")
        print(f"{'='*60}\n")
        
        # Create controller based on type
        if controller_name == 'pure_pursuit':
            controller = PurePursuitController(
                lookahead_distance=controller_params.get('lookahead_distance', 3.0),
                wheelbase=controller_params.get('wheelbase', 2.875)
            )
        elif controller_name == 'stanley':
            controller = StanleyController(
                k=controller_params.get('k', 1.0),
                wheelbase=controller_params.get('wheelbase', 2.875)
            )
        elif controller_name == 'hybrid':
            controller = HybridController(
                pp_lookahead=controller_params.get('pp_lookahead', 3.0),
                stanley_k=controller_params.get('stanley_k', 0.5),
                curvature_threshold=controller_params.get('curvature_threshold', 0.05),
                blend_zone=controller_params.get('blend_zone', 0.02),
                mode=controller_params.get('mode', 'adaptive')
            )
        else:
            raise ValueError(f"Unknown controller: {controller_name}")
        
        # Create visualization if enabled
        if self.enable_viz and self.viz is None:
            self.viz = VisualizationHUD()
        
        # Metrics storage
        metrics = {
            'lateral_errors': [],
            'heading_errors': [],
            'steering_angles': [],
            'speeds': [],
            'timestamps': []
        }
        
        # Experiment name
        param_str = '_'.join([f"{k}{v}" for k, v in sorted(controller_params.items())])
        experiment_name = f"{controller_name}_{param_str}"
        
        if self.viz:
            self.viz.update_metrics(experiment_name=experiment_name)
        
        # Run experiment
        start_time = time.time()
        step = 0
        
        while time.time() - start_time < duration:
            # Tick simulation
            self.world.tick()
            
            # Get waypoints ahead (real CARLA waypoints)
            waypoints = self.get_waypoints_ahead(num_waypoints=50, distance=2.0)
            
            # Get vehicle state
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # Compute control using YOUR controller's run_step method
            control = controller.run_step(self.vehicle, waypoints)
            
            # Apply control
            self.vehicle.apply_control(control)
            
            # Calculate metrics
            lateral_error = self._calculate_lateral_error(transform.location, waypoints)
            heading_error = self._calculate_heading_error(transform, waypoints)
            
            # Store metrics
            elapsed = time.time() - start_time
            metrics['lateral_errors'].append(lateral_error)
            metrics['heading_errors'].append(heading_error)
            metrics['steering_angles'].append(control.steer)
            metrics['speeds'].append(speed)
            metrics['timestamps'].append(elapsed)
            
            # Update visualization
            if self.viz:
                # Set camera image
                if self.camera_data is not None:
                    self.viz.set_camera_image(self.camera_data)
                
                # Get additional info for hybrid
                active_controller = controller_name
                blend_weight = 0.0
                curvature = 0.0
                
                if hasattr(controller, 'get_controller_info'):
                    info = controller.get_controller_info()
                    curvature = info.get('curvature', 0.0)
                    active_controller = info.get('active_controller', controller_name)
                    blend_weight = info.get('blend_weight', 0.0)
                
                self.viz.update_metrics(
                    experiment_name=experiment_name,
                    lateral_error=lateral_error,
                    heading_error=heading_error,
                    steering_angle=control.steer,
                    speed=speed,
                    time_elapsed=elapsed,
                    curvature=curvature,
                    active_controller=active_controller,
                    blend_weight=blend_weight
                )
                
                self.viz.render()
                
                # Check if user closed window
                if not self.viz.is_running():
                    print("\nVisualization closed by user. Stopping experiment.")
                    break
            
            step += 1
            
            # Progress
            if step % 100 == 0:
                print(f"  Step {step}, Time: {elapsed:.1f}s, "
                      f"Lat Error: {lateral_error:.3f}m, Speed: {speed:.1f}m/s")
        
        # Calculate summary statistics
        results = self._calculate_statistics(metrics, experiment_name)
        
        # Save results
        self._save_results(results, experiment_name)
        
        print(f"\n✓ Experiment complete!")
        print(f"  Mean Lateral Error: {results['mean_lateral_error']:.3f}m")
        print(f"  Steering Smoothness: {results['steering_smoothness']:.4f}")
        
        return results
    
    def _calculate_lateral_error(self, vehicle_location, waypoints):
        """Calculate lateral error to nearest waypoint."""
        if not waypoints:
            return 0.0
        
        min_dist = float('inf')
        
        for wp in waypoints[:10]:
            dist = vehicle_location.distance(wp.transform.location)
            if dist < min_dist:
                min_dist = dist
        
        return min_dist
    
    def _calculate_heading_error(self, vehicle_transform, waypoints):
        """Calculate heading error."""
        if not waypoints:
            return 0.0
        
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        # Get closest waypoint
        min_dist = float('inf')
        closest_wp = waypoints[0]
        
        for wp in waypoints[:10]:
            dist = vehicle_transform.location.distance(wp.transform.location)
            if dist < min_dist:
                min_dist = dist
                closest_wp = wp
        
        target_yaw = np.radians(closest_wp.transform.rotation.yaw)
        
        error = target_yaw - vehicle_yaw
        error = np.arctan2(np.sin(error), np.cos(error))
        
        return error
    
    def _calculate_statistics(self, metrics, name):
        """Calculate summary statistics."""
        lateral_errors = metrics['lateral_errors']
        steering_angles = metrics['steering_angles']
        
        # Steering smoothness (std of changes)
        steering_changes = np.diff(steering_angles)
        smoothness = np.std(steering_changes) if len(steering_changes) > 0 else 0.0
        
        results = {
            'experiment_name': name,
            'mean_lateral_error': float(np.mean(lateral_errors)),
            'max_lateral_error': float(np.max(lateral_errors)),
            'std_lateral_error': float(np.std(lateral_errors)),
            'mean_heading_error': float(np.mean([abs(x) for x in metrics['heading_errors']])),
            'max_heading_error': float(np.max([abs(x) for x in metrics['heading_errors']])),
            'steering_smoothness': float(smoothness),
            'mean_speed': float(np.mean(metrics['speeds'])),
            'duration': float(metrics['timestamps'][-1]) if metrics['timestamps'] else 0,
            'data_points': len(lateral_errors)
        }
        
        return results
    
    def _save_results(self, results, name):
        """Save results to JSON file."""
        filename = self.results_dir / f"{name}.json"
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"  Saved results to {filename}")
    
    def cleanup(self):
        """Cleanup CARLA actors and visualization."""
        print("\nCleaning up...")
        
        if self.camera:
            self.camera.destroy()
        if self.vehicle:
            self.vehicle.destroy()
        
        # Reset synchronous mode
        if self.world:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
        
        if self.viz:
            self.viz.close()
        
        print("✓ Cleanup complete")


def main():
    """Run all experiments with camera visualization."""
    runner = ExperimentRunnerWithCamera(enable_viz=True)
    
    try:
        # Setup
        runner.setup_carla()
        runner.spawn_vehicle()
        runner.setup_camera()
        
        # Wait for camera to start
        print("Waiting for camera...")
        time.sleep(2.0)
        
        # Define experiments matching your controller signatures
        experiments = [
            # Pure Pursuit experiments
            ('pure_pursuit', {'lookahead_distance': 2.0, 'wheelbase': 2.875}),
            ('pure_pursuit', {'lookahead_distance': 3.0, 'wheelbase': 2.875}),
            ('pure_pursuit', {'lookahead_distance': 5.0, 'wheelbase': 2.875}),
            
            # Stanley experiments
            ('stanley', {'k': 0.5, 'wheelbase': 2.875}),
            ('stanley', {'k': 1.0, 'wheelbase': 2.875}),
            ('stanley', {'k': 2.0, 'wheelbase': 2.875}),
            
            # Hybrid experiments
            ('hybrid', {'mode': 'switching', 'pp_lookahead': 3.0, 'stanley_k': 0.5}),
            ('hybrid', {'mode': 'blending', 'pp_lookahead': 3.0, 'stanley_k': 0.5}),
            ('hybrid', {'mode': 'adaptive', 'pp_lookahead': 3.0, 'stanley_k': 0.5}),
        ]
        
        # Run experiments
        all_results = []
        for i, (controller, params) in enumerate(experiments):
            print(f"\nExperiment {i+1}/{len(experiments)}")
            results = runner.run_experiment(controller, params, duration=60)
            all_results.append(results)
            
            # Brief pause between experiments
            time.sleep(2.0)
        
        print("\n" + "="*60)
        print("ALL EXPERIMENTS COMPLETE!")
        print("="*60)
        
        # Print summary
        print("\nSummary:")
        for result in all_results:
            print(f"  {result['experiment_name']}: "
                  f"Lateral Error = {result['mean_lateral_error']:.3f}m, "
                  f"Smoothness = {result['steering_smoothness']:.4f}")
    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        runner.cleanup()


if __name__ == "__main__":
    main()
