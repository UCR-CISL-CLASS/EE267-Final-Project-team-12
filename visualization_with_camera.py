#!/usr/bin/env python3
"""
Enhanced CARLA-Style Visualization with Camera View
Combines HUD overlay with actual CARLA camera feed
"""

import pygame
import numpy as np
import math
from collections import deque
from datetime import timedelta

class VisualizationHUD:
    """Enhanced CARLA-style HUD with camera view integration."""
    
    def __init__(self, width=1280, height=720):
        """Initialize the enhanced visualization HUD."""
        pygame.init()
        self.width = width
        self.height = height
        self.display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Lane Keeping Controller - Real-Time Monitoring")
        
        # Fonts (CARLA style)
        font_name = 'ubuntumono'
        try:
            self.font_mono = pygame.font.Font(pygame.font.match_font(font_name), 14)
        except:
            self.font_mono = pygame.font.Font(pygame.font.get_default_font(), 14)
        
        try:
            self.font_large = pygame.font.Font(pygame.font.match_font(font_name), 20)
        except:
            self.font_large = pygame.font.Font(pygame.font.get_default_font(), 20)
        
        # Colors (CARLA style)
        self.colors = {
            'bg': (70, 70, 70),  # Gray background when no camera
            'panel': (0, 0, 0),
            'text': (255, 255, 255),
            'bar_border': (255, 255, 255),
            'bar_fill': (255, 255, 255),
            'graph_line': (255, 136, 0),  # CARLA orange
            'good': (0, 255, 0),
            'warning': (255, 255, 0),
            'danger': (255, 0, 0),
        }
        
        # Data buffers
        self.lateral_error_history = deque(maxlen=200)
        self.heading_error_history = deque(maxlen=200)
        self.steering_history = deque(maxlen=200)
        self.collision_history = deque(maxlen=200)
        
        # Current metrics
        self.current_metrics = {}
        self.experiment_name = "Initializing..."
        self.start_time = 0
        self.elapsed_time = 0
        
        # Camera image
        self.camera_image = None
        self.camera_surface = None
        
        # Clock
        self.clock = pygame.time.Clock()
        self.fps = 30
        self.running = True
        
        # Panel dimensions
        self.panel_width = 280
        self.panel_alpha = 100
        
        # Graph dimensions
        self.graph_height = 200
        self.graph_margin = 20
    
    def set_camera_image(self, image_array):
        """
        Set the camera image to display.
        
        Args:
            image_array: numpy array of shape (height, width, 3) or (height, width, 4)
                        with values 0-255, RGB or RGBA format
        """
        if image_array is not None:
            # Convert to pygame surface
            if len(image_array.shape) == 2:
                # Grayscale
                image_array = np.stack([image_array] * 3, axis=-1)
            
            # Ensure RGB format
            if image_array.shape[2] == 4:
                image_array = image_array[:, :, :3]
            
            # Create surface
            self.camera_surface = pygame.surfarray.make_surface(
                np.transpose(image_array, (1, 0, 2))
            )
    
    def update_metrics(self, **metrics):
        """Update current metrics."""
        self.current_metrics = metrics
        
        # Update histories
        if 'lateral_error' in metrics:
            self.lateral_error_history.append(abs(metrics['lateral_error']))
        if 'heading_error' in metrics:
            self.heading_error_history.append(abs(metrics['heading_error']))
        if 'steering_angle' in metrics:
            self.steering_history.append(metrics['steering_angle'])
        
        # Collision indicator
        collision_val = 0
        if 'lateral_error' in metrics and abs(metrics['lateral_error']) > 2.0:
            collision_val = 1.0
        self.collision_history.append(collision_val)
        
        # Update time
        if 'time_elapsed' in metrics:
            self.elapsed_time = metrics['time_elapsed']
        
        # Update experiment name
        if 'experiment_name' in metrics:
            self.experiment_name = metrics['experiment_name']
    
    def render(self):
        """Render the HUD with camera view."""
        # Draw camera view or background
        if self.camera_surface is not None:
            # Scale camera to fill screen
            scaled_surface = pygame.transform.scale(self.camera_surface, 
                                                    (self.width, self.height))
            self.display.blit(scaled_surface, (0, 0))
        else:
            # Gray background if no camera
            self.display.fill(self.colors['bg'])
        
        # Render info panel overlay (left side)
        self._render_info_panel()
        
        # Render error graph (top right)
        self._render_error_graph()
        
        # Update display
        pygame.display.flip()
        self.clock.tick(self.fps)
        
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_F1:
                    # Toggle HUD (could implement)
                    pass
    
    def _render_info_panel(self):
        """Render the semi-transparent info panel (CARLA style)."""
        # Create semi-transparent surface
        info_surface = pygame.Surface((self.panel_width, self.height))
        info_surface.set_alpha(self.panel_alpha)
        info_surface.fill(self.colors['panel'])
        self.display.blit(info_surface, (0, 0))
        
        # Build info text
        v_offset = 4
        bar_h_offset = 160
        bar_width = 106
        
        info_lines = []
        
        # FPS
        info_lines.append('Client:  % 16.0f FPS' % self.clock.get_fps())
        info_lines.append('')
        
        # Experiment info
        info_lines.append('Experiment: %s' % self.experiment_name[:18])
        info_lines.append('Time: %s' % str(timedelta(seconds=int(self.elapsed_time))))
        info_lines.append('')
        
        # Speed
        speed = self.current_metrics.get('speed', 0)
        info_lines.append('Speed:   % 15.1f m/s' % speed)
        info_lines.append('Speed:   % 15.0f km/h' % (3.6 * speed))
        info_lines.append('')
        
        # Lateral error with status
        lateral_error = self.current_metrics.get('lateral_error', 0)
        info_lines.append('Lateral Error:')
        info_lines.append('  % 18.3f m' % lateral_error)
        
        if abs(lateral_error) < 0.5:
            status = "EXCELLENT"
        elif abs(lateral_error) < 1.0:
            status = "GOOD"
        else:
            status = "HIGH ERROR"
        info_lines.append('  Status: %s' % status)
        info_lines.append('')
        
        # Heading error
        heading_error = self.current_metrics.get('heading_error', 0)
        info_lines.append('Heading Error:')
        info_lines.append(u'  % 16.1f\N{DEGREE SIGN}' % math.degrees(heading_error))
        info_lines.append('')
        
        # Steering with bar
        steering = self.current_metrics.get('steering_angle', 0)
        info_lines.append(('Steering:', steering, -1.0, 1.0))
        
        # Curvature
        curvature = self.current_metrics.get('curvature', 0)
        info_lines.append('Curvature: %.4f' % curvature)
        info_lines.append('')
        
        # Controller
        controller = self.current_metrics.get('active_controller', 'Unknown')
        info_lines.append('Active Controller:')
        info_lines.append('  %s' % controller)
        
        # Blend weight
        blend_weight = self.current_metrics.get('blend_weight', 0)
        if blend_weight > 0:
            info_lines.append('')
            info_lines.append('Hybrid Blend:')
            info_lines.append(('  PP <-> Stanley', blend_weight, 0.0, 1.0))
        
        # Collision indicator
        info_lines.append('')
        info_lines.append('Collision Indicator:')
        info_lines.append(list(self.collision_history))
        
        # Statistics
        if len(self.lateral_error_history) > 0:
            info_lines.append('')
            info_lines.append('Statistics:')
            avg_error = sum(self.lateral_error_history) / len(self.lateral_error_history)
            max_error = max(self.lateral_error_history)
            info_lines.append('  Avg Error: %.3f m' % avg_error)
            info_lines.append('  Max Error: %.3f m' % max_error)
        
        # Render lines
        for item in info_lines:
            if v_offset + 18 > self.height:
                break
            
            if isinstance(item, list):
                # Graph (collision)
                if len(item) > 1:
                    points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) 
                             for x, y in enumerate(item)]
                    pygame.draw.lines(self.display, self.colors['graph_line'], 
                                    False, points, 2)
                v_offset += 18
                
            elif isinstance(item, tuple):
                # Bar display
                label, value, min_val, max_val = item
                
                # Border
                rect_border = pygame.Rect((bar_h_offset, v_offset + 8), 
                                        (bar_width, 6))
                pygame.draw.rect(self.display, self.colors['bar_border'], 
                               rect_border, 1)
                
                # Fill
                f = (value - min_val) / (max_val - min_val)
                f = max(0, min(1, f))
                
                if min_val < 0.0:
                    # Centered bar for steering
                    bar_pos = bar_h_offset + f * (bar_width - 6)
                    rect = pygame.Rect((bar_pos, v_offset + 8), (6, 6))
                else:
                    rect = pygame.Rect((bar_h_offset, v_offset + 8), 
                                     (f * bar_width, 6))
                
                pygame.draw.rect(self.display, self.colors['bar_fill'], rect)
                
                # Label
                surface = self.font_mono.render(label, True, self.colors['text'])
                self.display.blit(surface, (8, v_offset))
                v_offset += 18
                
            else:
                # Text
                if item:
                    # Color based on status
                    color = self.colors['text']
                    if 'EXCELLENT' in str(item):
                        color = self.colors['good']
                    elif 'GOOD' in str(item):
                        color = self.colors['warning']
                    elif 'HIGH ERROR' in str(item):
                        color = self.colors['danger']
                    
                    surface = self.font_mono.render(str(item), True, color)
                    self.display.blit(surface, (8, v_offset))
                v_offset += 18
    
    def _render_error_graph(self):
        """Render error history graph (top right, overlay style)."""
        if len(self.lateral_error_history) < 2:
            return
        
        # Graph position (top right corner)
        graph_width = 600
        graph_x = self.width - graph_width - self.graph_margin
        graph_y = self.graph_margin
        
        # Semi-transparent background
        graph_surface = pygame.Surface((graph_width, self.graph_height))
        graph_surface.set_alpha(80)
        graph_surface.fill((0, 0, 0))
        self.display.blit(graph_surface, (graph_x, graph_y))
        
        # Border
        pygame.draw.rect(self.display, self.colors['bar_border'],
                        (graph_x, graph_y, graph_width, self.graph_height), 1)
        
        # Title
        title = self.font_large.render("Lateral Error History", True, 
                                      self.colors['text'])
        self.display.blit(title, (graph_x + 10, graph_y - 25))
        
        # Plot data
        errors = list(self.lateral_error_history)
        max_error = max(max(errors), 2.0)
        
        points = []
        for i, error in enumerate(errors):
            x = graph_x + (i / len(errors)) * graph_width
            y = graph_y + self.graph_height - (error / max_error) * self.graph_height
            points.append((x, y))
        
        if len(points) > 1:
            pygame.draw.lines(self.display, self.colors['graph_line'], 
                            False, points, 2)
        
        # Reference lines
        # 0.5m (good)
        y_05 = graph_y + self.graph_height - (0.5 / max_error) * self.graph_height
        pygame.draw.line(self.display, self.colors['good'],
                        (graph_x, y_05), (graph_x + graph_width, y_05), 1)
        label = self.font_mono.render("0.5m", True, self.colors['good'])
        self.display.blit(label, (graph_x + graph_width - 40, y_05 - 15))
        
        # 1.0m (warning)
        if 1.0 <= max_error:
            y_10 = graph_y + self.graph_height - (1.0 / max_error) * self.graph_height
            pygame.draw.line(self.display, self.colors['warning'],
                            (graph_x, y_10), (graph_x + graph_width, y_10), 1)
            label = self.font_mono.render("1.0m", True, self.colors['warning'])
            self.display.blit(label, (graph_x + graph_width - 40, y_10 - 15))
        
        # Scale
        max_label = self.font_mono.render("%.1fm" % max_error, True, 
                                         self.colors['text'])
        self.display.blit(max_label, (graph_x + 5, graph_y + 5))
    
    def is_running(self):
        """Check if visualization should continue."""
        return self.running
    
    def close(self):
        """Close the visualization."""
        pygame.quit()


def test_visualization():
    """Test with simulated data and gradient background."""
    import time
    import random
    
    viz = VisualizationHUD()
    
    print("Testing Enhanced CARLA-style visualization...")
    print("Simulating camera view with gradient background")
    print("Press ESC to close")
    
    t = 0
    
    while viz.is_running() and t < 60:
        # Create fake camera image (gradient)
        height, width = 720, 1280
        camera_img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Create sky-to-ground gradient
        for y in range(height):
            # Sky (blue) to ground (gray)
            sky_amount = 1.0 - (y / height)
            r = int(100 * (1 - sky_amount) + 135 * sky_amount)
            g = int(100 * (1 - sky_amount) + 206 * sky_amount)
            b = int(100 * (1 - sky_amount) + 235 * sky_amount)
            camera_img[y, :] = [r, g, b]
        
        # Add some "road" texture
        road_y = int(height * 0.6)
        camera_img[road_y:, :] = [80, 80, 80]
        
        # Add lane lines
        center_x = width // 2
        for y in range(road_y, height, 40):
            cv2_line_start = max(0, center_x - 5)
            cv2_line_end = min(width, center_x + 5)
            camera_img[y:y+20, cv2_line_start:cv2_line_end] = [255, 255, 0]
        
        viz.set_camera_image(camera_img)
        
        # Simulate metrics
        lateral_error = 0.5 + 0.3 * math.sin(t * 0.5) + random.uniform(-0.1, 0.1)
        heading_error = 0.1 * math.sin(t * 0.3)
        steering = 0.3 * math.sin(t * 0.5)
        speed = 8.0 + random.uniform(-0.5, 0.5)
        curvature = 0.02 * math.sin(t * 0.2)
        
        viz.update_metrics(
            experiment_name="Pure Pursuit Ld=3.0",
            lateral_error=lateral_error,
            heading_error=heading_error,
            steering_angle=steering,
            speed=speed,
            time_elapsed=t,
            curvature=curvature,
            active_controller="Pure Pursuit",
            blend_weight=0.0
        )
        
        viz.render()
        time.sleep(1/30)
        t += 1/30
    
    viz.close()
    print("Test complete!")


if __name__ == "__main__":
    test_visualization()
