import cv2
import numpy as np
from picamera2 import Picamera2
from threading import Thread, Lock
from collections import deque
import time
import serial
import lgpio

class TennisBallRover:
    def __init__(self, resolution=(640, 480), process_scale=0.5, display=True, 
                     rotation=0, serial_port='/dev/serial0', baudrate=9600,
                     ir_sensor_pin=23,storage_ir_pin=24, ir_enabled=True,target_balls=3,
                     initial_forward_duration=4.0):
        """
        Tennis ball chasing rover for Raspberry Pi 5 with IR proximity sensor
        NOW CATCHES MULTIPLE BALLS!
        
        Args:
            resolution: Camera capture resolution (before rotation)
            process_scale: Scale factor for processing
            display: Enable/disable display window
            rotation: Camera rotation in degrees (0, 90, 180, 270)
            serial_port: Serial port for PIC communication
            baudrate: Serial communication speed
            ir_sensor_pin: GPIO pin for IR sensor (default: 23)
            ir_enabled: Enable/disable IR sensor
            target_balls: Number of balls to catch (default: 3)
            initial_forward_duration: Time to move forward at startup (default: 4.0)

        """
        self.mission_ended = False
        
        self.rotation = rotation
        self.capture_resolution = resolution
        
        self.search_burst_duration = 1.5      # 1 second of continuous forward
        self.search_burst_start = 0
        self.search_burst_active = False
        
        self.storage_ir_pin = storage_ir_pin

        # Initial startup movement
        self.initial_forward_duration = initial_forward_duration
        self.initial_forward_active = True
        self.initial_forward_start = None

        # Adjust display resolution based on rotation
        if rotation in [90, 270]:
            # Swap width and height for 90/270 degree rotations
            self.display_resolution = (resolution[1], resolution[0])
        else:
            self.display_resolution = resolution
        
        self.process_scale = process_scale
        self.process_size = (int(self.display_resolution[0] * process_scale), 
                            int(self.display_resolution[1] * process_scale))
        self.display_enabled = display
        
        # HSV color ranges (can be adjusted)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([40, 255, 255])
        
        # Pre-allocate arrays
        self.kernel = np.ones((5, 5), np.uint8)
        
        # Thread-safe variables
        self.lock = Lock()
        self.latest_frame = None
        self.result_frame = None
        self.detection_result = None
        self.running = False
        
        # Performance tracking
        self.fps_queue = deque(maxlen=30)
        self.last_time = time.time()
        
        # Tracking for temporal filtering
        self.detection_history = deque(maxlen=5)
        
        # Control parameters (using display resolution after rotation)
        self.center_x = self.display_resolution[0] // 2
        self.center_y = self.display_resolution[1] // 2
        self.dead_zone_x = 60  # Pixels from center to consider "centered"
        self.dead_zone_y = 60
        self.min_ball_size = 130  # Minimum radius to approach
        self.max_ball_size = 180  # Maximum radius (too close)
        
        # Search mode parameters
        self.last_detection_time = time.time()
        self.search_mode_delay = 2.0  # Seconds before entering search mode
        self.search_forward_interval = 0.4  # Send forward command every 0.5s
        self.last_search_command_time = 0
        self.in_search_mode = False
        
        # MULTI-BALL CATCHING PARAMETERS
        self.target_balls = target_balls
        self.balls_caught = 0
        self.mission_complete = False
        
        # Post-catch behavior state machine
        self.post_catch_state = None  # None, 'backing_up', 'rotating_search'
        self.post_catch_start_time = 0
        self.backup_duration = 5.0  # Seconds to back up after catching
        self.last_post_catch_command = 0
        self.post_catch_command_interval = 0.1
        self.rotating_search_interval = 0.2  # Send rotate command every 0.2s
        
        # IR Sensor setup
        self.ir_enabled = ir_enabled
        self.ir_sensor_pin = ir_sensor_pin
        self.ir_object_detected = False
        self.ir_gpio_handle = None
        
        # NEW: Multi-catch IR logic
        self.ir_catch_sequence_active = False
        self.ir_catch_count = 0
        self.ir_catch_max_count = 1  # Send 'C' 3 times
        self.ir_catch_delay = 3.0  # 3 seconds between each 'C'
        self.ir_last_catch_time = 0
        
        # Red line detection HSV range
        self.lower_red1 = np.array([0, 100, 100])    # Lower red range
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 100, 100])  # Upper red range
        self.upper_red2 = np.array([180, 255, 255])

        # Red line detection parameters
        self.red_line_detected = False
        self.red_line_y_position = 0  # Y coordinate of red line
        self.red_line_target_y = self.display_resolution[1] - 100  # Stop when line is this close to bottom
        self.returning_home = False  # New state for return-to-base mode
    
        # Final approach to red line
        self.red_line_final_approach = False
        self.red_line_approach_start = 0
        self.red_line_approach_duration = 5.0  # Move forward for 3 seconds
        
        if self.ir_enabled:
            try:
                # Open chip ONCE
                self.ir_gpio_handle = lgpio.gpiochip_open(0)

                # Claim both IR pins
                lgpio.gpio_claim_input(self.ir_gpio_handle, self.ir_sensor_pin)
                lgpio.gpio_claim_input(self.ir_gpio_handle, self.storage_ir_pin)

                print(f"IR1 initialized on GPIO {self.ir_sensor_pin}")
                print(f"Storage IR2 initialized on GPIO {self.storage_ir_pin}")

            except Exception as e:
                print("Warning: Could not initialize IR sensors:", e)
                self.ir_enabled = False

        # Serial communication
        self.serial_enabled = True
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
            time.sleep(2)  # Wait for serial to initialize
            print(f"Serial connection established on {serial_port}")
        except Exception as e:
            print(f"Warning: Could not open serial port: {e}")
            print("Running in simulation mode (no commands sent)")
            self.serial_enabled = False
        
        # Command tracking
        self.last_command = None
        self.last_command_time = 0
        self.command_interval = 0.1  # Minimum time between commands (seconds)
        
        # Initialize camera
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": resolution, "format": "RGB888"}
        )
        self.picam2.configure(config)
    
    def detect_red_line(self, frame):
        """
        Detect red line in frame
        Returns: (detected, y_position, x_center)
        """
        small_frame = cv2.resize(frame, self.process_size, 
                                 interpolation=cv2.INTER_LINEAR)
        
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for red (red wraps around in HSV, so two ranges)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            min_area = 1000 * (self.process_scale ** 2)
            
            if area > min_area:
                # Get bounding box
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Check if it's horizontal-ish (width > height)
                if w > h * 1.5:  # Line should be wider than tall
                    # Scale back to display resolution
                    scale_factor = 1.0 / self.process_scale
                    y_pos = int((y + h/2) * scale_factor)
                    x_center = int((x + w/2) * scale_factor)
                    return True, y_pos, x_center
        
        return False, 0, 0

    
    def read_ir_sensor(self):
        """Read IR sensor state (0 = object detected, 1 = no object)"""
        if not self.ir_enabled or self.ir_gpio_handle is None:
            return 1  # Return "no object" if sensor disabled
        
        try:
            value = lgpio.gpio_read(self.ir_gpio_handle, self.ir_sensor_pin)
            return value
        except Exception as e:
            print(f"IR sensor read error: {e}")
            return 1
    def read_storage_ir(self):
        """Read storage IR (1 = empty, 0 = balls full / blocked)"""
        if not self.ir_enabled or self.ir_gpio_handle is None:
            return 1

        try:
            return lgpio.gpio_read(self.ir_gpio_handle, self.storage_ir_pin)
        except:
            return 1

    def check_ir_should_be_active(self):
        """
        Check if IR sensor should be active
        Returns True only when in search mode
        """
        return self.in_search_mode
    
    def rotate_frame(self, frame):
        """Rotate frame based on rotation setting"""
        if self.rotation == 0:
            return frame
        elif self.rotation == 90:
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 180:
            return cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 270:
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        else:
            return frame
    
    def send_command(self, command):
        """Send command to PIC18 via serial"""
        current_time = time.time()
        
        # Rate limiting: don't spam the same command (except for catch sequence)
        if (command == self.last_command and 
            current_time - self.last_command_time < self.command_interval and
            command != 'C'):  # Allow 'C' to bypass rate limiting for catch sequence
            return
        
        if self.serial_enabled:
            try:
                self.ser.write(command.encode())
                print(f"Sent: {command}")
            except Exception as e:
                print(f"Serial error: {e}")
        else:
            print(f"[SIM] Command: {command}")
        
        self.last_command = command
        self.last_command_time = current_time
    
    def handle_ball_caught(self):
        """Handle the event when a ball is caught"""
        self.balls_caught += 1
        print(f"\nÃ°Å¸Å½Â¾ BALL CAUGHT! ({self.balls_caught}/{self.target_balls})")
        
        if self.balls_caught >= self.target_balls:
            print("Ã°Å¸Ââ€  MISSION COMPLETE! All balls collected!")
            print("Ã°Å¸â€â€ž Starting return to base - searching for red line...")
            self.mission_complete = True
            self.returning_home = True  # Start return sequence
            # DON'T send 'X' here - let the state machine handle it
        else:
            # Start post-catch sequence
            print(f"Ã°Å¸â€â€ž Starting post-catch sequence... Looking for ball {self.balls_caught + 1}")
            self.post_catch_state = 'backing_up'
            self.post_catch_start_time = time.time()
            self.in_search_mode = False
            self.detection_history.clear()
        
        # Reset IR catch sequence
        self.ir_catch_sequence_active = False
        self.ir_catch_count = 0
    
    def handle_post_catch_sequence(self, detected):
        """
        Execute post-catch behavior sequence:
        1. Back up for backup_duration seconds
        2. Keep rotating right (D) until a new ball is detected
        3. Resume normal tracking when ball found
        """
        current_time = time.time()
        elapsed = current_time - self.post_catch_start_time
        
        # Rate limit commands during post-catch
        if current_time - self.last_post_catch_command < self.post_catch_command_interval:
            return None
        
        self.last_post_catch_command = current_time
        
        if self.post_catch_state == 'backing_up':
            if elapsed < self.backup_duration:
                return 'S'  # Back up
            else:
                print("Ã°Å¸â€â€ž Rotating to search for next ball...")
                self.post_catch_state = 'rotating_search'
                self.post_catch_start_time = current_time
                return 'D'  # Start rotating right
        
        elif self.post_catch_state == 'rotating_search':
            # Check if ball is detected
            if detected:
                print("Ã¢Å“â€¦ Ball detected! Resuming normal tracking mode!")
                self.post_catch_state = None
                self.last_detection_time = time.time()  # Reset detection timer
                return None
            else:
                # Keep rotating until ball is found
                if current_time - self.post_catch_start_time >= self.rotating_search_interval:
                    self.post_catch_start_time = current_time
                    return 'D'  # Keep rotating right
        
        return None
    
    def handle_ir_catch_sequence(self):
        """
        Handle the IR catch sequence: send 'C' command 3 times with 3-second delays
        Returns the command to send, or None if sequence is complete
        """
        current_time = time.time()
        
        # Check if it's time to send the next 'C' command
        if current_time - self.ir_last_catch_time >= self.ir_catch_delay:
            self.ir_catch_count += 1
            self.ir_last_catch_time = current_time
            
            print(f"Ã°Å¸Å½Â¯ Sending catch command {self.ir_catch_count}/{self.ir_catch_max_count}")
            
            # Check if sequence is complete
            if self.ir_catch_count >= self.ir_catch_max_count:
                print("Ã¢Å“â€¦ Catch sequence complete!")
                self.handle_ball_caught()
                return 'C'  # Send final 'C'
            
            return 'C'
        
        return None  # Still waiting for next delay interval
    def handle_return_to_base(self, ball_detected, ball_x, ball_y, ball_radius):
        """
        Handle return to base sequence:
        1. Rotate slowly until red line detected
        2. Approach red line
        3. When close enough, move forward for 3 seconds
        4. Stop
        """
        current_time = time.time()
        
        # PRIORITY 1: Final approach in progress (move forward for 3 seconds)
        if self.red_line_final_approach:
            elapsed = current_time - self.red_line_approach_start
            if elapsed < self.red_line_approach_duration:
                return 'W'  # Keep moving forward
            else:
                print("Ã°Å¸ÂÂ FINAL APPROACH COMPLETE! Rover stopped at base.")
                self.red_line_final_approach = False
                self.returning_home = False 
                self.mission_ended = True 
                self.running = False
                return 'X'  # Stop
        
        # Get the latest frame from processing thread
        line_detected = self.red_line_detected
        line_y = self.red_line_y_position
        
        if not line_detected:
            # No red line yet - keep rotating slowly
            if current_time - self.last_search_command_time >= 0.15:
                self.last_search_command_time = current_time
                return 'A'  # Rotate right
            return None
        else:
            # Red line detected!
            if line_y >= self.red_line_target_y:
                # Start final approach!
                if not self.red_line_final_approach:
                    print("Ã°Å¸Å½Â¯ RED LINE REACHED! Starting 3-second final approach...")
                    self.red_line_final_approach = True
                    self.red_line_approach_start = current_time
                    return 'W'
            else:
                # Move forward toward the line
                return 'W'
                
    def calculate_command(self, detected, x, y, radius):
        """
        Calculate rover command based on ball position, IR sensor, and multi-ball logic
        
        Enhanced Logic:
        - Initial forward movement for specified duration at startup
        - Only check IR sensor when ball is in blind spot (centered and close)
        - Send 'C' command 3 times with 3-second delays when IR detects
        """
        if self.mission_ended:
            return 'X'
		
        current_time = time.time()
        

        
        # PRIORITY 0: Initial forward movement at startup
        if self.initial_forward_active:
            if self.initial_forward_start is None:
                self.initial_forward_start = current_time
                print(f"?? Starting initial forward movement for {self.initial_forward_duration} seconds...")
            
            elapsed = current_time - self.initial_forward_start
            if elapsed < self.initial_forward_duration:
                return 'W'  # Keep moving forward
            else:
                print("? Initial forward movement complete. Starting normal tracking mode.")
                self.initial_forward_active = False
                self.last_detection_time = current_time  # Reset detection timer
                # Continue to normal operation
        
        # STORAGE IR PRIORITY (higher than everything)
        storage_full = (self.read_storage_ir() == 0)
        
        if storage_full and not self.mission_complete:
            print("Ã°Å¸ÂÂ STORAGE FULL! Mission Complete - Starting return to base!")
            self.mission_complete = True
            self.returning_home = True  # Ã¢â€ Â This was missing!
            self.post_catch_state = None  # Cancel any post-catch sequence
            self.ir_catch_sequence_active = False  # Cancel any catch sequence
            print("Ã°Å¸â€Â Searching for red line to return home...")
            # Don't return 'X' here - let the return logic handle it
        
        if self.returning_home:
            return self.handle_return_to_base(detected, x, y, radius)

        # Update IR active status
        ir_should_be_active = self.check_ir_should_be_active()
        
 

        
        # PRIORITY 1: IR catch sequence in progress
        if self.ir_catch_sequence_active:
            catch_cmd = self.handle_ir_catch_sequence()
            if catch_cmd:
                return catch_cmd
            else:
                return None  # Waiting for next delay

        # PRIORITY 2: Post-catch sequence (backup and rotate until ball found)
        if self.post_catch_state is not None:
            return self.handle_post_catch_sequence(detected)
        
        # Check if ball was detected
        if detected:
            self.last_detection_time = current_time
            self.in_search_mode = False
        
        # PRIORITY 3: IR sensor detection - START CATCH SEQUENCE!
        # Only check IR when in SEARCH MODE
        if ir_should_be_active and self.ir_object_detected and not self.ir_catch_sequence_active:
            print("Ã°Å¸Å½Â¯ IR triggered during search mode - Starting catch sequence!")
            self.ir_catch_sequence_active = True
            self.ir_catch_count = 0
            self.ir_last_catch_time = current_time
            return 'C'  # Send first 'C' immediately
        
        # Check if we should enter search mode
        time_since_detection = current_time - self.last_detection_time
        
        if not detected and time_since_detection > self.search_mode_delay:
            # FIRST TIME ENTERING SEARCH MODE
            if not self.in_search_mode:
                print("Ã°Å¸â€Â Entering search mode...")
                self.in_search_mode = True
                self.search_burst_active = True
                self.search_burst_start = current_time
                return 'W'   # Start immediate forward burst

            # HANDLE THE 1 SECOND FORWARD BURST
            if self.search_burst_active:
                if current_time - self.search_burst_start < self.search_burst_duration:
                    return 'W'   # Keep moving forward continuously
                else:
                    self.search_burst_active = False   # End burst

            # AFTER BURST Ã¢â€ â€™ NORMAL SLOW SEARCH
            if current_time - self.last_search_command_time >= self.search_forward_interval:
                self.last_search_command_time = current_time
                return 'W'

            return None
        
        # Normal tracking mode
        if not detected:
            self.in_search_mode = False
            return None  # No command when ball not found (and not in search mode yet)
        
        # Calculate position relative to center
        offset_x = x - self.center_x
        offset_y = y - self.center_y
        
        # Determine command based on ball position and size
        if abs(offset_x) > self.dead_zone_x:
            # Ball is off-center horizontally
            if offset_x < 0:
                return 'A'  # Turn left
            else:
                return 'D'  # Turn right
        else:
            # Ball is centered horizontally
            if radius < self.min_ball_size:
                return 'W'  # Move forward (ball too far)
            elif radius > self.max_ball_size:
                # Ball is too close - should be in blind spot now
                # Let IR handle it, stop movement
                return None
            else:
                return None  # Good distance, stop
        
        return None
    
    def detect_tennis_ball(self, frame):
        """
        Optimized detection with temporal filtering
        Frame should already be rotated before calling this
        """
        # Resize for faster processing
        small_frame = cv2.resize(frame, self.process_size, 
                                 interpolation=cv2.INTER_LINEAR)
        
        # Convert to HSV
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                       cv2.CHAIN_APPROX_SIMPLE)
        
        ball_detected = False
        ball_x, ball_y, ball_radius = 0, 0, 0
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            min_area = 500 * (self.process_scale ** 2)
            
            if area > min_area:
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                
                perimeter = cv2.arcLength(largest_contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter ** 2)
                    
                    min_radius = 10 * self.process_scale
                    if circularity > 0.6 and radius > min_radius:
                        # Scale coordinates back to rotated frame resolution
                        scale_factor = 1.0 / self.process_scale
                        ball_x = int(x * scale_factor)
                        ball_y = int(y * scale_factor)
                        ball_radius = int(radius * scale_factor)
                        ball_detected = True
        
        # Temporal filtering
        detection = (ball_detected, ball_x, ball_y, ball_radius)
        self.detection_history.append(detection)
        
        if len(self.detection_history) >= 3:
            recent_detections = [d[0] for d in list(self.detection_history)[-3:]]
            if sum(recent_detections) >= 2:
                ball_detected = detection[0]
                ball_x, ball_y, ball_radius = detection[1], detection[2], detection[3]
            else:
                ball_detected = False
        
        return ball_detected, ball_x, ball_y, ball_radius
    
    def draw_detection(self, frame, detected, x, y, radius, command):
        """Draw detection overlay and command on frame (frame should already be rotated)"""
        # Draw center crosshair
        cv2.line(frame, (self.center_x - 50, self.center_y), 
                (self.center_x + 50, self.center_y), (255, 255, 255), 1)
        cv2.line(frame, (self.center_x, self.center_y - 50), 
                (self.center_x, self.center_y + 50), (255, 255, 255), 1)
        
        # Draw dead zones
        cv2.rectangle(frame, 
                     (self.center_x - self.dead_zone_x, self.center_y - self.dead_zone_y),
                     (self.center_x + self.dead_zone_x, self.center_y + self.dead_zone_y),
                     (100, 100, 100), 1)
        
        if detected:
            # Draw ball detection
            cv2.circle(frame, (x, y), radius, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            
            # Draw line from center to ball
            cv2.line(frame, (self.center_x, self.center_y), (x, y), (255, 0, 255), 2)
            
            # Ball info
            label = f"Ball: ({x}, {y}) R:{radius}"
            cv2.putText(frame, label, (x - 80, y - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Size indicator
            if radius < self.min_ball_size:
                size_status = "TOO FAR"
                color = (0, 165, 255)  # Orange
            elif radius > self.max_ball_size:
                size_status = "TOO CLOSE"
                color = (0, 0, 255)
            else:
                size_status = "GOOD DISTANCE"
                color = (0, 255, 0)  # Green
            
            cv2.putText(frame, size_status, (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # ===== BALL COUNTER DISPLAY (LARGE AND PROMINENT) =====
        counter_text = f"BALLS: {self.balls_caught}/{self.target_balls}"
        if self.mission_complete:
            counter_color = (0, 255, 0)  # Green when complete
        else:
            counter_color = (0, 255, 255)  # Yellow during mission
        
        cv2.putText(frame, counter_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, counter_color, 3)
        # Display red line detection (if returning home)
        # In draw_detection method, update the red line visualization section:

        # Display red line detection (if returning home)
        if self.returning_home:
            line_detected = self.red_line_detected
            line_y = self.red_line_y_position
            
            if line_detected:
                # Draw red line indicator
                cv2.line(frame, (0, line_y), (frame.shape[1], line_y), 
                        (0, 0, 255), 3)
                # Draw target line
                cv2.line(frame, (0, self.red_line_target_y), (frame.shape[1], self.red_line_target_y), 
                        (0, 255, 0), 2)
                
                distance = self.red_line_target_y - line_y
                cv2.putText(frame, f"Line distance: {distance}px", 
                           (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 0, 255), 2)
        # Display current status
        
        if self.initial_forward_active:
            if self.initial_forward_start:
                elapsed = time.time() - self.initial_forward_start
                remaining = self.initial_forward_duration - elapsed
                status = f"STARTUP: Moving forward {remaining:.1f}s"
                color = (255, 255, 0)  # Yellow
            else:
                status = "INITIALIZING..."
                color = (255, 255, 0)
        elif self.mission_complete:
            if self.red_line_final_approach:
                elapsed = time.time() - self.red_line_approach_start
                remaining = self.red_line_approach_duration - elapsed
                status = f"FINAL APPROACH: {remaining:.1f}s"
                color = (0, 255, 0)  # Green
            elif self.returning_home and not self.red_line_detected:
                status = "RETURNING HOME - SEARCHING RED LINE"
                color = (0, 255, 255)  # Cyan
            elif self.returning_home and self.red_line_detected:
                status = "APPROACHING RED LINE"
                color = (0, 0, 255)  # Red
            else:
                status = "MISSION COMPLETE!"
                color = (0, 255, 0)
        elif self.ir_catch_sequence_active:
            status = f"CATCHING... ({self.ir_catch_count}/{self.ir_catch_max_count})"
            color = (0, 255, 0)  # Green
        elif self.post_catch_state == 'backing_up':
            status = "BACKING UP..."
            color = (255, 165, 0)  # Orange
        elif self.post_catch_state == 'rotating_search':
            status = "ROTATING - SEARCHING..."
            color = (255, 0, 255)  # Magenta
        elif self.in_search_mode:
            status = f"SEARCHING (Ball {self.balls_caught + 1})"
            color = (0, 255, 255)  # Cyan
        else:
            status = "TRACKING" if detected else "SCANNING..."
            color = (0, 255, 0) if detected else (0, 165, 255)
        
        cv2.putText(frame, status, (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Display IR sensor status - ONLY when in search mode
        if self.ir_enabled:
            if self.in_search_mode:
                ir_status = "IR: DETECTING!" if self.ir_object_detected else "IR: ACTIVE (searching...)"
                ir_color = (0, 0, 255) if self.ir_object_detected else (0, 255, 255)
            else:
                ir_status = "IR: Standby (not searching)"
                ir_color = (128, 128, 128)
            
            cv2.putText(frame, ir_status, (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, ir_color, 2)
        
        # Display current command
        if command == 'C':
            cmd_text = "Command: CATCH!"
            cmd_color = (0, 255, 0)
        elif command == 'X':
            cmd_text = "Command: STOP"
            cmd_color = (0, 0, 255)
        else:
            cmd_text = f"Command: {command if command else 'STOP'}"
            cmd_color = (0, 255, 255) if command else (128, 128, 128)
        
        cv2.putText(frame, cmd_text, (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, cmd_color, 2)
        
        # Display time since last detection (only if not in post-catch)
        if self.post_catch_state is None and not self.ir_catch_sequence_active:
            time_since = time.time() - self.last_detection_time
            cv2.putText(frame, f"Lost: {time_since:.1f}s", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display FPS
        if self.fps_queue:
            avg_fps = 1.0 / (sum(self.fps_queue) / len(self.fps_queue))
            cv2.putText(frame, f"FPS: {avg_fps:.1f}", (10, frame.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame
    
    def ir_sensor_thread(self):
        """Separate thread for IR sensor monitoring"""
        while self.running:
            if self.ir_enabled:
                ir_value = self.read_ir_sensor()
                with self.lock:
                    self.ir_object_detected = (ir_value == 0)
            
            time.sleep(0.05)  # Check IR sensor at 20Hz
    
    def capture_thread(self):
        """Separate thread for camera capture"""
        while self.running:
            frame = self.picam2.capture_array()
            # Apply rotation immediately after capture
            if self.rotation != 0:
                frame = self.rotate_frame(frame)
            with self.lock:
                self.latest_frame = frame.copy()
    
    def processing_thread(self):
        """Separate thread for detection and control"""
        while self.running:
            with self.lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
            
            # Frame is already rotated
            
            # If returning home, detect red line instead of tennis ball
            if self.returning_home:
                line_detected, line_y, line_x = self.detect_red_line(frame)
                with self.lock:
                    self.red_line_detected = line_detected
                    self.red_line_y_position = line_y
                
                # Calculate command for return to base
                command = self.calculate_command(False, 0, 0, 0)  # No ball detection
                
                # Send command
                if command:
                    self.send_command(command)
                
                # Draw red line detection
                with self.lock:
                    if self.display_enabled:
                        self.result_frame = self.draw_detection(frame, False, 0, 0, 0, command)
            else:
                # Normal tennis ball detection
                detected, x, y, radius = self.detect_tennis_ball(frame)
                
                # Calculate command (includes multi-ball logic)
                command = self.calculate_command(detected, x, y, radius)
                
                # Send command to rover
                if command:
                    self.send_command(command)
                
                # Store result
                with self.lock:
                    self.detection_result = (detected, x, y, radius, command)
                    if self.display_enabled:
                        self.result_frame = self.draw_detection(frame, detected, x, y, 
                                                                radius, command)
        
        # Update FPS
        current_time = time.time()
        self.fps_queue.append(current_time - self.last_time)
        self.last_time = current_time
    
    def run(self):
        """Run the rover controller"""
        print("=" * 60)
        print(f"Tennis Ball Chasing Rover - {self.target_balls} Ball Collection Mission")
        print("=" * 60)
        print("Commands: W=Forward, A=Left, S=Backward, D=Right, X=Stop, C=Catch")
        print(f"Target: Catch {self.target_balls} tennis balls")
        print(f"Capture resolution: {self.capture_resolution[0]}x{self.capture_resolution[1]}")
        print(f"Display resolution: {self.display_resolution[0]}x{self.display_resolution[1]}")
        print(f"Camera rotation: {self.rotation} degrees")
        print(f"Serial: {'ENABLED' if self.serial_enabled else 'DISABLED (Simulation)'}")
        print(f"IR Sensor: {'ENABLED on GPIO {}'.format(self.ir_sensor_pin) if self.ir_enabled else 'DISABLED'}")
        print(f"IR Logic: Activates only in SEARCH MODE")
        print(f"Catch Sequence: 3x 'C' commands with 3-second delays")
        print(f"Search mode delay: {self.search_mode_delay}s")
        print(f"Post-catch: Backup {self.backup_duration}s, then rotate until ball detected")
        print("Press 'q' to quit")
        print("Press 'c' to capture HSV values at center")
        print("Press 'm' to toggle manual control mode")
        print("Press 'r' to reset ball counter")
        print("=" * 60)
        
        self.running = True
        self.picam2.start()
        
        # Start threads
        capture_thread = Thread(target=self.capture_thread, daemon=True)
        process_thread = Thread(target=self.processing_thread, daemon=True)
        ir_thread = Thread(target=self.ir_sensor_thread, daemon=True)
        
        capture_thread.start()
        process_thread.start()
        ir_thread.start()
        
        manual_mode = False
        
        try:
            while self.running:
                if self.display_enabled:
                    with self.lock:
                        if self.result_frame is not None:
                            display_frame = self.result_frame.copy()
                            
                            # Add manual mode indicator
                            if manual_mode:
                                cv2.putText(display_frame, "MANUAL MODE", 
                                          (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 
                                          0.6, (0, 255, 255), 2)
                            
                            cv2.imshow("Tennis Ball Rover", display_frame)
                
                # Handle keypresses
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    self.running = False
                    break
                elif key == ord('c'):
                    with self.lock:
                        if self.latest_frame is not None:
                            hsv = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2HSV)
                            h, w = hsv.shape[:2]
                            center_hsv = hsv[h//2, w//2]
                            print(f"Center HSV values: {center_hsv}")
                elif key == ord('r'):
                    # Reset ball counter
                    with self.lock:
                        self.balls_caught = 0
                        self.mission_complete = False
                        self.post_catch_state = None
                        self.ir_catch_sequence_active = False
                        self.ir_catch_count = 0
                        print("\nÃ°Å¸â€â€ž Ball counter reset! Mission restarted.")
                elif key == ord('m'):
                    manual_mode = not manual_mode
                    print(f"Manual mode: {'ON' if manual_mode else 'OFF'}")
                elif manual_mode:
                    # Manual control
                    if key == ord('w'):
                        self.send_command('W')
                    elif key == ord('a'):
                        self.send_command('A')
                    elif key == ord('s'):
                        self.send_command('S')
                    elif key == ord('d'):
                        self.send_command('D')
                    elif key == ord('x'):
                        self.send_command('X')
                    elif key == ord('b'):
                        self.send_command('C')
                
                time.sleep(0.001)
        
        finally:
            self.running = False
            capture_thread.join(timeout=1.0)
            process_thread.join(timeout=1.0)
            ir_thread.join(timeout=1.0)
            self.picam2.stop()
            if self.serial_enabled:
                self.ser.close()
            if self.ir_enabled and self.ir_gpio_handle is not None:
                lgpio.gpiochip_close(self.ir_gpio_handle)
            cv2.destroyAllWindows()
            print(f"\nÃ°Å¸ÂÂ Mission ended: {self.balls_caught}/{self.target_balls} balls caught")


def main():
    """Main function with configuration"""
    # Configuration
    ENABLE_DISPLAY = True      # Set to False for headless operation
    PROCESS_SCALE = 0.6        # Processing resolution scale
    CAMERA_ROTATION = 270      # Camera rotation: 0, 90, 180, or 270
    SERIAL_PORT = '/dev/serial0'  # Serial port for PIC
    BAUDRATE = 9600            # Serial communication speed
    IR_SENSOR_PIN = 23         # GPIO pin for IR sensor
    IR_ENABLED = True          # Enable/disable IR sensor
    TARGET_BALLS = 100         # Number of balls to catch
    INITIAL_FORWARD_TIME = 1.0 # Seconds to move forward at startup
    
    # Create and run rover
    rover = TennisBallRover(
        resolution=(640, 640),
        process_scale=PROCESS_SCALE,
        display=ENABLE_DISPLAY,
        rotation=CAMERA_ROTATION,
        serial_port=SERIAL_PORT,
        baudrate=BAUDRATE,
        ir_sensor_pin=IR_SENSOR_PIN,
        ir_enabled=IR_ENABLED,
        target_balls=TARGET_BALLS,
        initial_forward_duration=INITIAL_FORWARD_TIME
    )
    
    rover.run()


if __name__ == "__main__":
    main()