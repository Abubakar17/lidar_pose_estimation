#!/usr/bin/env python3
"""
LSS Smart Servo conveyor control module.
Handles serial communication with Lynxmotion LSS servo.
"""

import serial
import time


class ConveyorControl:
    """Control LSS servo for conveyor belt movement"""
    
    def __init__(self, port='/dev/ttyUSB1', servo_id=5, degrees_per_cm=22.0):
        """
        Initialize conveyor control.
        
        Args:
            port: Serial port (default: /dev/ttyUSB1)
            servo_id: LSS servo ID (default: 5)
            degrees_per_cm: Calibration value (default: 22.0)
        """
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.id = servo_id
            self.degrees_per_cm = degrees_per_cm
            time.sleep(0.5)
            
            # Test connection
            response = self.query("QID")
            if response:
                print(f"Conveyor connected: servo ID {self.id}, {self.degrees_per_cm}°/cm")
                print(f"  Test response: {response}")
            else:
                print(f"WARNING: Conveyor at {port} not responding!")
        except Exception as e:
            print(f"ERROR: Failed to initialize conveyor: {e}")
            raise
    
    def send_command(self, command):
        """Send command to servo without expecting response"""
        try:
            cmd = f"#{self.id}{command}\r"
            self.ser.write(cmd.encode())
            time.sleep(0.02)
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    
    def query(self, command):
        """Send query and read response"""
        try:
            self.ser.reset_input_buffer()
            cmd = f"#{self.id}{command}\r"
            self.ser.write(cmd.encode())
            time.sleep(0.1)
            response = self.ser.readline().decode().strip()
            return response if response else None
        except Exception as e:
            print(f"Error querying '{command}': {e}")
            return None
    
    def move_cm(self, cm):
        """
        Move conveyor belt by specified distance.
        
        Args:
            cm: Distance to move in centimeters
            
        Returns:
            bool: True if successful
        """
        try:
            degrees = cm * self.degrees_per_cm
            current = self.get_position()
            
            if current is None:
                print("Warning: Could not get current position, attempting move anyway...")
                self.send_command(f"D{int(degrees * 10)}")
                time.sleep(1.0)
                return True
            
            new_pos = current + degrees
            success = self.send_command(f"D{int(new_pos * 10)}")
            
            if success:
                time.sleep(1.0)
                return True
            return False
            
        except Exception as e:
            print(f"Error in move_cm: {e}")
            return False
    
    def get_position(self):
        """Get current servo position in degrees"""
        response = self.query("QD")
        if response:
            try:
                pos_str = response.split('QD')[1]
                return int(pos_str) / 10.0
            except (IndexError, ValueError) as e:
                print(f"Error parsing position '{response}': {e}")
                return None
        return None
    
    def close(self):
        """Close serial connection"""
        try:
            self.ser.close()
        except:
            pass
