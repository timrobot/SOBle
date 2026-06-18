import time
import serial
import argparse

# --- CONFIGURATION ---
# PORT: Update this to match your USB-to-RS485/TTL adapter (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
BAUDRATE = 1000000  # FeeTech factory default is usually 1,000,000 or 115,200
CURRENT_ID = 254    # Factory default ID is usually 1 (or use 254 as a broadcast ID if only ONE motor is connected)

# FeeTech Register Addresses
REG_ID = 5          # ID register address
REG_LOCK = 55       # EEPROM Lock register address (0: unlocked, 1: locked)

def checksum(packet):
  """Calculate FeeTech checksum: ~ (ID + Length + Cmd + Params) & 0xFF"""
  return (~sum(packet[2:])) & 0xFF

def write_reg_byte(ser, motor_id, register, value):
  """Writes a 1-byte value to a specific register."""
  # Packet format: [START, START, ID, LENGTH, COMMAND (WRITE), REG_ADDR, VALUE, CHECKSUM]
  length = 4  # 3 + number of values
  cmd = 3     # WRITE_DATA command
  
  packet = [0xFF, 0xFF, motor_id, length, cmd, register, value]
  packet.append(checksum(packet))
  
  ser.write(bytes(packet))
  time.sleep(0.05)  # Give the motor time to process

def change_motor_id(comport, current_id, target_id):
  try:
    # Initialize serial communication
    ser = serial.Serial(comport, BAUDRATE, timeout=1)
    print(f"Connected to {comport} at {BAUDRATE} baud.")
    
    # Step 1: Unlock EEPROM (required to change the ID on most FeeTech models)
    print(f"Unlocking EEPROM for Motor {current_id}...")
    write_reg_byte(ser, CURRENT_ID, REG_LOCK, 0) 
    
    # Step 2: Write the new ID to register 5
    print(f"Changing ID from {current_id} to {target_id}...")
    write_reg_byte(ser, current_id, REG_ID, target_id)
    
    # Step 3: Lock EEPROM to save changes safely
    print(f"Locking EEPROM for Motor {target_id}...")
    write_reg_byte(ser, target_id, REG_LOCK, 1)
    
    print(success_msg := f"\nSuccess! Motor ID has been changed to {target_id}.")
    print("Please power cycle your motor for the changes to fully take effect.")
    
    ser.close()
      
  except serial.SerialException:
    print(f"Error: Could not open port {comport}. Check your connection and permissions.")
  except Exception as e:
    print(f"An error occurred: {e}")

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Configure STS motor ID")
    p.add_argument("--comport", "-c", help="COM port of the motor", default="/dev/ttyACM0")
    args = p.parse_args()

    print("Plug in the left motor and press enter to continue")
    input()
    change_motor_id(args.comport, CURRENT_ID, 7)
    print("Unplug the left motor and plug in the right motor and press enter to continue")
    input()
    change_motor_id(args.comport, CURRENT_ID, 8)

    print("Done! Please power cycle the motors for the changes to fully take effect.")
