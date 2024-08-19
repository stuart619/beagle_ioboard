import can
import time

bus = can.Bus(bustype='serial', channel='COM8', bitrate=250000)

def send_can_message(identifier, data):
    ts = time.time()
    message = can.Message(arbitration_id=identifier, data=data, is_extended_id=True)
    bus.send(message)
    
# function to send TWAI message to the CAN bus
    

if __name__ == "__main__":
    CAN_LEFT_CAM_ID = 0x014  # Replace with the actual identifier value

    for position in range(0, 1501, 100):
        print(f"Position: {position}")
        data = [0x00, position >> 8, position & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00]
        send_can_message(CAN_LEFT_CAM_ID, data)
        time.sleep(1.0)  # Adjust the delay as needed
    
    bus.shutdown()