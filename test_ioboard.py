import struct

def construct_can_message(number):
    # Convert the number to a 32-bit unsigned integer
    number = struct.pack('I', number)

    # Create the CAN message
    can_message = bytearray()
    can_message.extend(number)

    return can_message

# Example usage
number = 1234567890
can_message = construct_can_message(number)
print(can_message)