import struct

class EncoderReader():
    def __init__(self):
        self.ticks_left = 0
        self.ticks_right = 0

    @staticmethod
    def read_string(string):
        test = struct.unpack("I", string)
        print test

if __name__ == "__main__":
    encode = EncoderReader()
    encode.read_string(str(chr(255))+str(chr(255))+str(chr(255))+str(chr(255)))
    """motor_pub = rospy.Publisher("motor_command", String)
    pub = rospy.init_node("motor_command", "navigation")"""