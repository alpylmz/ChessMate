import serial
import rospy
from chessmate.srv import chesswatch_serv, chesswatch_servResponse

SERIAL_PORT="/dev/ttyUSB0"
BAUDRATE=9600

ROBOT_PLAY=100000
OPPONENT_PLAY=100001
WIN=100002
LOSS=100003
IDLE=100004

class ArduinoDriver():

    def __init__(self) -> None:
        self.serial_com = serial.Serial(SERIAL_PORT,BAUDRATE)


    def get_game_state(self):
        self.serial_com.write("G".encode('utf-8'))
        packet = self.serial_com.read()
        print(packet.decode('utf-8'))
        if packet.decode('utf-8') == 'R':
            print("Robot play")
            return ROBOT_PLAY

        elif packet.decode('utf-8') == 'O':
            print("Opponent play")
            return OPPONENT_PLAY
        
        elif packet.decode('utf-8') == 'W':
            print("Win")
            return WIN

        elif packet.decode('utf-8') == 'L':
            print("Loss")
            return LOSS
        
        else:
            print("Idle")
            return IDLE


    def change_game_state(self):
        self.serial_com.write("C".encode('utf-8'))
        print("Signal is sent.")


arduino_driver = ArduinoDriver()
def rospy_service_callback(req):
    global arduino_driver
    if req.request == "get":
        return chesswatch_servResponse(arduino_driver.get_game_state())
    elif req.request == "change":
        arduino_driver.change_game_state()
        return chesswatch_servResponse(1)
    


if __name__ == '__main__':
    print("this is arduino driver")
    # rospy create node
    rospy.init_node('arduino_driver')
    # create service "/chess_clock"
    rospy.Service('/chess_clock', chesswatch_serv, rospy_service_callback)
    rospy.spin()
    """
    a = input()
    arduino_driver.change_game_state()
    a = input() 
    arduino_driver.get_game_state()
    a = input()
    arduino_driver.get_game_state()
    a = input()
    arduino_driver.get_game_state()
    """