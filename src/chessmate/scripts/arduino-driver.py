import serial


SERIAL_PORT="/dev/ttyUSB0"
BAUDRATE=9600

ROBOT_PLAY=100000
OPPONENT_PLAY=100001
WIN=100002
LOSS=100003
IDLE=100004

class GetState():

    def __init__(self) -> None:
        self.serial_com = serial.Serial(SERIAL_PORT,BAUDRATE)


    def get_state(self):
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


if __name__ == '__main__':
    getstate = GetState()
    a = input()
    getstate.get_state()
    a = input() 
    getstate.get_state()
    a = input()
    getstate.get_state()

