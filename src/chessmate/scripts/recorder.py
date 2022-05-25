import json
import rospy
from sensor_msgs.msg import JointState
from time import time

# read old joint names and positions from file
# if the file does not exist, create it
# if the file exists, read the old names and positions
# and create a dictionary 
data_dict = {}
try:
    with open('joint_positions.json') as json_file:
        data_dict = json.load(json_file)
except:
    with open('joint_positions.json', 'w') as json_file:
        json.dump(data_dict, json_file, indent=4, sort_keys=True)





joint_state_glob = None
def jointStateCallback(msg):
    global joint_state_glob
    joint_state_glob = msg

# init ros node
rospy.init_node('joint_state_recorder')


print("Subscribing")
# subscribe to topic /joint_states
sub = rospy.Subscriber('/joint_states', JointState, jointStateCallback)
print("Subscribed")

# add new joint_states to dict
def addJointState(joint_state, chessboard_place):
    global data_dict
    data_dict[chessboard_place] = joint_state.position
    # before writing dict to file
    # backup the current dict to backups folder using current timestamp
    curr_time = time()
    with open("backups/" + str(curr_time) + '_joint_positions.json', 'w') as json_file:
        json.dump(data_dict, json_file, indent=4, sort_keys=True)
    with open('joint_positions.json', 'w') as json_file:
        json.dump(data_dict, json_file, indent=4, sort_keys=True)


# wait for first message
while joint_state_glob is None:
    pass
print("Got first message")

while True:
    # wait for user input, chessboard place
    chessboard_place = input("Please enter chessboard place(eg. \"a8\", \"h3\" ): ")

    # get current joint_state
    temp_joint_state = joint_state_glob

    # add current joint_state to dict
    addJointState(temp_joint_state, chessboard_place)

    


