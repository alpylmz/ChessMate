import rospy
import random
from franka_msgs.srv import SetPositionCommand


# assume a 50 cm table located in 0.51 0.039
TABLE_CENTER_X = 0.51
TABLE_CENTER_Y = 0.039 
MAX_X_Y = 0.2


# decrease 40 cm for table
Z_FOR_TABLE = 0.3

number_of_trials = 0
rospy.init_node("movement_stress_test")
srv = rospy.ServiceProxy("/franka_go", SetPositionCommand)

while True:
    # choose a random place on the table
    x = random.uniform(TABLE_CENTER_X - MAX_X_Y, TABLE_CENTER_X + MAX_X_Y)
    y = random.uniform(TABLE_CENTER_Y - MAX_X_Y, TABLE_CENTER_Y + MAX_X_Y)
    z = Z_FOR_TABLE

    # choose a second place on the table
    x2 = random.uniform(TABLE_CENTER_X - MAX_X_Y, TABLE_CENTER_X + MAX_X_Y)
    y2 = random.uniform(TABLE_CENTER_Y - MAX_X_Y, TABLE_CENTER_Y + MAX_X_Y)
    z2 = Z_FOR_TABLE

    print("Moving to: ", x, y, z)
    # assume all of them are successful for now
    srv.call(x, y, z, 0, 0, 0, 1, False, False)

    print("Moving to: ", x2, y2, z2)
    # assume all of them are successful for now
    srv.call(x2, y2, z2, 0, 0, 0, 1, False, False)

    print("Moving to center")
    srv.call(TABLE_CENTER_X, TABLE_CENTER_Y, 0.68, 0, 0, 0, 1, False, False)


    number_of_trials += 1

    print("Number of trials: ", number_of_trials)
