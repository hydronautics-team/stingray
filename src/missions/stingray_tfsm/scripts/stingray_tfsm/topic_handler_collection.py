from ast import literal_eval
import rospy


def decipher_objects(response):
    detected = dict()
    response = response.split('\n')
    curr_name = None
    possibilities = ('confidence', 'top_left_x', 'top_left_y', 'bottom_right_x', 'bottom_left_y')
    for line in response:
        lin = [i.strip() for i in line.split(':')]
        if lin[0] == 'name':
            curr_name = literal_eval(lin[1])
            detected[curr_name] = []
        elif lin[0] in possibilities:
            detected[curr_name].append(literal_eval(lin[1]))

    return detected
