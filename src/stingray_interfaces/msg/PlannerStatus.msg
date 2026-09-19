std_msgs/Header header

uint8 IDLE=0
uint8 PLANNING=1
uint8 EXECUTING=2
uint8 GOAL_REACHED=3
uint8 REPLANNING=4
uint8 BLOCKED=5
uint8 FAILED=6
uint8 STOPPED=7

uint8 state

float32 distance_to_goal
bool obstacle_detected

string message
