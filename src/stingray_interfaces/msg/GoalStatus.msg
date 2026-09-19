uint8 ACTIVE=0
uint8 REACHED=1
uint8 FAILED=2
uint8 ABORTED=3

std_msgs/Header header

uint8 status

float32 distance_to_goal
float32 position_error
float32 orientation_error

string message
