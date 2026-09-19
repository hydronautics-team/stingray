std_msgs/Header header

uint8 UNKNOWN=0
uint8 GATE=1
uint8 YELLOW_FLARE=2
uint8 RED_FLARE=3
uint8 BLUE_FLARE=4
uint8 ORANGE_FLARE=5
uint8 MAT=6
uint8 DRUM=7
uint8 TARGET_DRUM=8
uint8 GATE_QUALIFICATION=9

uint8 type

uint8 STATUS_UNKNOWN=0
uint8 ACTIVE=1
uint8 STALE=2
uint8 LOST=3

uint8 status

uint64 id

geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 size

float32 confidence
uint32 observation_count

builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
