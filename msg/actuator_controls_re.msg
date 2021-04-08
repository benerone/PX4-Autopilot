uint64 timestamp			# time since system start (microseconds)
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 ACT_STATUS_NONE = 0			# switch is not mapped
uint8 ACT_STATUS_PARTIAL = 1			# switch activated (value = 1)
uint8 ACT_STATUS_COMPLETE = 2			# middle position (value = 0)

float32[4] control
int32 index
uint8 status

# TOPICS actuator_controls_re actuator_controls_re_1 actuator_controls_re_2 actuator_controls_re_3
