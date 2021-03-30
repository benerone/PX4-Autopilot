uint64 timestamp		# time since system start (microseconds)

uint8 INTEGRALE_STATUS_NONE = 0			# switch is not mapped
uint8 INTEGRALE_STATUS_PARTIAL = 1		# switch activated (value = 1)
uint8 INTEGRALE_STATUS_COMPLETE = 2		# middle position (value = 0)

float32 roll_rate_integral	# roll rate integrale
float32 pitch_rate_integral	# pitch rate integrale
float32 yaw_rate_integral	# yaw rate integrale
float32 thrust			# thrust
float32 vx			# thrust
float32 vy			# thrust
float32 throttle_act		# throttle_act
uint8 status			# data status
int32 index			# mavlink index

# TOPICS integrale r1integrale r2integrale r3integrale
