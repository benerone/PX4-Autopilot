# uORB hil state quaternion

uint64 timestamp			# time since system start (microseconds)
float32[4] attitude_quaternion #Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
float32 rollspeed  # [rad/s] Body frame roll / phi angular speed
float32 pitchspeed # [rad/s] Body frame pitch / theta angular speed
float32 yawspeed # [rad/s] Body frame yaw / psi angular speed
int32 lat # [degE7] Latitude
int32 lon # [degE7] Longitude
int32 alt # [mm] Altitude
int16 vx # [cm/s] Ground X Speed (Latitude)
int16 vy # [cm/s] Ground Y Speed (Longitude)
int16 vz # [cm/s] Ground Z Speed (Altitude)
uint16 ind_airspeed # [cm/s] Indicated airspeed
uint16 true_airspeed # [cm/s] True airspeed
int16 xacc # [mG] X acceleration
int16 yacc # [mG] Y acceleration
int16 zacc # [mG] Z acceleration
