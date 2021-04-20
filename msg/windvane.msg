uint64 timestamp                # time since system start (microseconds)

float64 lat			# Latitude, (degrees)
float64 lon			# Longitude, (degrees)
float32 alt

int16[3] ground_speed 	# int16 / 100
int16[3] attitude 		# int16 / 100

int16[2] voltage 		# int16 / 100
int16 aoa 			# int16 / 100
int16 aos 			# int16 / 100
int16 airspeed 			# int16 / 100

float32 differential_pressure_pa
float32 baro_pressure_pa

float32[3] wind_speed_3d

float32 wind_speed_horiz
float32 wind_angle_horiz
