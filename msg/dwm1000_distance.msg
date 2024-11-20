# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)

uint16[6] anchor_distance	# distance in cm to each UWB Anchor (UWB Device wich takes part in Ranging)
float32[3] anchor1_position
float32[3] position		# Position of the Landing point in relation to the Drone (x,y,z in Meters NED)
