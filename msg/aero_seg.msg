uint64 timestamp	# time since system start (microseconds)

# aero segment for analysis purpose

float32 alpha 		# angle of attack [deg]
float32 alpha_eff 	# angle of attack effective [deg]
float32 alpha_eff_dot 	# angle of attack effective derivative [deg/s]
float32 alpha_min 	# minimum angle of attack [deg]
float32 alpha_max	# maximum angle of attack [deg]
float32 flap 		# flap angle [deg]
float32 cl 		# lift coefficient
float32 cd 		# drap coefficient
float32 cm 		# pitching moment coefficient
float32 ar 		# aspect ratio span/mac
float32 cr 		# chord ratio flap_chord/chord
uint8   test_nb 	# index to separate the different tests
