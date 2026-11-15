# One sample of a planned trajectory, base_link (rear-axle centre) pose in the map frame.
float64 x
float64 y
float64 z
float64 yaw
float64 v          # signed longitudinal speed [m/s] (negative when reversing)
float64 a          # planned acceleration [m/s^2]
float64 kappa      # path curvature [1/m]
float64 s          # arc length from the start [m]
float64 grade      # road grade along the heading (sin of pitch, positive uphill)
int8 direction     # +1 forward, -1 reverse
