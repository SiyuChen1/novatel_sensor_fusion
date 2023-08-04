std_msgs/Header header

float64 latitude
float64 longitude
# Height above mean sea level (metres)
float64 altitude

float64[9] position_covariance
string pos_sol_status
string pos_type

# Undulation - the relationship between the geoid and the ellipsoid (m) of the chosen datum
float64 undulation

int8 nb_sat_tracked
int8 nb_sat_solution

string vel_sol_status
string vel_type

# Horizontal speed over ground, in metres per second
float64 horizontal_speed

# Actual direction of motion over ground (track over ground) with respect to True North, in degrees
float64 direction_to_north

# Vertical speed, in metres per second, where positive values indicate increasing altitude (up)
# and negative values indicate decreasing altitude (down)
float64 vertical_speed