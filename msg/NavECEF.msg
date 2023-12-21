# msg format used for measured Cartesian position and velocity
# https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
# definition of ecef coordinate system on page 559
std_msgs/Header header

string pos_sol_status
string pos_type
# in the order with
# Position X-coordinate (m)
# Position Y-coordinate (m)
# Position Z-coordinate (m)
geometry_msgs/Vector3 pos
float64[9] pos_covariance

string vel_sol_status
string vel_type

geometry_msgs/Vector3 vel
float64[9] vel_covariance

string base_station_id

float64 diff_age
float64 sol_age

int8 nb_sat_tracked
int8 nb_sat_solution