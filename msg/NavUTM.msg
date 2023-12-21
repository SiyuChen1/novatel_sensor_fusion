# msg format used for UTM data
# https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
# definition of ecef coordinate system on page 551

std_msgs/Header header
string pos_sol_status
string pos_type

int8 longitudinal_zone_num
string longitudinal_zone_letter

float64 northing
float64 easting
float64 height

float64 undulation

float64[9] pos_covariance

string base_station_id

float64 diff_age
float64 sol_age

int8 nb_sat_tracked
int8 nb_sat_solution