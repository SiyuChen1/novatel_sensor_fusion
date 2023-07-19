std_msgs/Header header
geometry_msgs/Quaternion quaternion

float64[9] angle_covariance

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3

uint8 angle_covariance_type

# Inertial Solution Status, https://docs.novatel.com/OEM7/Content/PDFs/OEM7_Commands_Logs_Manual.pdf
# page 1111
# uint8 INS_INACTIVE = 0
# uint8 INS_ALIGNING = 1
# uint8 INS_HIGH_VARIANCE = 2
# uint8 INS_SOLUTION_GOOD = 3
# uint8 INS_SOLUTION_FREE = 6
# uint8 INS_ALIGNMENT_COMPLETE = 7
# uint8 DETERMINING_ORIENTATION = 8
# uint8 WAITING_INITIALPOS = 9
# uint8 WAITING_AZIMUTH = 10
# uint8 INITIALIZING_BIASES = 11
# uint8 MOTION_DETECT = 12
# uint8 WAITING_ALIGNMENTORIENTATION = 14
# uint8 ins_status
string ins_status

# Position type See Table 93: Position or Velocity Type on page 541
string pos_type

string rec_status