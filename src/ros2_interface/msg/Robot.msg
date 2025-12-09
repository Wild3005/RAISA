float32 pose_x
float32 pose_y
float32 pose_theta

float32 vel_linear
float32 vel_angular

int8 mode    # 0 idle 1 manual 2 auto

int8 battery_level   # percentage 0-100
int8 charge_flag 
int8 emergency_flag

int8 nav_status_res
int8 nav_status_reason
string nav_status_goal
float32 nav_status_dist
float32 nav_status_mileage