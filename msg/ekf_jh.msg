uint64 timestamp # time since system start (microseconds)

float32 yaw_wall
float32 x
float32 y
float32 z

#measurement
float32 distance_front
float32 distance_down
float32 distance_uwb_1
float32 distance_uwb_2

float32 roll
float32 pitch
float32 yaw

float32 odom_vx
float32 odom_vy
float32 odom_vz

float32[3] central_force
float32[3] quad_force
float32[4] actuators

float32 distance_front_filtered
float32 distance_down_filtered
float32 distance_uwb_1_filtered
float32 distance_uwb_2_filtered

float32[3] uwb_anchor_1_pos
float32[3] uwb_anchor_2_pos

float32 delta_t_s
