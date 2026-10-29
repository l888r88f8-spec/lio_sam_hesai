# Cloud Info
std_msgs/Header header 

int32[] start_ring_index
int32[] end_ring_index

int32[]  point_col_ind # point column index in range image
float32[] point_range # point range 

int64 imu_available
int64 odom_available

# Attitude for LOAM initialization
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

# Initial guess from imu pre-integration
float32 initial_guess_x
float32 initial_guess_y
float32 initial_guess_z
float32 initial_guess_roll
float32 initial_guess_pitch
float32 initial_guess_yaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature
sensor_msgs/PointCloud2 cloud_ground    # extracted ground feature

# Optional export-only input. Empty unless the selected ground algorithm needs it.
sensor_msgs/PointCloud2 cloud_export_scan # complete deskewed scan before projection thinning
uint32[] cloud_projection_indices # original row*columns+column for each cloud_deskewed point
uint32 cloud_projection_rows # original projection dimensions, current export only
uint32 cloud_projection_columns
