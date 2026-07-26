# SwarmCore 目标地图消息（见产品定义书 附录A）
std_msgs/Header header
uint32 target_id
string label
float32 confidence
geometry_msgs/Point position
float64[9] position_covariance
uint8 source_uav_id
float32 detection_age
uint8 tracking_state
