# SwarmCore 任务分配消息（见产品定义书 附录A）
std_msgs/Header header
uint32 task_id
uint8 assignee_type    # 0 无人机 1 无人车
uint8 assignee_id
uint8 task_type        # 0 确认 1 跟踪 2 接近 3 待命
uint32 target_id
geometry_msgs/Pose goal
