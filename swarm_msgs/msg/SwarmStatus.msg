# 三机集群状态汇总
std_msgs/Header header
uint8   robot_id       # 1=Scout 2=Carrier 3=Specialist
uint8   mode
float32 battery_pct
float32 pos_x
float32 pos_y
float32 yaw
bool    is_scanning    # Scout正在扫描
bool    network_ok     # Carrier网络状态
uint8   supply_count   # Carrier剩余物资数
bool    arm_busy       # Specialist机械臂工作中
string  status_msg
