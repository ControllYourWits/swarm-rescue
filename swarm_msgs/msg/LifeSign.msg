# 毫米波雷达生命体征检测结果
std_msgs/Header header
float32 range          # 目标距离 m
float32 breath_freq    # 呼吸频率 Hz
float32 heart_freq     # 心跳频率 Hz
float32 confidence     # 置信度 0~1
float32 world_x        # 估算世界坐标
float32 world_y
