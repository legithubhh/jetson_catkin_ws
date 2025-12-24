float32 tpy        # TPY: Yaw 目标角度(°)，-180~180，空值表示速度控制
float32 tpp        # TPP: Pitch 目标角度(°)，-90~90，空值表示速度控制
float32 tvy        # TVY: Yaw 目标速度(°/s)，-360~360
float32 tvp        # TVP: Pitch 目标速度(°/s)，-360~360
uint8  te          # TE: 使能位，高位Yaw, 低位Pitch，0=使能, 1=失能
uint8  tpid        # TPID: PID 串环控制开关，0 开启 PID，其他值或无值关闭
uint8  td          # TD: 保留位

uint8  angle_auto  # 1: 触发角度自动/角度PID任务；否则不触发
uint8  speed_auto  # 1: 触发速度自动/速度PID任务；否则不触发
uint8  pid_auto    # 0: PID 定控模式；非 0 或无值：普通定控模式
