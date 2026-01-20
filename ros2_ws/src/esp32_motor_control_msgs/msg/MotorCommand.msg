# Motor command + strategy trigger message.
#
# NOTE (source of truth): Strategy triggers are implemented in the bridge node:
# - angle_auto == 1 starts angle task
# - speed_auto == 1 starts speed task

float32 tpy        # TPY: yaw target angle (deg); when omitted/empty -> speed control mode
float32 tpp        # TPP: pitch target angle (deg); when omitted/empty -> speed control mode
float32 tvy        # TVY: yaw target speed (deg/s)
float32 tvp        # TVP: pitch target speed (deg/s)
uint8  te          # TE: enable bits (yaw: high, pitch: low); 0=enable, 1=disable
uint8  tpid        # TPID: PID mode flag (bridge uses 0 for PID on, 0xFF for off/ignore)
uint8  td          # TD: reserved

uint8  angle_auto  # 1: trigger angle task; otherwise no angle task
uint8  speed_auto  # 1: trigger speed task; otherwise no speed task
uint8  pid_auto    # 0: PID task mode; otherwise normal task mode
