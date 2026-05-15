std_msgs/Header                 header
uint64 seq
#attribute of the inspection point
bool                            IsInspectionPoint
geometry_msgs/Point             point                           #航点
bool                            free_nav                        #自由导航
bool                            ENABLE_YAW                      #使能航点yaw
float64                         point_yaw                       #航点yaw
float64                         stop_duration                   #航点处的停留时间
uint16[]                        ACTIONS                         #航点处需要执行的动作
geometry_msgs/Point             point_vel                       #用户指定的航点的速度默认且必须为0，为了确保轨迹平直而插入的点可以不为0
bool                            FORCEGOAL                       #用户给的强制点，接收到该点时可以在不是waittarget状态下直接进行规划    

#attribute of the inspection line
float64                         line_vel                        #航线上的速度 发布前需要提前确认是否与point vel冲突
bool                            ENABLE_LINE_AVOIDANCE           #是否沿线避障，若不避障则遇到障碍物急停，若避障则遇到障碍物后急停然后择机避障
float64                         avoidance_radius                #沿着航线上的避障半径

                         



