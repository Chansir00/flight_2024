# main

#### Key_Scan（）按键扫描

##### Key_Init(void)定义按键

_button.state[UP] P1.1

_button.state[DOWN] P1.4

##### Read_Button_State_One(button_state *button)记录时间定义标志位

##### Key_Scan(uint8_t release)按键扫描)(扫描后添加功能)

有按键翻页功能和遥控器翻页功能

short press给按键翻页

当第四页时按键长按为SDK1_Mode_Setup++;

第五页时SDK2_Mode_Setup++;



## sdk1和2的区别?

SDK1好像才是切换模式，SDK2可能是预留的



#### QuadShow(Oled_Show_Enable);OLED显示

将各个数据显示到oled各个页上





#### Accel_Calibartion()加速度计标定即校准

通过400hz刷新的Accel_Calibration_Check();随时检测按键进行校准采集

将传感器采集到的的xyz轴上的WP_Sensor.acce_filter[j]转换成1g标准

采集6个面（即6种角度）上xyz轴的1g标准值分别加起来输入到acce_sample_sum[j]

再保存到acce_sample[i]

再通过Calibrate_accel（）处理后输入到new_scales.xyz和new_offset.xyz

#### Mag_Calibartion_LS(&WP_Sensor.mag_raw,Circle_Angle);磁力计椭球校准

入口参数：陀螺仪积分角度值、三轴磁力计原始值

WP_Sensor.mag_raw(未滤波的陀螺仪积分角度值)

校准后得到Mag_Offset[j]



#### RC_Calibration_Check(PPM_Databuf);遥控器行程标定

输入PPM_Databuf

输出RC_Calibration[i].middle行程中位

RC_Calibration[i].deadband中位死区





# Task_200hz_Thread()



## CarryPilot_Control();总控制器

### Controler_Mode_Select();

先进行模式选择

### Total_Control();

#### Main_Leading_Control();主导控制

先判断是否为一键降落模式，若是则直接跳出

再判断高度纯手动模式和定高模式

##### 纯手动模式

Throttle=Throttle_Control;//油门直接来源于遥控器油门给定

Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];遥控数据赋给控制器

##### 定高模式下又分SDK模式和普通定高、光流定点

SDK模式直接Auto_Flight_Ctrl(SDK1_Mode_Setup);

普通定高也可以手动控制，也是将RC_Data.rc_rpyt[RC_ROLL]输入到Controller.roll_outer_control_output

光流定点下直接调用

OpticalFlow_Control_Pure(0);
并Controller.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];

两种定高模式最后都执行

Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);高度控制

#### Attitude_Control();姿态控制（PID控制）



##### Angle_Control_Target(ROTATE,Controller.roll_outer_control_output,Controller.pitch_outer_control_output,Controller.yaw_outer_control_output);角度控制，即外环控制

输入为Controller.roll_outer_control_output

还有一个输入为Roll-Roll_Offset)

运行PID_Control后将外环输出作为内环期望

- 以上只控制ROLL和PITCH，偏航角用rotate状态控制
- 当rotate==ROTATE时，由遥控器控制输出直接作为内环期望，只进行内环角速度控制
- 当rotate==AZIMUTH时，先进行角度控制，再由角度控制器输出作为内环期望



 ##### Gyro_Control();角速度PID控制



#### Control_Output();控制总输出

解锁后开始自检，若未解锁电机赋0

里面比较重要的就

#####  landon_earth_check();着陆条件自检

其他就是判断一下姿态档位还是定高

但是输出其实都没区别

都是用Moter1_Thr/roll/pitch_Scale参数或乘以Throttle_Output

或者Total_Controller.Roll_Gyro_Control.Control_OutPut

这个值是由PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control);PID最后得出的







## Get_Status_Feedback();获取姿态数据、水平与竖直方向惯导数据

若接了磁力计即Sensor_Flag.Mag_Health==TRUE

则通过偏航角一阶互补融合得到 Inertia_Yaw

##### Sensor_Update()传感器数据采集更新

通过mpu6050和SPL06和磁力计读取数据

得到校准后的角速度、加速度、磁力计数据

对数据进行滤波处理后输入到

gyro_bpf_filter.xyz

gyro_filter_bug.xyz

gyro_filter.xyz

accel_filter.xyz

ins_accel_filter.xyz

Accel_For_Cal[j]

WP_Sensor.acce_filter[j]

mag_filter.xyz(滤波后的磁力计数据)



##### Strapdown_INS_High_Kalman();

高度方向卡尔曼滤波估计竖直速度、位置

输出 Ins_Kf惯导结构体





# Task_400hz_Thread()

### Remote_Control();遥控器数据解析



### Get_Battery_Voltage();		    测量电池电压

宏定义在WP.ctrl内有锂电池电压前馈补偿使能,若电池不好可以进行补偿



### Read_Button_State_All();按键

内有按键计时，可以设置长按和短按功能



### Check_All_Calibartion();传感器校准按键检测

##### Accel_Calibration_Check();			//加速度

##### Horizontal_Calibration_Check();机架水平

##### Mag_Calibration_Check();				//磁力计

##### ESC_Calibration_Check();





## 一键起飞 20HZ

按键触发

解锁标志位赋1

给定起飞目标高度

给定起飞速度

（目标高度-当前高度）*1000/起飞速度

 runtime_ms += dT_ms;即每运行一次 runtime_ms加50，因为这个函数50ms触发一次

AUTO_TAKE_OFF_SPEED赋给高度控制器中的起飞降落Z速度PROGRAM_CTRLF_LAND_VEL

当runtime_ms也就是运行时间大于exp_time_ms起飞时间后起飞标志位置于起飞完成，起飞降落Z速度置0





没有触发时one_key_takeoff_f等于0，会判断是否处于起飞完成状态

若进入起飞完成状态会给program_ctrl.auto_takeoff_land_velocity也就是PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL起飞速度置0

当起飞状态为初始时ref_height = ALT_CM

ALT_CM为HeightInfo.Z_Postion(Z轴当前位置)





触发后通过One_Key_Takeoff()解锁将one_key_takeoff_f置1

将auto_takeoff_land_state 置为 AUTO_TAKE_OFF

进入起飞主程序

EXP_TAKEOFF_ALT_CM 为起飞目标高度，默认为90

AUTO_TAKE_OFF_SPEED默认为30

  自动计算起飞时间，即将(EXP_TAKEOFF_ALT_CM - ref_height) * 1000 / AUTO_TAKE_OFF_SPEED赋给exp_time_ms也就是起飞时间

 runtime_ms += dT_ms;即每运行一次 runtime_ms加50，因为这个函数50ms触发一次

AUTO_TAKE_OFF_SPEED赋给高度控制器中的起飞降落Z速度PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL

当runtime_ms也就是运行时间大于exp_time_ms起飞时间后起飞标志位置于起飞完成，起飞降落Z速度置0







## 定高控制

### Flight_Alt_Hold_Control

模式ALTHOLD_MANUAL_CTRL高度手动下，可自己手动调一次高度

模式ALTHOLD_AUTO_VEL_CTRL高度控制下，用target_vel赋给Total_Controller.High_Speed_Control.Expect

模式ALTHOLD_AUTO_POS_CTRL下，用NamelessQuad.Position[_YAW]给

Total_Controller.High_Position_Control.FeedBack赋值













# 仿真参数

### 标志参数

Controler_High_Mode	2为定高1为手动

Controler_SDK1_Mode	1为sdk模式，0为非

Controler_SDK1_Mode     2为一键降落，1非

Controler_SDK1_Mode     1水平自稳，0光流

SDK_Duty_Cnt   SDK内部函数的模式计数标志位

SDK_Duty_Status.Status[SDK_Duty_Cnt].Start_Flag///Execute_Flag///End_flag   分别为SDK函数的开始执行结束标志位





### 输出参数

##### 电机

Motor_PWM_1等为电机最终输出

Throttle_Output 为油门输出

##### 超声波

Ground_Distance  也可能是高度

Ground_Distance_Div=us100.vel  可能是超声波高度

Ground_Distance_Acc

##### 陀螺仪

gyro_offset.x 开机零偏标定

accel->x加速度

gyro->x角速度



————————————————
版权声明：本文为凤玲天天_Electronics的原创文章，转载请附上原文出处链接及本声明。
原文链接：http://fenglingtiantian.cn