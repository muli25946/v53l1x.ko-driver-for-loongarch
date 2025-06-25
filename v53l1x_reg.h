#ifndef __v53l1x_reg_h
#define __v53l1x_reg_h

/*不明的数组，可能用于不同结果下的修正？*/
static const uint8_t status_rtn[24] = {255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
                                       255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                                       255, 255, 11, 12};

/***************** ST官方驱动寄存器地址表 (VL53L1X) *******************/

/* 系统控制寄存器 */
#define SOFT_RESET 0x0000                       // 软件复位控制 (写入0x00复位所有配置)
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS 0x0001 // I²C从机地址配置 (7位地址，默认0x29)

/* 测距参数校准寄存器 */
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x0008       // VHV超时宏循环边界 (影响测量范围)
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x0016     // 串扰补偿平面偏移 (kcps单位)
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018 // X平面串扰补偿梯度
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A // Y平面串扰补偿梯度
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x001E                 // 器件间距离偏移校准 (毫米)
#define MM_CONFIG__INNER_OFFSET_MM 0x0020                         // 内部偏移校准值 (毫米)
#define MM_CONFIG__OUTER_OFFSET_MM 0x0022                         // 外部偏移校准值 (毫米)

/* GPIO与中断控制 */
#define GPIO_HV_MUX__CTRL 0x0030             // GPIO高电压多路复用控制
#define GPIO__TIO_HV_STATUS 0x0031           // GPIO/TIO引脚状态 (Bit0:TIO状态, Bit1:HV状态)
#define SYSTEM__INTERRUPT_CONFIG_GPIO 0x0046 // GPIO中断配置 (触发方式/极性)

/* 时序与相位配置 */
#define PHASECAL_CONFIG__TIMEOUT_MACROP 0x004B   // 相位校准超时宏周期
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI 0x005E // 测距超时宏周期A (高字节)
#define RANGE_CONFIG__VCSEL_PERIOD_A 0x0060      // VCSEL发射器周期A (影响精度/功耗)
#define RANGE_CONFIG__VCSEL_PERIOD_B 0x0063      // VCSEL发射器周期B
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI 0x0061 // 测距超时宏周期B (高字节)
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO 0x0062 // 测距超时宏周期B (低字节)

/* 测距算法配置 */
#define RANGE_CONFIG__SIGMA_THRESH 0x0064                  // 信号标准差阈值 (滤除噪声)
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS 0x0066 // 最小返回计数率限制 (Mcps)
#define RANGE_CONFIG__VALID_PHASE_HIGH 0x0069              // 有效相位高阈值
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD 0x006C      // 测量间隔周期 (控制采样率)

/* 阈值与ROI配置 */
#define SYSTEM__THRESH_HIGH 0x0072                           // 高阈值 (用于中断触发)
#define SYSTEM__THRESH_LOW 0x0074                            // 低阈值 (用于中断触发)
#define SD_CONFIG__WOI_SD0 0x0078                            // 窗口积分起始时间 (SD0)
#define SD_CONFIG__INITIAL_PHASE_SD0 0x007A                  // 初始相位配置 (SD0)
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD 0x007F              // 用户ROI中心SPAD坐标
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE 0x0080 // 用户ROI全局尺寸 (XY方向)

/* 系统操作寄存器 */
#define SYSTEM__SEQUENCE_CONFIG 0x0081               // 测量序列配置 (选择测距模式)
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 0x0082 // 参数组保持 (0=更新立即生效, 1=保持)
#define SYSTEM__INTERRUPT_CLEAR 0x0086               // 中断清除 (写入1清除中断)
#define SYSTEM__MODE_START 0x0087                    // 模式启动 (写入0x40启动连续测量)

/* 结果寄存器 */
#define VL53L1_RESULT__RANGE_STATUS 0x0089                                        // 测距状态 (0=有效, 其他=错误码)
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0 0x008C                      // 实际有效SPAD数 (SD0)
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD 0x0090                                 // 环境光计数率 (Mcps)
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 0x0096              // 最终串扰校正距离值 (毫米, SD0)
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098 // 峰值信号计数率 (Mcps, SD0)

/* 校准与状态寄存器 */
#define VL53L1_RESULT__OSC_CALIBRATE_VAL 0x00DE // 振荡器校准值
#define VL53L1_FIRMWARE__SYSTEM_STATUS 0x00E5   // 系统状态

/* 设备识别寄存器 */
#define VL53L1_IDENTIFICATION__MODEL_ID 0x010F // 设备型号ID (VL53L1X=0xEACC)

/* 高级ROI配置 */
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x013E // ROI中心SPAD模式配置

const u8 VL51L1X_DEFAULT_CONFIGURATION[] = {
    0x00, /* 0x2d : 设置 bit2 和 bit5 为1启用快速增强模式(1MHz I2C)，否则不要修改 */
    0x00, /* 0x2e : 如果I2C上拉到1.8V则bit0=0，否则设置bit0=1(上拉到AVDD) */
    0x00, /* 0x2f : 如果GPIO上拉到1.8V则bit0=0，否则设置bit0=1(上拉到AVDD) */
    0x01, /* 0x30 : 设置bit4=0为高电平有效中断，bit4=1为低电平有效(bit3:0必须为0x1)，使用SetInterruptPolarity() */
    0x02, /* 0x31 : bit1=根据极性配置中断，使用CheckForDataReady() */
    0x00, /* 0x32 : 不可用户修改 */
    0x02, /* 0x33 : 不可用户修改 */
    0x08, /* 0x34 : 不可用户修改 */
    0x00, /* 0x35 : 不可用户修改 */
    0x08, /* 0x36 : 不可用户修改 */
    0x10, /* 0x37 : 不可用户修改 */
    0x01, /* 0x38 : 不可用户修改 */
    0x01, /* 0x39 : 不可用户修改 */
    0x00, /* 0x3a : 不可用户修改 */
    0x00, /* 0x3b : 不可用户修改 */
    0x00, /* 0x3c : 不可用户修改 */
    0x00, /* 0x3d : 不可用户修改 */
    0xff, /* 0x3e : 不可用户修改 */
    0x00, /* 0x3f : 不可用户修改 */
    0x0F, /* 0x40 : 不可用户修改 */
    0x00, /* 0x41 : 不可用户修改 */
    0x00, /* 0x42 : 不可用户修改 */
    0x00, /* 0x43 : 不可用户修改 */
    0x00, /* 0x44 : 不可用户修改 */
    0x00, /* 0x45 : 不可用户修改 */
    0x20, /* 0x46 : 中断配置 0->低电平检测,1->高电平检测,2->超出窗口,3->在窗口内,0x20->新样本就绪 */
    0x0b, /* 0x47 : 不可用户修改 */
    0x00, /* 0x48 : 不可用户修改 */
    0x00, /* 0x49 : 不可用户修改 */
    0x02, /* 0x4a : 不可用户修改 */
    0x0a, /* 0x4b : 不可用户修改 */
    0x21, /* 0x4c : 不可用户修改 */
    0x00, /* 0x4d : 不可用户修改 */
    0x00, /* 0x4e : 不可用户修改 */
    0x05, /* 0x4f : 不可用户修改 */
    0x00, /* 0x50 : 不可用户修改 */
    0x00, /* 0x51 : 不可用户修改 */
    0x00, /* 0x52 : 不可用户修改 */
    0x00, /* 0x53 : 不可用户修改 */
    0xc8, /* 0x54 : 不可用户修改 */
    0x00, /* 0x55 : 不可用户修改 */
    0x00, /* 0x56 : 不可用户修改 */
    0x38, /* 0x57 : 不可用户修改 */
    0xff, /* 0x58 : 不可用户修改 */
    0x01, /* 0x59 : 不可用户修改 */
    0x00, /* 0x5a : 不可用户修改 */
    0x08, /* 0x5b : 不可用户修改 */
    0x00, /* 0x5c : 不可用户修改 */
    0x00, /* 0x5d : 不可用户修改 */
    0x01, /* 0x5e : 不可用户修改 */
    0xcc, /* 0x5f : 不可用户修改 */
    0x0f, /* 0x60 : 不可用户修改 */
    0x01, /* 0x61 : 不可用户修改 */
    0xf1, /* 0x62 : 不可用户修改 */
    0x0d, /* 0x63 : 不可用户修改 */
    0x01, /* 0x64 : Sigma阈值高字节(毫米，14.2格式)，使用SetSigmaThreshold()，默认值90mm */
    0x68, /* 0x65 : Sigma阈值低字节 */
    0x00, /* 0x66 : 最小计数率高字节(MCPS，9.7格式)，使用SetSignalThreshold() */
    0x80, /* 0x67 : 最小计数率低字节 */
    0x08, /* 0x68 : 不可用户修改 */
    0xb8, /* 0x69 : 不可用户修改 */
    0x00, /* 0x6a : 不可用户修改 */
    0x00, /* 0x6b : 不可用户修改 */
    0x00, /* 0x6c : 测量间隔周期高字节(32位寄存器)，使用SetIntermeasurementInMs() */
    0x00, /* 0x6d : 测量间隔周期 */
    0x0f, /* 0x6e : 测量间隔周期 */
    0x89, /* 0x6f : 测量间隔周期低字节 */
    0x00, /* 0x70 : 不可用户修改 */
    0x00, /* 0x71 : 不可用户修改 */
    0x00, /* 0x72 : 距离阈值上限高字节(毫米，MSB+LSB)，使用SetDistanceThreshold() */
    0x00, /* 0x73 : 距离阈值上限低字节 */
    0x00, /* 0x74 : 距离阈值下限高字节(毫米，MSB+LSB)，使用SetDistanceThreshold() */
    0x00, /* 0x75 : 距离阈值下限低字节 */
    0x00, /* 0x76 : 不可用户修改 */
    0x01, /* 0x77 : 不可用户修改 */
    0x0f, /* 0x78 : 不可用户修改 */
    0x0d, /* 0x79 : 不可用户修改 */
    0x0e, /* 0x7a : 不可用户修改 */
    0x0e, /* 0x7b : 不可用户修改 */
    0x00, /* 0x7c : 不可用户修改 */
    0x00, /* 0x7d : 不可用户修改 */
    0x02, /* 0x7e : 不可用户修改 */
    0xc7, /* 0x7f : ROI中心点，使用SetROI() */
    0xff, /* 0x80 : XY ROI(X=宽度,Y=高度)，使用SetROI() */
    0x9B, /* 0x81 : 不可用户修改 */
    0x00, /* 0x82 : 不可用户修改 */
    0x00, /* 0x83 : 不可用户修改 */
    0x00, /* 0x84 : 不可用户修改 */
    0x01, /* 0x85 : 不可用户修改 */
    0x00, /* 0x86 : 清除中断，使用ClearInterrupt() */
    0x00  /* 0x87 : 开始测距，使用StartRanging()或StopRanging()，如果希望在VL53L1X_init()调用后自动开始，请设置0x87位置为0x40 */
};

#endif
