#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "v53l1x_reg.h"

#define V53L1X_CNT 1
#define V53L1X_NAME "v53l1x"
#define LONG_RANGE_MODE // 可选LONG_RANGE_MODE宏定义，开启长距离模式 or SHORT_RANGE_MODE宏定义，开启短距离模式

u8 first_range = 1; // 第一次测量标志位，第一次测量将被忽略，因此需要两次调用(readdata中)

struct v53l1x_dev
{
    dev_t devid;            /* 设备号 	 */
    struct cdev cdev;       /* cdev 	*/
    struct class *class;    /* 类 		*/
    struct device *device;  /* 设备 	 */
    struct device_node *nd; /* 设备节点 */
    int major;              /* 主设备号 */
    void *private_data;     /* 私有数据 */
    u16 distance_mm;
};

struct VL53L1X_Result_t // 最终结果结构体
{
    uint8_t Status;      /* 状态 */
    uint16_t Distance;   /* 距离 */
    uint16_t Ambient;    /* 环境 */
    uint16_t SigPerSPAD; /* 信号/SPAD */
    uint16_t NumSPADs;   /* SPADs号 */
};

static struct v53l1x_dev v53l1xdev;
static struct VL53L1X_Result_t v53l1xresult; // 指向结果结构体的指针

/*
 * @description	: 从v53l1x读取多个寄存器数据（支持16位地址）
 * @param - dev:  v53l1x设备
 * @param - reg:  16位寄存器地址
 * @param - val:  读取到的数据缓冲区
 * @param - len:  要读取的数据长度
 * @return 		: 0成功，负值失败
 */
static int v53l1x_read_regs_16addr(struct v53l1x_dev *dev, u16 reg, void *val, int len)
{
    int ret;
    struct i2c_msg msg[2];
    struct i2c_client *client = dev->private_data;
    u8 addr_buf[2] = {reg >> 8, reg & 0xFF}; // 高字节在前

    /* 消息1: 发送16位寄存器地址 */
    msg[0].addr = client->addr;
    msg[0].flags = 0; // 写操作
    msg[0].buf = addr_buf;
    msg[0].len = 2;

    /* 消息2: 读取数据 */
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD; // 读操作
    msg[1].buf = val;
    msg[1].len = len;

    ret = i2c_transfer(client->adapter, msg, 2);
    if (ret != 2)
    {
        printk("i2c read failed @ 0x%04X, ret=%d\n", reg, ret);
        return -EIO;
    }
    return 0;
}

/*
 * @description	: 向v53l1x指定寄存器处写入多个数据（支持16位地址）
 * @param - dev:  v53l1x设备
 * @param - reg:  16位寄存器地址
 * @param - buf:  要写入的数据缓冲区
 * @param - len:  要写入的数据长度
 * @return 	  :   0成功，负值失败
 */
static s32 v53l1x_write_regs_16addr(struct v53l1x_dev *dev, u16 reg, u8 *buf, u8 len)
{
    struct i2c_client *client = dev->private_data;
    u8 *tx_buf;
    int ret;
    struct i2c_msg msg;

    /* 分配缓冲区：地址(2字节) + 数据 */
    tx_buf = kmalloc(len + 2, GFP_KERNEL);
    if (!tx_buf)
        return -ENOMEM;

    /* 构建寄存器地址（高字节在前） */
    tx_buf[0] = reg >> 8;   // 高字节
    tx_buf[1] = reg & 0xFF; // 低字节

    /* 复制数据 */
    memcpy(&tx_buf[2], buf, len);

    msg.addr = client->addr;
    msg.flags = 0; // 写操作
    msg.buf = tx_buf;
    msg.len = len + 2;

    ret = i2c_transfer(client->adapter, &msg, 1);
    kfree(tx_buf);

    if (ret != 1)
    {
        printk("i2c write failed @ 0x%04X, ret=%d\n", reg, ret);
        return -EIO;
    }
    return 0;
}

/*
 * @description	: 读取v53l1x指定寄存器值（支持16位地址）
 * @param - dev:  v53l1x设备
 * @param - reg:  16位寄存器地址
 * @return 	  :   读取到的寄存器值
 */
static u8 v53l1x_read_reg_16addr(struct v53l1x_dev *dev, u16 reg)
{
    u8 data = 0;
    if (v53l1x_read_regs_16addr(dev, reg, &data, 1))
    {
        printk("Failed to read reg 0x%04X\n", reg);
    }
    return data;
}

/*
 * @description	: 向v53l1x指定寄存器写入指定的值（支持16位地址）
 * @param - dev:  v53l1x设备
 * @param - reg:  16位寄存器地址
 * @param - data: 要写入的值
 * @return   :    无
 */
static void v53l1x_write_reg_16addr(struct v53l1x_dev *dev, u16 reg, u8 data)
{
    v53l1x_write_regs_16addr(dev, reg, &data, 1);
}

void v53l1x_readdata(struct v53l1x_dev *dev)
{
    u8 temp, InterruptPolarity, isDataReady = 0, RgSt = 255;
    u8 result_temp[17];

    /*查看中断极性*/
    temp = v53l1x_read_reg_16addr(dev, GPIO_HV_MUX__CTRL);
    temp = temp & 0x10;               // 取出中断极性位
    InterruptPolarity = !(temp >> 4); // 0为低电平触发，1为高电平触发

    /*读取GPIO__TIO_HV_STATUS寄存器*/
    temp = v53l1x_read_reg_16addr(dev, GPIO__TIO_HV_STATUS);
    if ((temp & 1) == InterruptPolarity)
        isDataReady = 1;
    else
        isDataReady = 0;

    /*数据处理结果*/
    if (isDataReady)
    {
        /*读取测距结果寄存器*/
        v53l1x_read_regs_16addr(dev, VL53L1_RESULT__RANGE_STATUS, result_temp, 17);
        RgSt = result_temp[0] & 0x1F;
        if (RgSt < 24)
            RgSt = status_rtn[RgSt];
        v53l1xresult.Status = RgSt;
        v53l1xresult.Ambient = (result_temp[7] << 8 | result_temp[8]) * 8;
        v53l1xresult.NumSPADs = result_temp[3];
        v53l1xresult.SigPerSPAD = (result_temp[15] << 8 | result_temp[16]) * 8;
        printk("[13]=%02X, [14]=%02X\n", result_temp[13], result_temp[14]);
        v53l1xresult.Distance = result_temp[13] << 8 | result_temp[14];

        printk("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
               v53l1xresult.Status, v53l1xresult.Distance, v53l1xresult.Ambient,
               v53l1xresult.SigPerSPAD, v53l1xresult.NumSPADs);
        dev->distance_mm = v53l1xresult.Distance; // 更新设备距离值
    }

    /*清除中断标志,触发下次测量*/
    v53l1x_write_reg_16addr(dev, SYSTEM__INTERRUPT_CLEAR, 0x01); // 清除中断
    if (first_range)
    {
        /*第一次测量将被忽略，因此需要两次调用*/
        v53l1x_write_reg_16addr(dev, SYSTEM__INTERRUPT_CLEAR, 0x01); // 清除中断
        first_range = 0;
    }
}

/*
 * @description		: 打开且初始化设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int v53l1x_open(struct inode *inode, struct file *filp)
{
#if defined(LONG_RANGE_MODE)
    u8 long_range_config_buff[4] = {0x07, 0x05, 0x06, 0x06};
    u8 TB_config_buff[24] = {0x00, 0x1E, 0x00, 0x22,
                             0x00, 0x60, 0x00, 0x6E,
                             0x00, 0xAD, 0x00, 0xC6,
                             0x01, 0xCC, 0x01, 0xEA,
                             0x02, 0xD9, 0x02, 0xF8,
                             0x04, 0x8F, 0x04, 0xA4};

#else
    u8 short_range_config_buff[4] = {0x0F, 0x0D, 0x0E, 0x0E};
    u8 TB_config_buff[28] = {0x00, 0x1D, 0x00, 0x27,
                             0x00, 0x51, 0x00, 0x6E,
                             0x00, 0xD6, 0x00, 0x6E,
                             0x01, 0xAE, 0x01, 0xE8,
                             0x02, 0xE1, 0x03, 0x88,
                             0x03, 0xE1, 0x04, 0x96,
                             0x05, 0x91, 0x05, 0xC1};
#endif

    u8 sys_boot_status = 5, IntPol, Temp, isDataReady, TB_config_status = 0;
    u8 tx_Init_Config_buff[91], check_Init_Config_buff[91], rxdata_DeviceID[2], temp_for_TB_buff[2], inner_config_buff[4], ClockPLL[2];
    u8 wait_timeout = 10;
    u16 temp_for_TB, TB;
    u32 measure_period;
    filp->private_data = &v53l1xdev;

    /*检查设备ID以确认iic通信是否正常*/
    v53l1x_read_regs_16addr(&v53l1xdev, VL53L1_IDENTIFICATION__MODEL_ID, rxdata_DeviceID, 2); // 读取设备ID
    printk("Device ID = 0x%06X\n", rxdata_DeviceID[0] << 8 | rxdata_DeviceID[1]);             // 应为0xEACC

    do
    {
        sys_boot_status = v53l1x_read_reg_16addr(&v53l1xdev, VL53L1_FIRMWARE__SYSTEM_STATUS); /* 读取设备系统状态，正常则返回0 */
        printk("waiting for device to be ready\r\n");
        wait_timeout--;
        mdelay(2);
        if (wait_timeout == 0)
        {
            printk("v53l1x open timeout!\n");
            return -EIO; /* 如果设备状态不正常，返回错误 */
        }
    } while (sys_boot_status == 0);
    printk("v53l1x system status = 0x%02X,open sucess\n", sys_boot_status);

    /*写入初始化配置，使用ST官方给出的寄存器配置*/
    memcpy(tx_Init_Config_buff, VL51L1X_DEFAULT_CONFIGURATION, sizeof(VL51L1X_DEFAULT_CONFIGURATION));
    v53l1x_write_regs_16addr(&v53l1xdev, 0x002D, tx_Init_Config_buff, sizeof(VL51L1X_DEFAULT_CONFIGURATION)); // 进行初始化配置
    mdelay(100);                                                                                              // 等待100ms
    // for (i = 0; i < 91; i++)
    // {
    //     check_Init_Config_buff[i] = v53l1x_read_reg_16addr(&v53l1xdev, 0x002D + i); // 读取配置数据进行校验
    //     mdelay(5);                                                                  // 每个寄存器读取后等待5ms
    // }
    v53l1x_read_regs_16addr(&v53l1xdev, 0x002D, check_Init_Config_buff, sizeof(VL51L1X_DEFAULT_CONFIGURATION)); // 读取系统状态寄存器

    if (memcmp(tx_Init_Config_buff, check_Init_Config_buff, sizeof(VL51L1X_DEFAULT_CONFIGURATION)) != 0)
    {
        printk("v53l1x configuration write failed!\n");
        return -EIO; /* 如果配置不一致，返回错误 */
    }
    else
    {
        printk("v53l1x configuration write complete!\n");
    }

    /*启动一次测量以校准*/
    v53l1x_write_reg_16addr(&v53l1xdev, SYSTEM__MODE_START, 0x40); // 启动测距模式

    /*检查是否有新数据产生*/
    // 1.查看中断寄存器的极性
    IntPol = v53l1x_read_reg_16addr(&v53l1xdev, GPIO_HV_MUX__CTRL);
    // 2.查看引脚状态
    while (!isDataReady)
    {
        Temp = v53l1x_read_reg_16addr(&v53l1xdev, GPIO__TIO_HV_STATUS);
        if ((Temp & 1) == IntPol)
            isDataReady = 1;
        else
            isDataReady = 0;
        mdelay(100); // 等待100ms
        wait_timeout--;
        if (wait_timeout == 0)
        {
            printk("wait for data ready timeout!\n");
            return -EIO; /* 如果超时，返回错误 */
        }
    }

    /*清除中断标志*/
    v53l1x_write_reg_16addr(&v53l1xdev, SYSTEM__INTERRUPT_CLEAR, 0x01); // 清除中断标志

    /*停止测量*/
    v53l1x_write_reg_16addr(&v53l1xdev, SYSTEM__MODE_START, 0x00); // 停止测距模式

    /*VHV超时宏循环边界配置*/
    v53l1x_write_reg_16addr(&v53l1xdev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); // 设置VHV超时宏循环边界,2个边界VHV

    /*从上一温度开始启动 VHV，未查明*/
    v53l1x_write_reg_16addr(&v53l1xdev, 0x0B, 0); // 启动VHV

    printk("v53l1x preinit success!\n");

    /*获取TB值，未查明*/
    v53l1x_read_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, temp_for_TB_buff, 2);
    temp_for_TB = (temp_for_TB_buff[0] << 8) | temp_for_TB_buff[1]; // 组合高低字节
    switch (temp_for_TB)
    {
    case 0x001D:
        TB = 15;
        break;
    case 0x0051:
    case 0x001E:
        TB = 20;
        break;
    case 0x00D6:
    case 0x0060:
        TB = 33;
        break;
    case 0x1AE:
    case 0x00AD:
        TB = 50;
        break;
    case 0x02E1:
    case 0x01CC:
        TB = 100;
        break;
    case 0x03E1:
    case 0x02D9:
        TB = 200;
        break;
    case 0x0591:
    case 0x048F:
        TB = 500;
        break;
    default:
        TB = 0;
        printk("TB value not recognized, using default 20ms.\n");
    }
#if defined(LONG_RANGE_MODE)
    /*设置距离模式*/
    v53l1x_write_reg_16addr(&v53l1xdev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    v53l1x_write_regs_16addr(&v53l1xdev, SD_CONFIG__WOI_SD0, long_range_config_buff, 2);
    v53l1x_write_regs_16addr(&v53l1xdev, SD_CONFIG__INITIAL_PHASE_SD0, &long_range_config_buff[2], 2);
    /*配置TB*/
    switch (TB)
    {
    case 20:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 2, 2);
        break;
    case 33:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 4, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 6, 2);
        break;
    case 50:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 8, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 10, 2);
        break;
    case 100:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 12, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 14, 2);
        break;
    case 200:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 16, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 18, 2);
        break;
    case 500:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 20, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 22, 2);
        break;
    default:
        TB_config_status = 1;
        break;
    }
    if (TB_config_status)
    {
        printk("TB value %d is not supported by long-mode, using default configuration.\n", TB);
        return -EINVAL; // 如果TB值不支持，返回错误
    }

#else
    /*设置距离模式*/
    v53l1x_write_reg_16addr(&v53l1xdev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
    v53l1x_write_reg_16addr(&v53l1xdev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    v53l1x_write_regs_16addr(&v53l1xdev, SD_CONFIG__WOI_SD0, short_range_config_buff, 2);
    v53l1x_write_regs_16addr(&v53l1xdev, SD_CONFIG__INITIAL_PHASE_SD0, &short_range_config_buff[2], 2);
    /*配置TB*/
    switch (TB)
    {
    case 15:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 2, 2);
        break;
    case 20:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 4, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 6, 2);
        break;
    case 33:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 8, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 10, 2);
        break;
    case 50:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 12, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 14, 2);
        break;
    case 100:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 16, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 18, 2);
        break;
    case 200:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 20, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 22, 2);
        break;
    case 500:
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, TB_config_buff + 24, 2);
        v53l1x_write_regs_16addr(&v53l1xdev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, TB_config_buff + 26, 2);
        break;
    default:
        TB_config_status = 1;
        break;
    }
    if (TB_config_status)
    {
        printk("TB value %d is not supported by short-mode, using default configuration.\n", TB);
        return -EINVAL; // 如果TB值不支持，返回错误
    }
#endif

    /*设置内部参数*/
    v53l1x_read_regs_16addr(&v53l1xdev, VL53L1_RESULT__OSC_CALIBRATE_VAL, ClockPLL, 2); // 读取时钟PLL值
    measure_period = (u32)((u16)((ClockPLL[0] & 0x03) | ClockPLL[1]) * 1075 / 10);      // measure_period若为0x0000 0F1E
    printk("ClockPLL = 0x%02X%02X, measure_period = %d ms\n", ClockPLL[0], ClockPLL[1], measure_period);
    inner_config_buff[0] = (measure_period >> 24) & 0xFF; // 0x00
    inner_config_buff[1] = (measure_period >> 16) & 0xFF; // 0x00
    inner_config_buff[2] = (measure_period >> 8) & 0xFF;  // 0x0F
    inner_config_buff[3] = measure_period & 0xFF;         // 0x1E
    v53l1x_write_regs_16addr(&v53l1xdev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, inner_config_buff, 4);

    /*开始测量*/
    v53l1x_write_reg_16addr(&v53l1xdev, SYSTEM__MODE_START, 0x40); // 启动测距模式

    return 0;
}

/*
 * @description		: 从设备读取数据
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t v53l1x_read(struct file *filp, char __user *buf, size_t cnt, loff_t *off)
{
    u16 data[1];
    long err = 0;

    struct v53l1x_dev *dev = (struct v53l1x_dev *)filp->private_data;

    v53l1x_readdata(dev);

    data[0] = dev->distance_mm; // 将设备距离值存入data数组

    err = copy_to_user(buf, data, sizeof(data));
    return 0;
}

/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int v53l1x_release(struct inode *inode, struct file *filp)
{
    return 0;
}

/* v53l1x操作函数 */
static const struct file_operations v53l1x_ops = {
    .owner = THIS_MODULE,
    .open = v53l1x_open,
    .read = v53l1x_read,
    .release = v53l1x_release,
};

/*
 * @description     : i2c驱动的probe函数，当驱动与
 *                    设备匹配以后此函数就会执行
 * @param - client  : i2c设备
 * @param - id      : i2c设备ID
 * @return          : 0，成功;其他负值,失败
 */
static int v53l1x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    /* 1、构建设备号 */
    if (v53l1xdev.major)
    {
        v53l1xdev.devid = MKDEV(v53l1xdev.major, 0);
        register_chrdev_region(v53l1xdev.devid, V53L1X_CNT, V53L1X_NAME);
    }
    else
    {
        alloc_chrdev_region(&v53l1xdev.devid, 0, V53L1X_CNT, V53L1X_NAME);
        v53l1xdev.major = MAJOR(v53l1xdev.devid);
    }

    /* 2、注册设备 */
    cdev_init(&v53l1xdev.cdev, &v53l1x_ops);
    cdev_add(&v53l1xdev.cdev, v53l1xdev.devid, V53L1X_CNT);

    /* 3、创建类 */
    v53l1xdev.class = class_create(THIS_MODULE, V53L1X_NAME);
    if (IS_ERR(v53l1xdev.class))
    {
        return PTR_ERR(v53l1xdev.class);
    }

    /* 4、创建设备 */
    v53l1xdev.device = device_create(v53l1xdev.class, NULL, v53l1xdev.devid, NULL, V53L1X_NAME);
    if (IS_ERR(v53l1xdev.device))
    {
        return PTR_ERR(v53l1xdev.device);
    }

    v53l1xdev.private_data = client;

    return 0;
}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client 	: i2c设备
 * @return          : 0，成功;其他负值,失败
 */
static int v53l1x_remove(struct i2c_client *client)
{
    /* 删除设备 */
    cdev_del(&v53l1xdev.cdev);
    unregister_chrdev_region(v53l1xdev.devid, V53L1X_CNT);

    /* 注销掉类和设备 */
    device_destroy(v53l1xdev.class, v53l1xdev.devid);
    class_destroy(v53l1xdev.class);
    return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id v53l1x_id[] = {
    {"lijiaxin,v53l1x", 0},
    {}};

/* 设备树匹配列表 */
static const struct of_device_id v53l1x_of_match[] = {
    {.compatible = "lijiaxin,v53l1x"},
    {/* Sentinel */}};

/* i2c驱动结构体 */
static struct i2c_driver v53l1x_driver = {
    .probe = v53l1x_probe,
    .remove = v53l1x_remove,
    .driver = {
        .owner = THIS_MODULE,
        .name = "v53l1x",
        .of_match_table = v53l1x_of_match,
    },
    .id_table = v53l1x_id,
};

/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init v53l1x_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&v53l1x_driver);
    return ret;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit v53l1x_exit(void)
{
    i2c_del_driver(&v53l1x_driver);
}

/* module_i2c_driver(v53l1x_driver) */

module_init(v53l1x_init);
module_exit(v53l1x_exit);

MODULE_LICENSE("GPL");
MODULE_VERSION("V3.0");
MODULE_AUTHOR("lijiaxin 2687025869@qq.com");
MODULE_DESCRIPTION("Built on June 25, 2025. Based on ST's kernel module, modified to suit the loongarch platform.");
