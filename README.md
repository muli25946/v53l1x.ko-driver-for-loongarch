+ 该驱动参考ST公司给出的驱动(https://www.st.com.cn/zh/imaging-and-photonics-solutions/vl53l1x.html#tools-software) ，但原驱动是linux内核驱动，通过在用户空间使用封装好的iic API接口与v53l1x通信。该仓库仅是基于ST公司驱动修改，改为.ko驱动，移植更方便。请注意开源声明。
+ 仓库中的v53l1x.c为驱动源文件，v53l1x_reg.h为寄存器地址表(注释可能不准确，注意辨别)和ST配置数组定义。
  + 单次测试:v53l1xAPP.c为测试程序，上传至开发板后先insmod加载.ko驱动，查看/dev下是否有设备v53l1x，若有则运行./v53l1xAPP /dev/v53l1x即可看到结果。
  + 建议使用方式:将v53l1x.ko移至/usr/lib/modules/4.19.190+/,在.etc/rc.local/中do_start函数中添加insmod /usr/lib/modules/4.19.190+/v53l1x.ko;在do_stop中添加rmmod v53l1x。这样就可以开机后自动加载内核模块。程序中只需使用open等操作进行读写即可。
	+ 其余的开发板同理，路径可自定义，用法大致同上 
+ 使用该驱动需在内核设备树中自行添加设备节点，示例如下
```C
//iic驱动
//v53l1x驱动 GPIO48--SCL GPIO49--SDA GPIO45--XSHUT
&i2c0{
	status = "okay";
	clock-frequency = <400000>;
	pinctrl-names = "default";
	pinctrl-0 = <&i2c0_mux_m0>;
	
	v53l1x@29{
		status = "okay";
		compatible = "lijiaxin,v53l1x";
		reg = <0x29>;
	};
};
```
+ 对于XSHUT引脚，主要用于低功耗：当该引脚拉低时，v53l1x进入低功耗模式。再次拉高触发硬件复位。相当于关闭电源再上电。写入配置文件之前一定要先复位，否则无法写入。
