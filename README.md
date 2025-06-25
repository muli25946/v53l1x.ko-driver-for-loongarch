该程序大量参考ST公司给出的驱动，但ST公司给出的是内核驱动，且在用户空间进行iic通信。该仓库仅是基于ST公司驱动改为.ko驱动。请注意开源声明。
使用该驱动需在内核设备树中自行添加设备节点，示例如下
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
		xshut-gpios = <&gpa2 13 GPIO_ACTIVE_HIGH>;// GPIO045
	};
};

对于XSHUT引脚，主要用于低功耗：当该引脚拉低时，v53l1x进入低功耗模式。再次拉高触发硬件复位。相当于关闭电源再上电。写入配置文件之前一定要先复位，否则无法写入。
