### 说明

本项目是基于正点原子的项目模板进行开发创作。

### 功能

通过 `STM32F103` 驱动 `MAX31856` 读取热电偶温度传感器的温度值；并在 LCD 屏上进行显示。

（可将所有LCD显示信息通过串口发出）

### 接线 

```
MCU.PA4 ---> MAX31856.CS
MCU.PA5 ---> MAX31856.CLK
MCU.PA6 ---> MAX31856.MISO
MCU.PA7 ---> MAX31856.MOSI
```

### 文件说明

1. `keilkill.bat` 清理编译链接过程产生的中间文件；
2. `HARDWARE` 文件夹下是自己写的一些代码；
3. `0相关资料` 文件夹下是电路原理图以及芯片相关资料。