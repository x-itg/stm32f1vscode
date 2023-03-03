##### 一、工程搭建

- makefile文件由stm32cube生成后修改

- 事先安装的软件：vscode cortex-debug插件；tup-latest、 openocd 、arm-gdb/gcc 、GNU MCU Eclipse安装到C:\Program Files文件夹，它们的bin文件夹路径加入到path环境变量当中。

- 修改的stm32f1discovery.cfg文件放到C:\Program Files\OpenOCD\0.10.0-13\scripts\board的文件夹；这个make upload烧入的时候用到


##### 二、编译指令

`make`

##### 三、烧入指令

`make upload`

##### 四、进入调试

`Fn+F5`


