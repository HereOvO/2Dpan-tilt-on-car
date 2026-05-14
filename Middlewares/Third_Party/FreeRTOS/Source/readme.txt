每个 FreeRTOS 实时内核移植通常由两类文件组成：

1. 所有移植共用的核心内核文件
2. 与具体微控制器或编译器相关的移植文件

+ `FreeRTOS/Source` 目录包含所有移植都会使用的公共内核文件：
  `list.c`、`queue.c` 和 `tasks.c`。
  FreeRTOS 内核的核心功能主要由这三个文件实现。

+ `croutine.c` 实现的是可选的协程功能。
  该功能通常只在 RAM 非常紧张的系统中才会使用。

+ `FreeRTOS/Source/Portable` 目录包含与具体微控制器或编译器相关的移植文件。

+ `FreeRTOS/Source/include` 目录包含实时内核头文件。

如果需要查看更多与移植相关的信息，请继续查看 `FreeRTOS/Source/Portable` 目录中的说明文件。
