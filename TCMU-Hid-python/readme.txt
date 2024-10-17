Company: 广东大镓传感技术有限公司
Version: V1.0
DateTime: 2024/2/23
Author: YouYou
注意：
请先使用pip执行安装hidapi
pip install hidapi
本例子基于python v3.8.1测试通过
电脑插入单路USB测温仪后，运行此python文件可以实现读取温度值
1S读取一次温度值
对于多路测温仪同样操作，读取多个温度通道的寄存器解析为温度