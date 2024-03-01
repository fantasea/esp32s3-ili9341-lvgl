#  蜂鸣器

## 组件的功能函数
1. buzzer_init() 用于初始化蜂鸣器，需要在初始化中改变传参来改变蜂鸣器的引脚。
2. buzzer()用于蜂鸣器发声，需要传参，参数有5个。
   * 音调：buzzer.h中piano_note_t里的音调
   * 响度：可根据0~8191的来调节响度
   * 响的时间，单位（秒）
   * 不响的时间，单位（秒）
   * 持续响与不响的次数，单位（次）

##  Kconfig
Kconfig放在buzzer组件的根目录下。
可在menuconfig里的buzzer中的Buzzer Pin中修改蜂鸣器引脚，默认是GPIO14

##  测试样例
1. 测试代码在buzzer路径下的example文件夹下
2. 如果要进行测试，可单独将当成一个项目来测试效果。
3. 实现的效果是蜂鸣器响、不响5个周期，然后不响5秒，在进行下一个响、不响的周期。