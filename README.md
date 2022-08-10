# RL-PPO算法下对阿克曼小车自动泊车的MATLAB仿真

本项目主要包括三部分：<br>
* 1. 垂直泊车的轨迹规划<br>
* 2. 阿克曼小车的泊停过程<br>
* 3. 强化学习下阿克曼小车自动泊车的MATLAB仿真<br>

下面我们将对此分别进行简单的说明，具体介绍请参考各自目录下的相关说明文件<br>

## 垂直泊车轨迹规划与路径探索
[step 1](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/tree/main/step1%E5%80%92%E8%BD%A6%E8%BD%A8%E8%BF%B9%E4%BB%A3%E7%A0%81)
中我们主要对垂直泊车轨迹规划的代码进行了复现，泊车过程主要采用圆弧-直线方式。<br>
具体垂直泊车轨迹请看[step1说明文档](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step1%E5%80%92%E8%BD%A6%E8%BD%A8%E8%BF%B9%E4%BB%A3%E7%A0%81/Vertical%20%20trajectory.pdf)。 <br>

代码所绘制轨迹如下：<br>
![垂直泊车轨迹](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step1%E5%80%92%E8%BD%A6%E8%BD%A8%E8%BF%B9%E4%BB%A3%E7%A0%81/%E5%9E%82%E7%9B%B4%E6%B3%8A%E8%BD%A6%E8%BD%A8%E8%BF%B9.png) <br>

[step 2](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/tree/main/step1%E5%80%92%E8%BD%A6%E8%BD%A8%E8%BF%B9%E4%BB%A3%E7%A0%81) 
中我们根据阿克曼小车的运动学原理复现代码，初步实现了阿克曼小车的指定位置停泊。<br>
具体阿克曼小车垂直泊车过程请看[step2说明文档](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step2%E5%9E%82%E7%9B%B4%E5%80%92%E8%BD%A6%E8%BF%87%E7%A8%8B%E4%BB%A3%E7%A0%81/The%20Parking%20Process%20of%20Ackerman's%20Vehicle.pdf)。 <br>

## 强化学习下阿克曼小车自动泊车的MATLAB仿真
本部分是**暑期项目中的最重要部分**,旨在通过前面的steps深入学习垂直泊车后，以MATLAB软件模拟一辆阿克曼结构的小车运动学参数，实现强化学习下的垂直倒车入库。我们基于前两个项目倒车入库的轨迹和流程，分步实现了PPO算法下的MATLAB仿真。并对强化学习的训练及测试情况进行展示。<br>
具体[step3 强化学习下阿克曼小车自动泊车的MATLAB仿真](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/tree/main/step3%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0%E4%B8%8B%E8%87%AA%E5%8A%A8%E6%B3%8A%E8%BD%A6%E9%A1%B9%E7%9B%AE)的实现过程请看[step3说明文档](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step3%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0%E4%B8%8B%E8%87%AA%E5%8A%A8%E6%B3%8A%E8%BD%A6%E9%A1%B9%E7%9B%AE/Automatic%20Parking%20by%20Reinforcement%20Learning.pdf) 。以下我们对项目过程略作展示。小车在训练过程中，会对选定位置中不断进行倒车尝试。下图所示为训练过程中的训练代理的实时效果图，其训练情况及其拟合效果会实时一并显示： <br>
![训练过程实时展示](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step3%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0%E4%B8%8B%E8%87%AA%E5%8A%A8%E6%B3%8A%E8%BD%A6%E9%A1%B9%E7%9B%AE/%E8%AE%AD%E7%BB%83%E8%BF%87%E7%A8%8B%E5%AE%9E%E6%97%B6%E6%95%88%E6%9E%9C%E5%B1%95%E7%A4%BA.jpg)<br>

我们将现有的拟合情况与
[MathWorks中的拟合情况](https://www.mathworks.com/help/reinforcement-learning/ug/train-ppo-agent-to-land-rocket.html)
进行对比，下图中右侧为本项目情况，左侧为MathWorks中情况：<br>

![训练效果对比](https://github.com/supersteve2001/Automatic-Parking-by-Reinforcement-Learning-PPO/blob/main/step3%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0%E4%B8%8B%E8%87%AA%E5%8A%A8%E6%B3%8A%E8%BD%A6%E9%A1%B9%E7%9B%AE/%E8%AE%AD%E7%BB%83%E6%95%88%E6%9E%9C%E5%AF%B9%E6%AF%94.png)<br>

可以明显看出，我们的拟合情况相较于原有的算法与训练场景具有更好的训练成果。
