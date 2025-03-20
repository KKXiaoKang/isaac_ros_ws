# isaac-sim 服务端 / 客户端实现
* 此库用于构建isaac-sim的服务端及客户端
* 服务端构建MPC控制器所需的状态信息和控制信息，并且以时间微分的形式插值至500Hz
* 客户端接收控制信息，并且以正确的频率下发给isaac-sim仿真下一步
```bash
├── controller_tcp  # tcp控制器/isaac-sim服务端，用于接收客户端发过来的状态信息/下发控制信息
├── nio-isaac       # issac-sim客户端，用于维护机器人、场景、启动isaac-sim
```
# 架构
* ![架构](./IMG/image.png)

# build
* 编译isaac-sim仿真功能包
```bash
cd ~/kuavo-ros-control/src/
git clone -b beta https://www.lejuhub.com/highlydynamic/kuavo-isaac-sim.git

cd ~/kuavo-ros-control/
catkin build tcp_cpp humanoid_controllers
```
* kuavo-ros-control控制器可用分支如下
```bash
* beta
* opensource/0214_produce
* opensource/CASTest
```

# 启动
### 启动OCS2控制器 | 启动isaac-sim仿真
```bash
roslaunch humanoid_controllers load_kuavo_isaac_sim.launch joystick_type:=bt2pro
``` 
* 如当前目录下没有`load_kuavo_isaac_sim.launch`，请执行如下cherry-pick
```bash
git cherry-pick f256b73e7d8ba5555613818e2318b7e8fa8f9253
```
* 请注意，`omni_python`为isaac-sim的pythonEnv环境索引，指向的目录为如下，请你在本机环境当中一定要正确进行设置到`bashrc`当中
```bash
alias omni_python

alias omni_python='/home/lab/.local/share/ov/pkg/isaac-sim-4.1.0/python.sh'
```
### 关键launch解析
* tcp_cpp_simulate : isaac-sim服务端
* launch.py : isaac-sim客户端，用于维护开启场景
```yaml
<launch>  
    <node pkg="tcp_cpp" name="tcp_cpp_simulate" type="tcp_cpp_simulate" output="screen" />
    <node pkg="tcp_cpp" name="isaac_sim_launcher" type="launch.py" output="screen" />
</launch>  
```