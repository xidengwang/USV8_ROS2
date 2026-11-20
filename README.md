# USV8 ROS2代码迁移
## 推送以及拉取
由于AGX本身不方便使用梯子，因此本仓库同时在`github`和`gitee`上进行存储，拉取流程如下：
```python
git clone https://gitee.com/wang-xideng/usv8_-ros2.git # gitee 当中克隆当前仓库
cd usv8_-ros2 # 进入仓库
```
把默认origin修改为gitee，并
```
git remote rename origin gitee
```
则拉取代码为：
```python
git pull gitee main
```
推送代码为
```python
git push gitee main # 推送至gitee
```
网络好的时候（自己笔记本上挂了梯子），可以再添加github的仓库地址进行备份
```python
git remote add origin https://github.com/xidengwang/USV8_ROS2.git
```
推送代码为
```python
git push origin main # 推送至github
```

## traj包
ROS1版本的traj包的读取方式是通过修改python代码，在实际实验时非常麻烦，而且有的时候会忘记代码逻辑，产生错误，因此，ROS2版本的代码对此进行改进：
### 1.轨迹跟踪逻辑的改进