# USV8 ROS2代码迁移
### 推送以及拉取
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