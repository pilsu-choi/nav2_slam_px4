# To use
``` python
$chmod +x build_and_run_docker.sh

$./build_and_run_docker.sh
```

### gpu 환경에 따라 Dockerfile.dev_gpu의 base image, 필요 라이브러리가 바뀔 수 있습니다. 해당 부분 확인하고 필요에 따라 변경해주세요.
- 현재 RTX 4070 기준으로 작성

### 필요에 따라 rtabmap.db가 필요하다면 rtabmap_pull.sh을 실행하세요.
```
$chmod +x rtabmap_pull.sh 
$./rtabmap_pull.sh
```

## when you finish docker build, nav2_slam_px4 build
```
$docker exec -it nav2_slam_px4_container bash

$cd /home/ubuntu/nav2_slam_px4 && colcon build
```


## try - use 4 terminals

terminal 1
```
$docker exec -it nav2_slam_px4_container bash

$cd ~/ && ./QGroundControl-x86_64.AppImage #qgroundcontrol 실행

```

terminal 2
```
$docker exec -it nav2_slam_px4_container bash

$cd ~/Micro-XRCE-DDS-Agent/ && MicroXRCEAgent udp4 -p 8888 #마이크로 XRCE 실행

```

terminal 3
```
$docker exec -it nav2_slam_px4_container bash

$cd ~/PX4-Autopilot && PX4_GZ_WORLD=turtlebot3_world make px4_sitl gz_x500_rtab #가제보 and px4 실행

```

terminal 4
```
$docker exec -it nav2_slam_px4_container bash

$cd ~/nav2_slam_px4 && export GZ_SIM_RESOURCE_PATH=/home/ubuntu/PX4-Autopilot/Tools/simulation/gz/models && source install/setup.bash && ros2 launch rtabmap_nav2_px4 bringup.launch.py use_sim_time:=true


```
