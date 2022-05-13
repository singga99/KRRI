### 1번 PC - Bag 실행

Domain 설정

``` bash
export ROS_DOMAIN_ID=5
```

Terminal (1) - bag file & GUI

``` bash
cd /home/krri/bag_manager
python run_check.py
```

Terminal (2) - SharedMemory

```
cd dev_ws
. install/setup.bash
ros2 run brt_check alive_counter
```

d

#### 1번 PC 실행 순서

``` 
bag file & GUI -> SharedMemory
```



#### ※ 확인 ※ 

##### GUI 꺼줄 때, 모든 버튼 OFF로 해놓고 close하기





### 2번 PC

##### Domain Bridge (ID=5 <-> ID=3)  >> 사용 X

```
export ROS_DOMAIN_ID=5
```

Terminal(1) - domain bridge

```
cd dev_ws
. install/setup.bash
cd /home/krri/dev_ws/src/domain_bridge
ros2 run domain_bridge comain_brige bridge.yaml
```



Domain 설정

```
export ROS_DOMAIN_ID=5
```

Terminal (2) - 스프링클라우드 code

```
# camera detection
sros2 (source ros bash file)
cd /home/krri/colcon_ws/src/krri_detections/krri_detections
python3 cam_detection.py
python3 cam_detection_none.py (another terminal)

# lidar-calibrator
cd ~/colcon_ws
export WORKSPACE_DIR=`pwd` 
sros2
ros2 launch multi_lidar_calibrator multi_lidar_calibrator_integrated.BAG.launch.py

# lidar detection
sros2
cd /home/krri/colcon_ws/src/sfa/sfa
python3 krri_demo.py

# rviz2
rviz2 -d rviz/multi_lidar_calibrator_demo.rviz
```



##### audio 연결

Terminal (3) - status_alert

```
cd dev_ws
. install/setup.bash
cd /home/krri/dev_ws/src/bit_unpack/bit_unpack
python3 unpack.py
```



##### 이완상태 확인

Terminal (4) - fsm manager

```
cd dev_ws
. install/setup.bash
ros2 run brt_check fsm_control
```

Terminal (5) - stop control

```
cd dev_ws
. install/setup.bash
ros2 run brt_check stop_control
```



#### 실행결과

##### 1번 PC

```
[ System Condition ]
sensor 버튼(camera, lidar)

[ Control System ]
- Control : CANID52 데이터 ON/OFF
- Autonomous approve : 자율주행 승인 버튼
- Warning : 관제 경고 버튼
```



##### 2번 PC

```
fsm manager, stop control, rqt 실행
sensor 버튼 모두 누른 후, rqt의 Autonomous ON/OFF 클릭 -> 자율주행 ON/OFF

[ 비상운전 -> 수동운전 ]
- Autonomous approve = True, Autonomous = False, sensor 모두 ON
```



