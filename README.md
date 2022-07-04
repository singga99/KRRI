# KRRI

KRRI 관련 공부 & 문서 & Tool



### 1. Ubuntu

##### Ubuntu 환경변수 자동 적용

* open terminal

```
$ gedit /etc/bash.bashrc
```

* 맨 위나 맨 아래에 적용하고 싶은 환경변수 적어주기

* 저장 후, 터미널에서

```
$ source ~/.bashrc
```

* 새로운 터미널 열었을 때도 환경변수 적용되는지 확인



### 2. ROS

##### New Package 만들기

참고 사이트 : https://hostramus.tistory.com/112

* publish와 subscribe의 기초적인 부분은 위 사이트 참고



##### Ros 적용

```
$ . /opt/ros/galactic/setup.bash
$ . install/setup.bash
```

* .은 source와 동일
* . install/setup.bash : 해당 폴더에 빌드 적용



##### Ros 실행 방법

```
$ ros2 run <package> <executable>
```

* 특정 패키지의 특정 노드 실행(1개의 노드)

```
$ ros2 topic echo /<topic name>
```

* 지정 토픽의 데이터 출력
* rqt에서 확인하는 것보다 빠름

```
$ ros2 bag play <bag file>
$ ros2 bag record /<topic name> or -a
```

* paly : 지정 bag file 재생
* record : bag 기록
* -a : 특정 topic만 기록하지 않고 전부 기록



##### RQT

* GUI 프레임 워크
* 다양한 목적의 GUI 툴을 모다운 ROS의 종합 GUI 툴박스
* 현재 ros 환경내에서 발행중인 topic들 확인 가능



##### Convert rosbag versions

참고 사이트 : https://ternaris.gitlab.io/rosbags/topics/convert.html

```
$ pip install rosbags
```

* terminal에 아래와 같이 작성

```
$ rosbags-convert <rosbag file>
```



##### Launch

* 하나 이상의 정해진 노드를 실행시키는 명령어
* launch_test.py 작성

```
$ cd /home/krri/shin_dev/test_lauch
$ mkdir launch
$ touch launch_test.py
```

```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

* launch_test.py 실행

```
$ ros2 launch launch_test.py
```



