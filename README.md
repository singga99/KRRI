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



##### 단축키

* Ctrl + Alt + T : 새 터미널창 열기
* Ctrl + Shift + E : 터미널창 수직 분할
* Ctrl + Shift + O : 터미널창 수평 분할
* Ctrl + Shift + W : 터미널창 분할 제거
* Ctrl + Alt + W : 창 이름 변경
* Ctrl + C : 터미널내의 실행 파일 종료
* Ctrl + Z : 터미널내의 실행 파일 강제 종료
* cd : 기본 경로 이동
* cd .. : 이전 폴더 이동
* cd < 원하는 폴더 이름 > : 해당 폴더로 이동
* mkdir < 폴더 이름 > : 원하는 폴더 생성
* touch < 파일 이름 > : 원하는 이름으로 파일 생성
* rm < 파일 이름 > : 해당 파일 삭제



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
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	return launch.LaunchDescription([
		launch_ros.actions.Node(
			package='turtlesim',
            		namespace='turtlesim1',
            		executable='turtlesim_node',
			),
		
		launch_ros.actions.Node(
			package='turtlesim',
            		namespace='turtlesim2',
            		executable='turtlesim_node',
			),
		
		# launch
		launch.actions.IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				get_package_share_directory('packagename'),
				'/launch/launchname.launch.py']),
			),
```

* package => 해당 패키지 이름
* executable => setup.py의 pub&sub 노드 이름
* get_package_share_directory => launch 파일이 존재하는 해당 패키지 이름
* launch_test.py 실행

```
$ ros2 launch launch_test.py
```

