# robit_ros2_homework


## 노드 실행 방법

1.  ROS 2와 관련한 실행파일 라이브러리, 패키지 등의 사용을 위한 ROS2 환경 source 하기
   
```
 source /opt/ros/humble/setup.bash
```

2. Cmake 패키지 생성하기
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```
`ros2 pkg create`  : 패키지를 생성할 때 사용하는 명령어
`--build-type ament_cmake`: 패키지를 어떤 빌드 시스템으로 사용할지 결정하는 옵션으로 CMake 기반으로 한 시스템 사용을 위해 ament_cmake로 지정함. 
`--node-name my_node`  : 노드의 이름을 지정하는 옵션. my_node라는 이름으로 노드 생성.
`my_package`  : 생성할 패키지의 이름. 

3. 작업공간으로 이동하여 빌드 진행.
```
cd ~/ros2_ws // 작업 공간으로 이동
colcon build //작업 공간 내의 모든 패키지 빌드

colcon build --packages-select my_package // 특정 패키지 빌드시 사용되는 명령어
```

`colcon build`  : 패키지 빌드 명령어.
`--packages-select` : 일부 패키지만 선택하여 빌드할 수 있도록 함.


4. 작업 공간의 설치된 패키지 및 라이브러리 사용을 위한 설정
```
source ~/ros2_ws/install/setup.bash
```

`~/ros2_ws/install/setup.bash` : 작업 공간에서 패키지를 빌드 후 설치된 결과물이 저장된 디렉터리의 파일을 가리킴.
                                 빌드 된 명령어의 패키지를 install 폴더에 설치하고 setup.bash 파일을 사용하여 패키지의
                                 경로, 실행 파일, 라이브러리등을 사용할 수 있도록 함.

5. 특정 노드 실행
```
ros2 run turtlesim turtlesim_node //특정 노드를 실행하는 명령어로 turtlesim 노드를 실행.

```
```
ros2 run my_package my_nod //특정 노드 실행, 제작한 노드 실행.
```

`ros2 run`  : 노드를 실행할 때 사용하는 명령어. 
              ros2 run <패키지명> <노드명> 형식으로 사용함.

   `turtlesim`  : ROS 2의 Turtlesim 패키지. 터틀을 조작할 수 있는 시뮬레이션 환경 사용을 위해 실행함.

