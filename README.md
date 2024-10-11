# robit_ros2_homework


## 노드 실행 방법

1.  ROS2 환경 source 하기

   
```
 source /opt/ros/humble/setup.bash
```

2. Cmake 패키지 생성하기
```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 --node-name my_node my_package
```

3. 작업공간으로 이동하여 빌드 진행.
```
cd ~/ros2_ws
colcon build

// colcon build --packages-select my_package
```

4. 작업 공간의 설치된 패키지 및 라이브러리 사용을 위한 환경 변수 설정
```
source ~/ros2_ws/install/setup.bash
```

5. 특정 노드 실행
```
ros2 run turtlesim turtlesim_node

```
```
ros2 run my_package my_nod
```


