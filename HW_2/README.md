# robit_ros2_homework


## 노드 실행 방법

1. 빌드
```
cd ~/hw //특정 파일 위치로 이동

// colcon build // 모든 패키지 한 번에 빌드.

colcon build --packages-select my_interfaces //특정 패키지 빌드.
colcon build --packages-select my_publisher
colcon build --packages-select my_subscriber

```

2. 작업 공간 환경 설정 명령어.
   => 해당 작업 공간에 설치된 패키지와 관련한 환경 변수 설정.

```
source ~/hw/install/setup.bash
```

3. 노드실행.
```
// 각각 다른 터미널에서 세팅 후 노드 실행.
ros2 run my_publisher publisher_node 
ros2 run my_subscriber subscriber_node
```
