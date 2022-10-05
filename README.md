# HEVEN Hackathon

2022 HEVEN 자율주행 해커톤

### 설치 방법

* Dependencies 설치

    ```
    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server
    ```
    
### 패키지 설치

* 레포지토리 복제
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/jhforstudy/HEVEN_Hackathon.git
    ```

* 패키지 빌드
    ```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```
    
### 실행

* 시뮬레이터 실행
    ```
    roslaunch racecar_simulator simulate.launch
    ```
    
* 자율주행 main node 실행
    ```
    rosrun racecar_simulator main.py
    ```

* 수동 조작 노드 실행 (시뮬레이터 실행 후)
    ```
    rosrun racecar_simulator teleop_keyboard.py
    ```
### 센서 값 호출 방법

* Brain.py에서 다음과 같이 호출하여 사용할 수 있음
    ```python
    lidar_data = self.db.lidar_data
    pose_data = self.db.pose_data
    ```

* LiDAR<br>
    ```
    변수명 : self.db.lidar_data
    형태 : list[360]
    값 설명 : 차량 전방, 우측을 기준으로 반시계방향으로 측정된 값
    ```

* pose (현재 차량의 global position)<br>
    ```
    변수명 : self.db.pose_data
    형태 : list[3]
    값 설명 : [map frame으로부터 차량의 x좌표, map frame으로부터 차량의 y좌표, yaw (degree)]
    ```
