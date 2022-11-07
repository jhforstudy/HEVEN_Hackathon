# HEVEN Hackathon

### 2022 HEVEN 자율주행 해커톤
- 일정 : 2022-11-11(금) 16:00 ~ 2022-11-12(토) 10:00
- 장소 : 성균관대학교 산학협력관 러닝팩토리 (85133호)
- 인원 : 약 30명 내외 (4인 1팀 구성)
- 내용 : 4인 1팀이 되어 주어진 map 상에서 자율주행 및 주차, 정지선 알고리즘 구현

## 설치
### 1. Docker를 이용한 개발 환경 구성 (Recommended)

Windows 환경에서도 다음의 링크를 참고하여 편하게 개발할 수 있습니다.
[Docker 환경 설치하기](https://github.com/jhforstudy/HEVEN_Hackathon/blob/master/InstallDocker.md)

### 2. Ubuntu에 패키지 직접 설치

* Ubuntu 18.04 멀티부팅 설치

https://jimnong.tistory.com/676

* ROS 설치 (상단의 *"melodic"* 클릭 후 진행)

http://wiki.ros.org/Installation/Ubuntu

* Dependencies 설치

    ```
    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server
    ```

* ROS용 워크스페이스 생성
    ```
    mkdir catkin_ws && cd catkin_ws
    mkdir src && cd src
    ```
    
* 레포지토리 복제
    ```
    git clone https://github.com/jhforstudy/HEVEN_Hackathon.git
    ```

* 패키지 빌드
    ```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```
    
## 실행

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
## 센서 값 호출 방법

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
