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

## 최신 버전 업데이트

* 아래 명령어를 실행하여, "Already up to date." 라는 문구가 떠야 최신 버전임
    ```
    cd ~/catkin_ws/src/HEVEN_Hackathon/
    git pull
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
    traffic_light = self.db.traffic_light
    remaining_time = self.db.traffic_remaining_time
    park_x_1 = self.parking_x_1
    park_y_1 = self.parking_y_1
    park_yaw_1 = self.parking_yaw_1
    ```

* LiDAR<br>
    ```
    변수명 : self.db.lidar_data
    형태 : list[360]
    값 설명 : 차량 후방을 기준으로 반시계방향으로 측정된 값
    ```
![라이다정보](https://user-images.githubusercontent.com/48710703/200983104-8a88354d-960b-4b2b-970f-fd6531710450.png)

* GPS + IMU (현재 차량의 global position)<br>
    ```
    변수명 : self.db.pose_data
    형태 : list[3]
    값 설명 : [map frame으로부터 차량의 x좌표, map frame으로부터 차량의 y좌표, x축 방향 기준 yaw 각도 (degree)]
    ```
![포즈정보](https://user-images.githubusercontent.com/48710703/200983112-4e640c43-f009-4d51-b6a7-d308253c548e.png)
    
* 신호등 정보 <br>
    ```
    변수명 : self.db.traffic_light
    형태 : "STOP", "LEFT", "RIGHT", "GO"
    값 설명 : 정지 시 "STOP"
              이후 도로 정보에 따라 "LEFT", "RIGHT" (Map 2) "GO" (Map 3)
    ```

* 신호등 남은 시간 <br>
    ```
    변수명 : self.db.traffic_remaining_time
    형태 : float
    값 설명 : 신호등 정보가 "STOP"일 때 남은 시간 (5.0 sec ~ 0.0 sec)
    ```
    
* 주차 공간의 위치 <br>
    ```
    변수명 : self.parking_x_1, self.parking_y_1, self.parking_yaw_1, self.parking_x_2, self.parking_y_2, self.parking_yaw_2
    형태 : float
    값 설명 : 해당하는 주차 공간의 x, y 좌표 및 방향 (degree)
    ```
    
## 맵 번호 

* `self.map_number` 변수를 통해 현재 맵의 번호를 획득할 수 있음. (1, 2, 3)

* 개발 시에는 원하는 맵의 번호를 직접 할당하여 테스트하면 되고, 코드 제출 시 삭제할 것.

![map_number](https://user-images.githubusercontent.com/48710703/200983738-14cd3c83-4cc9-4793-a48b-fddb10162eeb.png)


## 시뮬레이터 사용 방법

### 1. 시뮬레이터 실행

* 실행해야 할 맵에 따라 `Map1.sh`, `Map2.sh`, `Map3.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
![캡처3](https://user-images.githubusercontent.com/48710703/199907347-0ea16bc2-b3c3-4a2b-aaeb-b652642cb594.PNG)

* 시뮬레이터 기능<br>
① 충돌 횟수, 경과 시간 측정<br>
② 충돌 횟수, 경과 시간, 차량 위치 초기화<br>
③ 시뮬레이터 화면<br>
&nbsp;&nbsp;&nbsp;&nbsp;좌클릭 - 화면 회전<br>
&nbsp;&nbsp;&nbsp;&nbsp;우클릭 - 화면 확대<br>
&nbsp;&nbsp;&nbsp;&nbsp;스크롤 클릭 - 화면 이동<br>
④ 시점 설정<br>
&nbsp;&nbsp;&nbsp;&nbsp;기본은 2D view이며 **Orbit(rviz)** 으로 변경 시 3D view 가능
![캡처4](https://user-images.githubusercontent.com/48710703/199908144-21a49b19-d5ba-4ae3-9c8c-605305b7932b.PNG)

### 2. 알고리즘 테스트

* `Brain.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
아래의 문구와 함께 **Brain.py**가 실행됨
![5](https://user-images.githubusercontent.com/48710703/199909682-9c98e999-167f-4233-93a8-761018de8c94.PNG)
                       
### 3. 차량 수동 조작
* `Joystick.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭<br>
아래의 문구와 함께 조이스틱이 실행됨<br>
열린 터미널을 클릭한 후,<br>
&nbsp;&nbsp;&nbsp;&nbsp;좌/우 화살표 - 차량 조향각 왼쪽/오른쪽 증가<br>
&nbsp;&nbsp;&nbsp;&nbsp;상/하 화살표 - 차량 속도 높임/낮춤<br>
&nbsp;&nbsp;&nbsp;&nbsp;스페이스 바 - 차량 정지<br>
&nbsp;&nbsp;&nbsp;&nbsp;탭 키 - 조향각 초기화<br>
**주의** : Brain.sh 와 따로 사용할 것
![캡처](https://user-images.githubusercontent.com/48710703/200274414-608ace90-05d1-4a65-8747-ead89e63efd6.PNG)
