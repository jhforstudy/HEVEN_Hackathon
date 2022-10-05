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
