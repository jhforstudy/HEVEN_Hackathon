## Docker 환경 구성

#### 1. docker 설치

* 다음의 링크를 참고하여 Docker 및 WSL 2 [설치](https://myjamong.tistory.com/296)

#### 2. Docker image pull 및 실행

* Windows Powershell을 관리자 권한으로 실행
* 터미널 창에 다음을 입력
```
docker run -p 6080:80 -e RESOLUTION=1920x1080 --shm-size=512m jhforstudy/heven_hack:latest
```
자동으로 Dockerhub에서 image를 받아 와서 실행 함.

위의 명령어가 성공적으로 실행된 후,

**Docker Desktop**을 실행하여 현재 작동되고 있는 Container를 확인 가능
![캡처](https://user-images.githubusercontent.com/48710703/199906569-ff047cd3-61af-49cf-8d66-f69add64935c.PNG)

#### 3. 가상 환경 접속

* 웹 브라우저 (크롬) 을 열고, 다음의 주소로 접속
```
http://127.0.0.1:6080/
```

개발 환경이 설치된 Ubuntu OS를 웹 브라우저에서 이용 가능
![캡처2](https://user-images.githubusercontent.com/48710703/199906904-f54b5a5a-8a8c-4a25-b977-b8a2e6381994.PNG)

## 시뮬레이터 사용 방법

### 1. 시뮬레이터 실행

* 실행해야 할 맵에 따라 `Map1.sh`, `Map2.sh`, `Map3.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭
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
**Execute in terminal** 클릭
아래의 문구와 함께 **Brain.py**가 실행됨
![5](https://user-images.githubusercontent.com/48710703/199909682-9c98e999-167f-4233-93a8-761018de8c94.PNG)
                       
### 3. 차량 수동 조작
* `Joystick.sh`을 더블 클릭하고,<br>
**Execute in terminal** 클릭
아래의 문구와 함께 조이스틱이 실행됨<br>
&nbsp;&nbsp;&nbsp;&nbsp;좌/우 화살표 - 차량 조향각 왼쪽/오른쪽 증가<br>
&nbsp;&nbsp;&nbsp;&nbsp;상/하 화살표 - 차량 속도 높임/낮춤<br>
&nbsp;&nbsp;&nbsp;&nbsp;스페이스 바 - 차량 정지<br>
&nbsp;&nbsp;&nbsp;&nbsp;탭 키 - 조향각 초기화<br>
**주의** : Brain.sh 와 따로 사용할 것
![캡처](https://user-images.githubusercontent.com/48710703/200274414-608ace90-05d1-4a65-8747-ead89e63efd6.PNG)
