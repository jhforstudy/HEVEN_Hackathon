## Docker 환경 구성

#### 1. docker 설치

* 다음의 링크를 참고하여 Docker 및 WSL 2 [설치](https://myjamong.tistory.com/296)

#### 2. Docker image pull 및 실행

* Windows Powershell을 관리자 권한으로 실행
* 터미널 창에 다음을 입력
```
docker run -p 6080:80 -e RESOLUTION=1920x1080 --shm-size=512m jhforstudy/heven_hack:latest
```
자동으로 Dockerhub에서 image를 받아 와서 실행 함

#### 3. 가상 환경 접속

* 웹 브라우저 (크롬) 을 열고, 다음의 주소로 접속
```
http://127.0.0.1:6080/
```

개발 환경이 설치된 Ubuntu OS를 웹 브라우저에서 이용 가능

## 시뮬레이터 사용 방법
