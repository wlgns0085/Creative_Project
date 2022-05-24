# Bebop & HoloLens Project
HoloLens의 음성인식 및 제스처를 통한 드론 제어

## 원하는 좌표로 이동
Bebop은 속도기반 제어  
### 방법A)  
일정 시간동안 publish
time = velocity / distance
### 방법B)
원하는 위치 및 방향에 일치할 때까지 publish  
비행 중 현재 Yaw 확인

## 북쪽 기준 방위각
include <sensor_msgs/MagneticField.h> 필요
driver 실행시 GPS 연결 가능해야 함

## 경로 메시지  
경로를 한 번에 전송하지 않고  
경로 전송 시작, 종료, 전송중을 알리는 데이터와 longitude와 latitude값을 전송  

ex)  
header header  
int mode  
double longitude
double latitude
