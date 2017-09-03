터틀봇3 오토레이스 - 동의대학교 분노의질주팀
=========================================

실행법
-----
$ roslaunch deu_racer turtlebot3_robot.launch
  >-터틀봇, 카메라, 라이다 구동
  
$ roslaunch deu_racer deu_racer.launch
 >터틀봇 자율주행을 위한 launch파일
 >>구성
 >>follower.py : 카메라 영상에서 양쪽의 라인을 찾음
 >>nav_mode.py : 자율/수동 주행모드를 지정
 >>tur3_nav.py : follower에서 찾은 라인을 따라 주행
 >>blocking.py : 주행중 차단바가 나타날경우 로봇을 정지시키는 토픽발생
 >>light.py    : 주행중 신호등이 나타날경우 신호에 맞게 주행하는 토픽발생
 >>stabilizer.py : 주행중 차선을 잃어버리면, 찾을때 까지 제자리 회전을함
 >>park_sign_match.py : 주차 미션을 수행
 >>maze.py     : 터널미션을 수행
    
기타
----
    cam_setting.py : 라인카메라 캘리브레이션
    mask_tool.py   : 영상의 마스크를 찾게 도와줌
