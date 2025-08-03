#!/bin/bash

echo "=== 최종 xclock 테스트 ==="

# X11 권한 설정
echo "1. X11 소켓 권한 설정 중..."
xhost +local:docker

echo "2. xclock 실행 중..."

# 더 강력한 X11 권한 설정으로 테스트
docker run --rm -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority \
    ubuntu:22.04 bash -c "
        apt update -qq && apt install -y x11-apps -qq
        echo 'DISPLAY: '\$DISPLAY
        echo 'X11 소켓 확인:'
        ls -la /tmp/.X11-unix/
        echo 'xclock 실행:'
        xclock
    "

# X11 권한 복원
echo "3. X11 소켓 권한 복원 중..."
xhost -local:docker

echo "테스트 완료!" 