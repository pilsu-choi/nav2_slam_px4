#!/bin/bash

echo "=== Docker 이미지 빌드 및 실행 ==="

# 이미지 이름과 태그 설정
IMAGE_NAME="nav2_slam_px4"
IMAGE_TAG="dev"

# Docker 데몬 설정 확인 및 설정
setup_docker_runtime() {
    echo "🔧 Docker NVIDIA runtime 설정 확인 중..."
    
    # nvidia-container-runtime 설치 확인
    if ! command -v nvidia-container-runtime &> /dev/null; then
        echo "❌ nvidia-container-runtime이 설치되지 않았습니다."
        echo "다음 명령어로 설치하세요:"
        echo "sudo apt-get install nvidia-container-runtime"
        return 1
    fi
    
    # Docker 데몬 설정 파일 확인
    if [ ! -f "/etc/docker/daemon.json" ]; then
        echo "📝 Docker 데몬 설정 파일 생성 중..."
        sudo mkdir -p /etc/docker
        echo '{"default-runtime": "nvidia", "runtimes": {"nvidia": {"path": "nvidia-container-runtime", "runtimeArgs": []}}}' | sudo tee /etc/docker/daemon.json > /dev/null
        
        echo "🔄 Docker 서비스 재시작 중..."
        sudo systemctl restart docker
        
        # Docker 서비스 재시작 후 잠시 대기
        sleep 3
        
        echo "✅ Docker NVIDIA runtime 설정 완료!"
    else
        echo "✅ Docker 데몬 설정 파일이 이미 존재합니다."
        
        # 설정 내용 확인
        if grep -q "nvidia" /etc/docker/daemon.json; then
            echo "✅ NVIDIA runtime이 이미 설정되어 있습니다."
        else
            echo "⚠️  Docker 데몬 설정에 NVIDIA runtime이 없습니다."
            echo "수동으로 /etc/docker/daemon.json을 확인하고 수정하세요."
        fi
    fi
}

# GPU 사용 가능 여부 확인
check_gpu_availability() {
    echo "🔍 GPU 사용 가능 여부 확인 중..."
    
    if command -v nvidia-smi &> /dev/null; then
        echo "✅ NVIDIA GPU 드라이버가 설치되어 있습니다."
        nvidia-smi --query-gpu=name --format=csv,noheader,nounits | head -1
        return 0
    else
        echo "⚠️  NVIDIA GPU 드라이버가 설치되지 않았습니다."
        echo "GPU 없이 컨테이너를 실행합니다."
        return 1
    fi
}

# Docker 데몬 설정 실행
setup_docker_runtime

# GPU 사용 가능 여부 확인
GPU_AVAILABLE=false
if check_gpu_availability; then
    GPU_AVAILABLE=true
    echo "🚀 GPU 모드로 실행합니다."
else
    echo "💻 CPU 모드로 실행합니다."
fi

gpu 사용 여부에 따라 다른 이미지 사용
if [ "$GPU_AVAILABLE" = true ]; then
    echo "1. with GPU Docker 이미지 빌드 중..."
    sudo docker build -f Dockerfile.dev_gpu -t ${IMAGE_NAME}:${IMAGE_TAG} .
else
    echo "1. without GPU Docker 이미지 빌드 중..."
    sudo docker build -f Dockerfile.dev -t ${IMAGE_NAME}:${IMAGE_TAG} .
fi

if [ $? -ne 0 ]; then
    echo "❌ Docker 이미지 빌드 실패!"
    exit 1
fi

echo "✅ Docker 이미지 빌드 완료: ${IMAGE_NAME}:${IMAGE_TAG}"

# X11 권한 설정
echo "2. X11 소켓 권한 설정 중..."
xhost +local:docker

echo "3. Docker 컨테이너 실행 중..."

# GPU 사용 가능 여부에 따라 다른 옵션으로 실행
if [ "$GPU_AVAILABLE" = true ]; then
    echo "🎮 GPU 가속 모드로 컨테이너 실행 중..."
    sudo docker run -d \
        --name ${IMAGE_NAME}_container \
        --privileged \
        --network host \
        --gpus all \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/home/ubuntu/.Xauthority \
        -v $(pwd):/home/ubuntu/workspace \
        ${IMAGE_NAME}:${IMAGE_TAG}
else
    echo "💻 CPU 모드로 컨테이너 실행 중..."
    sudo docker run -d \
        --name ${IMAGE_NAME}_container \
        --privileged \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $HOME/.Xauthority:/home/ubuntu/.Xauthority \
        -v $(pwd):/home/ubuntu/workspace \
        ${IMAGE_NAME}:${IMAGE_TAG}
fi

# 컨테이너 실행 확인
if [ $? -eq 0 ]; then
    echo "✅ 컨테이너가 백그라운드에서 실행 중입니다."
    echo "📋 컨테이너 정보:"
    sudo docker ps | grep ${IMAGE_NAME}_container
    
    echo ""
    echo "🔧 컨테이너에 접근하는 방법:"
    echo "sudo docker exec -it ${IMAGE_NAME}_container bash"
    echo ""
    echo "📊 컨테이너 로그 확인:"
    echo "sudo docker logs ${IMAGE_NAME}_container"
    echo ""
    echo "🛑 컨테이너 중지:"
    echo "sudo docker stop ${IMAGE_NAME}_container"
    echo ""
    echo "🗑️  컨테이너 삭제:"
    echo "sudo docker rm ${IMAGE_NAME}_container"
else
    echo "❌ 컨테이너 실행 실패!"
    exit 1
fi

#px4_nav2_slam 프로젝트 실행
'''
docker container exec -it nav2_slam_px4 bash 터미널 4개 실행

# Terminal 1 
cd ~/ && ./QGroundControl-x86_64.AppImage -> qgroundcontrol 실행

# Terminal 2
cd ~/Micro-XRCE-DDS-Agent/ && MicroXRCEAgent udp4 -p 8888 -> 마이크로 XRCE 실행

# Terminal 3
cd ~/PX4-Autopilot && PX4_GZ_WORLD=turtlebot3_world make px4_sitl gz_x500_rtab -> 가제보 and px4 실행 param set SYS_HAS_MAG 0

# Terminal 4
cd ~/px4_nav2_slam && colcon build
cd ~/px4_nav2_slam && export GZ_SIM_RESOURCE_PATH=/home/ubuntu/PX4-Autopilot/Tools/simulation/gz/models && source install/setup.bash && ros2 launch rtabmap_nav2_px4 bringup.launch.py use_sim_time:=true
'''

# X11 권한 복원
echo "4. X11 소켓 권한 복원 중..."
xhost -local:docker

echo "✅ 컨테이너 실행 완료!" 

