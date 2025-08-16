#!/bin/bash

# GPU 연결 테스트 스크립트

# 이미지 이름과 태그 설정
IMAGE_NAME="gpu_test"
TAG="latest"
CONTAINER_NAME="gpu_test_container"

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== GPU Connection Test Script ===${NC}"

# NVIDIA Container Toolkit 설치 확인
echo -e "${YELLOW}1. Checking NVIDIA Container Toolkit installation...${NC}"
if ! command -v nvidia-container-runtime &> /dev/null; then
    echo -e "${RED}NVIDIA Container Toolkit is not installed. Please install it first:${NC}"
    echo "sudo apt-get install nvidia-container-toolkit"
    echo "sudo systemctl restart docker"
    exit 1
else
    echo -e "${GREEN}✓ NVIDIA Container Toolkit is installed${NC}"
fi

# NVIDIA 드라이버 확인
echo -e "${YELLOW}2. Checking NVIDIA drivers...${NC}"
if ! nvidia-smi &> /dev/null; then
    echo -e "${RED}NVIDIA drivers are not properly installed or GPU is not detected.${NC}"
    exit 1
else
    echo -e "${GREEN}✓ NVIDIA drivers are working${NC}"
fi

# 호스트 GPU 정보 출력
echo -e "${YELLOW}3. Host GPU Information:${NC}"
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader,nounits

# 기존 컨테이너 정리
echo -e "${YELLOW}4. Cleaning up existing containers...${NC}"
sudo docker stop $CONTAINER_NAME 2>/dev/null || true
sudo docker rm $CONTAINER_NAME 2>/dev/null || true

# Docker 이미지 빌드
echo -e "${YELLOW}5. Building GPU test Docker image...${NC}"
sudo docker build -f Dockerfile.gpu_test -t $IMAGE_NAME:$TAG .

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Docker image built successfully!${NC}"
else
    echo -e "${RED}✗ Failed to build Docker image.${NC}"
    exit 1
fi

# Docker 컨테이너 실행 및 GPU 테스트
echo -e "${YELLOW}6. Running GPU test container...${NC}"
echo -e "${BLUE}Running comprehensive GPU tests inside container...${NC}"

sudo docker run -it --rm \
    --name $CONTAINER_NAME \
    --gpus all \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    $IMAGE_NAME:$TAG \
    /usr/local/bin/gpu_test.sh

echo -e "${GREEN}=== GPU Test Completed ===${NC}" 