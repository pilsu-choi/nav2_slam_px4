#!/bin/bash

set -e  # 에러 발생 시 스크립트 중단

# 필요한 패키지 설치
if ! python3 -c "import gdown" 2>/dev/null; then
    echo "gdown을 설치합니다..."
    pip install --user gdown
    # Python 경로에 user site-packages 추가
    export PYTHONPATH="$HOME/.local/lib/python3.$(python3 -c 'import sys; print(sys.version_info.minor)')/site-packages:$PYTHONPATH"
fi

# PATH에 local bin 디렉토리 추가
export PATH="$HOME/.local/bin:$PATH"

# 다운로드할 폴더 URL
FOLDER_URL="https://drive.google.com/drive/folders/10nP2gtEj7PCql3SgiOLGmmP9n3ZzYQve"

# 기존 downloaded_data 디렉토리 제거 (있다면)
if [ -d ./downloaded_data ]; then
    echo "기존 downloaded_data 디렉토리를 제거합니다..."
    rm -rf ./downloaded_data
fi

# 폴더 다운로드
echo "Google Drive에서 데이터를 다운로드합니다..."
python3 -m gdown --folder "$FOLDER_URL" -O ./downloaded_data

# 다운로드 성공 여부 확인
if [ ! -d ./downloaded_data ]; then
    echo "오류: downloaded_data 디렉토리가 생성되지 않았습니다."
    exit 1
fi

# 다운로드한 파일을 .ros/rtabmap.db 에 덮어쓰기
# 기존 파일 백업
if [ -f ~/.ros/rtabmap.db ]; then
    echo "기존 rtabmap.db 파일을 백업합니다..."
    cp ~/.ros/rtabmap.db ~/.ros/rtabmap.db.backup
fi

# .ros 디렉토리 생성 (없다면)
mkdir -p ~/.ros

# 새 파일로 교체
echo "새로운 rtabmap.db 파일로 교체합니다..."
if [ -f ./downloaded_data/rtabmap.db ]; then
    cp ./downloaded_data/rtabmap.db ~/.ros/rtabmap.db
    echo "rtabmap.db 파일이 성공적으로 업데이트되었습니다."
else
    echo "오류: downloaded_data/rtabmap.db 파일을 찾을 수 없습니다."
    exit 1
fi

# 임시 디렉토리 정리
echo "임시 파일을 정리합니다..."
rm -rf ./downloaded_data

