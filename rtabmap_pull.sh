#!/bin/bash

# 필요한 패키지 설치
pip show gdown &>/dev/null || pip install --user gdown

# PATH에 local bin 디렉토리 추가
export PATH="$HOME/.local/bin:$PATH"

# 다운로드할 폴더 URL
FOLDER_URL="https://drive.google.com/drive/folders/10nP2gtEj7PCql3SgiOLGmmP9n3ZzYQve"

# 폴더 다운로드
gdown --folder "$FOLDER_URL" -O ./downloaded_data

# 다운로드한 파일을 .ros/rtabmap.db 에 덮어쓰기
# 기존 파일 백업
if [ -f ~/.ros/rtabmap.db ]; then
    echo "기존 rtabmap.db 파일을 백업합니다..."
    cp ~/.ros/rtabmap.db ~/.ros/rtabmap.db.backup
fi

# 새 파일로 교체
echo "새로운 rtabmap.db 파일로 교체합니다..."
cd downloaded_data && cp rtabmap.db ~/.ros/rtabmap.db

echo "rtabmap.db 파일이 성공적으로 업데이트되었습니다."

