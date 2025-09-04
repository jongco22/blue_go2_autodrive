# unitree_ros2 (forked by jongco22)

이 저장소는 [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)를 fork하여,
**blue_go2_autodrive 프로젝트**에 맞게 일부 수정한 버전입니다.

---

## 📌 변경 사항
- `/unitree_ros2/example/src/src/go2/go2_sport_client.cpp` 파일 수정

---

## 🚀 실행 방법

### 1. 리포지토리 클론
```bash
git clone --recursive https://github.com/jongco22/blue_go2_autodrive.git
```
### 2. launch 실행
```bash
ros2 launch blue_segmentation follow_blue_go2.launch.py \
  image_topic:=/camera/camera/color/image_raw/compressed \
  go2_mode:=0
```

필요시 docker container 생성
