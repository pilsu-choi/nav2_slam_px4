# D* Lite 커스텀 플래너 개발 문서

## 1. 개요 (Overview)

### 1.1 프로젝트 소개

`nav2_dstar_planner`는 ROS2 Nav2 프레임워크를 위한 D* Lite 경로 계획 플러그인입니다. 이 플러그인은 동적 환경에서 효율적인 경로 재계산을 제공하며, 특히 로봇이 이동하면서 환경 정보가 업데이트되는 상황에 적합합니다.

### 1.2 개발 목적 및 배경

- **동적 환경 대응**: 기존 정적 플래너(A*, Dijkstra)와 달리, 환경 변화에 대해 효율적으로 경로를 재계산
- **Nav2 통합**: Nav2의 표준 플러그인 인터페이스를 구현하여 기존 Nav2 스택과 완전 호환
- **실시간 성능**: 부분 업데이트 메커니즘을 통해 불필요한 재계산을 최소화

### 1.3 D* Lite 알고리즘 개요

D* Lite는 LPA* (Lifelong Planning A*) 알고리즘을 기반으로 한 역방향 경로 계획 알고리즘입니다. Goal에서 Start로 역방향으로 탐색하며, 환경 변화 시 영향을 받은 부분만 효율적으로 재계산합니다.

**주요 특징:**
- 역방향 탐색 (Goal → Start)
- Lazy evaluation을 통한 효율적인 휴리스틱 계산
- 일관성(Consistency) 기반 업데이트 메커니즘

---

## 2. Task 설명 (Task Description)

### 2.1 요구사항 정의

#### 기능 요구사항
1. **Nav2 플러그인 인터페이스 구현**
   - `nav2_core::GlobalPlanner` 인터페이스 상속
   - Lifecycle 노드 관리 (configure, activate, deactivate, cleanup)
   - `createPlan()` 메서드를 통한 경로 생성

2. **Costmap 2D 통합**
   - World 좌표 ↔ Map 좌표 변환
   - Costmap의 장애물 정보 추출 및 반영
   - Costmap 크기 변경 시 동적 대응

3. **동적 경로 재계산**
   - Start 위치 변경 시 효율적 업데이트
   - Goal 변경 시 재초기화
   - 장애물 변화 감지 및 반영

#### 비기능 요구사항
- 실시간 성능: 경로 계산 시간 < 100ms (일반적인 costmap 크기 기준)
- 메모리 효율성: 대규모 costmap에서도 안정적 동작
- 확장성: 다양한 휴리스틱 함수 지원 가능한 구조

### 2.2 개발 범위

#### 포함 사항
- ✅ D* Lite 핵심 알고리즘 구현 (`DStarLite` 클래스)
- ✅ Nav2 플러그인 래퍼 구현 (`DStarPlanner` 클래스)
- ✅ Costmap 2D 통합
- ✅ 기본 빌드 시스템 구성
- ✅ 플러그인 등록 및 설정

#### 제외 사항
- ❌ 3D 경로 계획 (현재 2D만 지원)
- ❌ 경로 스무딩 (별도 smoother 사용)
- ❌ 다중 Goal 지원

### 2.3 제약사항 및 가정

- **2D 경로 계획**: 수평면에서의 경로 계획만 지원
- **Costmap 기반**: Nav2 Costmap 2D를 장애물 정보 소스로 사용
- **8방향 이동**: 대각선 이동 포함 (Manhattan + Diagonal)
- **정적 장애물**: 초기화 시점의 장애물 정보 사용 (동적 장애물은 향후 확장)

---

## 3. D* Lite 알고리즘 이론 (Algorithm Theory)

### 3.1 D* Lite 개요

D* Lite는 Koenig & Likhachev (2002)가 제안한 알고리즘으로, 다음과 같은 특징을 가집니다:

- **D*와의 차이점**: D*는 forward search, D* Lite는 backward search
- **LPA* 기반**: LPA*의 일관성 개념을 활용하여 효율적인 재계산
- **역방향 탐색**: Goal에서 Start로 탐색하여 Start 위치 변경에 효율적

### 3.2 핵심 개념

#### g-value와 rhs-value
- **g-value**: Start에서 해당 노드까지의 실제 비용 (계산된 값)
- **rhs-value**: Start에서 해당 노드까지의 추정 비용 (one-step lookahead)

```cpp
// 일관성 조건
if (g(s) == rhs(s)) {
    // 노드 s는 일관적(consistent)
} else {
    // 노드 s는 불일치(inconsistent) - 재계산 필요
}
```

#### Key 계산
Key는 우선순위 큐에서 노드의 우선순위를 결정합니다:

```cpp
key(s) = min(g(s), rhs(s)) + h(s)
```

여기서 `h(s)`는 휴리스틱 함수 (일반적으로 Euclidean distance)

### 3.3 알고리즘 동작 원리

#### 초기화 단계
1. Goal 노드의 `rhs = 0`, `g = ∞`로 설정
2. Goal을 Open List에 추가
3. 모든 노드의 휴리스틱은 lazy evaluation으로 계산

#### 경로 계산 단계 (`computeShortestPath`)
1. Open List에서 Key가 가장 작은 노드 선택
2. Start 노드가 일관적이고, Open List의 top 노드가 Start보다 우선순위가 낮으면 종료
3. 선택된 노드의 일관성에 따라 업데이트:
   - `g > rhs`: `g = rhs`로 업데이트하고 이웃 노드들 재계산
   - `g < rhs`: `g = ∞`로 설정하고 자신과 이웃 노드들 재계산

#### 경로 추출 단계 (`computePath`)
1. Start 노드에서 시작
2. 각 노드에서 g-value가 가장 작은 이웃 노드로 이동
3. Goal에 도달할 때까지 반복

---

## 4. 시스템 아키텍처 (System Architecture)

### 4.1 전체 구조

```
┌─────────────────────────────────────────┐
│         Nav2 Planner Server            │
│  (nav2_planner::PlannerServer)          │
└──────────────┬──────────────────────────┘
               │
               │ plugin interface
               ▼
┌─────────────────────────────────────────┐
│      DStarPlanner (Plugin)              │
│  - Nav2 인터페이스 구현                  │
│  - Costmap 통합                         │
│  - 좌표 변환                            │
└──────────────┬──────────────────────────┘
               │
               │ uses
               ▼
┌─────────────────────────────────────────┐
│         DStarLite (Core Algorithm)      │
│  - 경로 계산 알고리즘                    │
│  - 데이터 구조 관리                      │
│  - 최적화 로직                           │
└─────────────────────────────────────────┘
```

### 4.2 모듈 구성

#### 4.2.1 DStarLite 클래스 (핵심 알고리즘)

**주요 멤버 변수:**
```cpp
std::unordered_map<Node, double, NodeHash> g_values_;      // g-value 저장
std::unordered_map<Node, double, NodeHash> rhs_values_;    // rhs-value 저장
std::unordered_map<Node, double, NodeHash> h_values_;      // 휴리스틱 캐시
std::unordered_set<Node, NodeHash> obstacles_;             // 장애물 집합
std::priority_queue<Node, ...> open_list_;                 // 우선순위 큐
```

**주요 메서드:**
- `initialize()`: 초기화
- `computePath()`: 경로 계산 및 반환
- `computeShortestPath()`: 최단 경로 탐색
- `updateVertex()`: 정점 업데이트
- `setStart()`: Start 위치 변경 (효율적 업데이트)

#### 4.2.2 DStarPlanner 클래스 (Nav2 인터페이스)

**주요 멤버 변수:**
```cpp
std::unique_ptr<DStarLite> dstar_lite_;                    // 알고리즘 인스턴스
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;  // Costmap
int last_goal_mx_, last_goal_my_;                          // Goal 캐싱
bool is_initialized_;                                      // 초기화 상태
```

**주요 메서드:**
- `configure()`: Lifecycle 설정
- `createPlan()`: Nav2 인터페이스 구현
- `worldToMap()` / `mapToWorld()`: 좌표 변환

### 4.3 의존성 관계

```
nav2_dstar_planner
├── nav2_core (플러그인 인터페이스)
├── nav2_costmap_2d (Costmap 통합)
├── nav2_util (유틸리티)
├── rclcpp / rclcpp_lifecycle (ROS2)
├── geometry_msgs / nav_msgs (메시지 타입)
├── pluginlib (플러그인 시스템)
└── tf2 / tf2_ros (좌표 변환)
```

---

## 5. 개발 내용 (Development Details)

### 5.1 핵심 알고리즘 구현

#### 5.1.1 데이터 구조

**Node 구조체:**
```cpp
struct Node {
    int x, y;
    double g, rhs;
    double h;
    bool is_obstacle;
    
    bool operator==(const Node& other) const {
        return x == other.x && y == other.y;
    }
};
```

**NodeHash (해시 함수):**
```cpp
struct NodeHash {
    std::size_t operator()(const Node& node) const {
        return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
};
```

**NodeComparator (우선순위 큐 비교자):**
```cpp
struct NodeComparator {
    bool operator()(const Node& a, const Node& b) const {
        double key_a = min(g(a), rhs(a)) + h(a);
        double key_b = min(g(b), rhs(b)) + h(b);
        return key_a > key_b;  // 작은 key가 우선
    }
};
```

#### 5.1.2 주요 함수 구현

**초기화 (`initialize`):**
```cpp
void DStarLite::initialize(int start_x, int start_y, int goal_x, int goal_y) {
    start_x_ = start_x;
    start_y_ = start_y;
    goal_x_ = goal_x;
    goal_y_ = goal_y;
    
    reset();  // 이전 데이터 초기화
    
    // Goal 노드 초기화
    Node goal_node(goal_x, goal_y);
    rhs_values_[goal_node] = 0.0;
    g_values_[goal_node] = INFINITY;
    
    // Goal을 Open List에 추가
    open_list_.push(goal_node);
    open_set_.insert(goal_node);
}
```

**최단 경로 계산 (`computeShortestPath`):**
```cpp
void DStarLite::computeShortestPath() {
    Node start_node = getNode(start_x_, start_y_);
    int max_iterations = width_ * height_ * 10;  // 무한 루프 방지
    
    while (!open_list_.empty() && iterations < max_iterations) {
        // Start가 일관적이고 top 노드가 우선순위가 낮으면 종료
        if (start is consistent && top_key >= start_key) {
            break;
        }
        
        Node current = open_list_.top();
        open_list_.pop();
        
        if (g(current) > rhs(current)) {
            g(current) = rhs(current);
            // 이웃 노드들 업데이트
            for (neighbor : getNeighbors(current)) {
                updateVertex(neighbor);
            }
        } else {
            g(current) = INFINITY;
            updateVertex(current);
            // 이웃 노드들 업데이트
            for (neighbor : getNeighbors(current)) {
                updateVertex(neighbor);
            }
        }
    }
}
```

**정점 업데이트 (`updateVertex`):**
```cpp
void DStarLite::updateVertex(const Node& node) {
    if (node is goal) {
        rhs(node) = 0.0;
        return;
    }
    
    // 이웃 노드들 중 최소 비용 계산
    double min_rhs = INFINITY;
    for (neighbor : getNeighbors(node)) {
        if (g(neighbor) < INFINITY) {
            min_rhs = min(min_rhs, g(neighbor) + cost(node, neighbor));
        }
    }
    rhs(node) = min_rhs;
    
    // 불일치 노드는 Open List에 추가
    if (abs(g(node) - rhs(node)) > epsilon) {
        open_list_.push(node);
        open_set_.insert(node);
    }
}
```

#### 5.1.3 최적화 기법

1. **Lazy Heuristic Calculation**:**
   - 휴리스틱은 필요할 때만 계산하고 `h_values_` 맵에 캐싱
   - Goal 변경 시에만 휴리스틱 캐시 초기화

2. **Efficient Neighbor Search:**
   - 8방향 이웃 탐색 (대각선 포함)
   - 장애물 체크를 통한 불필요한 노드 제외

3. **Open Set 관리:**
   - `open_set_`을 별도로 유지하여 중복 체크 효율화
   - 우선순위 큐만으로는 업데이트된 노드를 찾기 어려움

### 5.2 Nav2 플러그인 구현

#### 5.2.1 Nav2 인터페이스 구현

**생성자 및 설정:**
```cpp
DStarPlanner::DStarPlanner()
: costmap_(nullptr), tolerance_(0.5), use_astar_(false), 
  allow_unknown_(true), is_initialized_(false)
{
}

void DStarPlanner::configure(...) {
    node_ = parent;
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    // 파라미터 선언 및 읽기
    declareParameters();
    getParameters();
    
    // DStarLite 초기화 (costmap 크기 기반)
    unsigned int width = costmap_->getSizeInCellsX();
    unsigned int height = costmap_->getSizeInCellsY();
    dstar_lite_ = std::make_unique<DStarLite>(width, height);
}
```

**경로 생성 (`createPlan`):**
```cpp
nav_msgs::msg::Path DStarPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
{
    // 1. World 좌표 → Map 좌표 변환
    int start_mx, start_my, goal_mx, goal_my;
    worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
    worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my);
    
    // 2. Costmap 크기 변경 체크
    if (dstar_lite_->needsResize(current_width, current_height)) {
        dstar_lite_->resize(current_width, current_height);
        is_initialized_ = false;
    }
    
    // 3. Goal 변경 체크
    bool goal_changed = (goal_mx != last_goal_mx_ || goal_my != last_goal_my_);
    bool needs_full_init = !is_initialized_ || goal_changed;
    
    if (needs_full_init) {
        // 전체 초기화 (Goal 변경 또는 첫 실행)
        dstar_lite_->initialize(start_mx, start_my, goal_mx, goal_my);
        
        // Costmap의 모든 장애물 반영
        for (all cells in costmap) {
            if (cost == LETHAL_OBSTACLE) {
                dstar_lite_->updateObstacle(mx, my, true);
            }
        }
    } else {
        // Start 위치만 업데이트 (효율적)
        dstar_lite_->setStart(start_mx, start_my);
    }
    
    // 4. 경로 계산
    std::vector<std::pair<int, int>> path_cells = dstar_lite_->computePath();
    
    // 5. Map 좌표 → World 좌표 변환
    path.poses = createPath(path_cells);
    
    return path;
}
```

#### 5.2.2 Costmap 통합

**좌표 변환:**
```cpp
bool DStarPlanner::worldToMap(double wx, double wy, int & mx, int & my) {
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();
    
    mx = static_cast<int>((wx - origin_x) / resolution);
    my = static_cast<int>((wy - origin_y) / resolution);
    
    return mx >= 0 && mx < costmap_->getSizeInCellsX() &&
           my >= 0 && my < costmap_->getSizeInCellsY();
}

void DStarPlanner::mapToWorld(int mx, int my, double & wx, double & wy) {
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();
    
    wx = origin_x + (mx + 0.5) * resolution;
    wy = origin_y + (my + 0.5) * resolution;
}
```

#### 5.2.3 성능 최적화

1. **Goal 변경 감지:**
   - `last_goal_mx_`, `last_goal_my_`로 Goal 변경 추적
   - Goal이 변경되지 않으면 전체 재초기화 생략

2. **부분 재초기화:**
   - Start 위치만 변경된 경우 `setStart()`만 호출
   - 불필요한 장애물 재스캔 방지

3. **Costmap 크기 변경 대응:**
   - `needsResize()`로 크기 변경 감지
   - `resize()`로 동적 크기 조정

### 5.3 빌드 시스템 구성

#### 5.3.1 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(nav2_dstar_planner)

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
# ... 기타 의존성

# 라이브러리 빌드
add_library(nav2_dstar_planner SHARED
  src/dstar_planner.cpp
  src/dstar_lite.cpp
)

# 의존성 연결
ament_target_dependencies(nav2_dstar_planner
  rclcpp
  nav2_core
  nav2_costmap_2d
  # ... 기타
)

# 플러그인 등록
pluginlib_export_plugin_description_file(nav2_core dstar_planner_plugin.xml)

# 설치
install(TARGETS nav2_dstar_planner ...)
install(DIRECTORY include/ ...)
```

#### 5.3.2 package.xml

```xml
<package format="3">
  <name>nav2_dstar_planner</name>
  <version>1.0.0</version>
  <description>D* Lite path planning plugin for Nav2</description>
  
  <depend>nav2_core</depend>
  <depend>nav2_costmap_2d</depend>
  <!-- ... 기타 의존성 -->
  
  <export>
    <build_type>ament_cmake</build_type>
    <nav2_core plugin="${prefix}/dstar_planner_plugin.xml"/>
  </export>
</package>
```

#### 5.3.3 플러그인 XML

```xml
<library path="libnav2_dstar_planner">
  <class name="nav2_dstar_planner/DStarPlanner" 
         type="nav2_dstar_planner::DStarPlanner" 
         base_class_type="nav2_core::GlobalPlanner">
    <description>D* Lite path planning plugin for Nav2</description>
  </class>
</library>
```

---

## 6. 주요 기능 (Key Features)

### 6.1 동적 경로 재계산

**Start 위치 변경 시:**
- 전체 재초기화 없이 `setStart()`만 호출
- 이전 계산 결과를 재활용하여 효율적 업데이트
- Goal이 변경되지 않으면 휴리스틱 재계산 불필요

**Goal 변경 시:**
- 전체 재초기화 수행
- 휴리스틱 캐시 초기화
- Costmap의 장애물 정보 재스캔

### 6.2 Costmap 통합

- **실시간 장애물 반영**: Costmap의 `LETHAL_OBSTACLE`을 장애물로 인식
- **동적 크기 변경**: Costmap 크기가 변경되면 자동으로 리사이즈
- **좌표 변환**: World 좌표와 Map 좌표 간 자동 변환

### 6.3 파라미터 설정

**사용 가능한 파라미터:**

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["DStarPlanner"]
    DStarPlanner:
      plugin: "nav2_dstar_planner/DStarPlanner"
      tolerance: 0.5          # 목표점 허용 오차 (미터)
      use_astar: false        # A* 모드 (향후 확장)
      allow_unknown: true     # 미탐색 영역 허용 여부
```

---

## 7. 사용 방법 (Usage)

### 7.1 빌드 방법

```bash
# 워크스페이스에서 빌드
cd ~/nav2_slam_px4
colcon build --packages-select nav2_dstar_planner

# 또는 전체 빌드
colcon build

# 환경 설정
source install/setup.bash
```

### 7.2 설정 방법

**navigation.yaml 설정:**

```yaml
planner_server:
  ros__parameters:
    use_sim_time: True
    expected_planner_frequency: 20.0
    planner_plugins: ["DStarPlanner"]
    DStarPlanner:
      plugin: "nav2_dstar_planner/DStarPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

**Launch 파일에서 활성화:**

```python
# rtabmap_nav2_px4/launch/navigation.launch.py
Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    parameters=[nav2_params],
    # ...
)
```

### 7.3 실행 방법

```bash
# 전체 시스템 실행
ros2 launch rtabmap_nav2_px4 bringup.launch.py

# 또는 네비게이션만 실행
ros2 launch rtabmap_nav2_px4 navigation.launch.py
```

### 7.4 파라미터 튜닝

- **tolerance**: 목표점 도달 허용 오차를 조정 (기본값: 0.5m)
- **allow_unknown**: 미탐색 영역을 경로로 사용할지 여부
  - `true`: 미탐색 영역도 경로로 사용 (더 유연한 경로)
  - `false`: 탐색된 영역만 사용 (더 안전한 경로)

---

## 8. 테스트 (Testing)

### 8.1 단위 테스트

**테스트 파일:** `test/test_dstar_planner.cpp`

**테스트 항목:**
- 알고리즘 정확성 검증
- 엣지 케이스 처리 (시작점=목표점, 경로 없음 등)
- 데이터 구조 정확성

**실행 방법:**
```bash
colcon test --packages-select nav2_dstar_planner
colcon test-result --verbose
```

### 8.2 통합 테스트

**Nav2와의 통합:**
- Planner Server와의 연동 테스트
- Costmap과의 통합 테스트
- 실제 시뮬레이션 환경에서 테스트

### 8.3 성능 테스트

**측정 항목:**
- 경로 계산 시간
- 메모리 사용량
- 다양한 costmap 크기에서의 성능

**예상 성능:**
- 작은 costmap (100x100): < 10ms
- 중간 costmap (500x500): < 50ms
- 큰 costmap (1000x1000): < 200ms

---

## 9. 문제 해결 및 최적화 (Troubleshooting & Optimization)

### 9.1 발생한 문제들

#### 문제 1: 초기화 타이밍 이슈
**증상:** Costmap이 아직 준비되지 않았을 때 플래너가 호출됨

**해결:**
```cpp
void DStarPlanner::configure(...) {
    costmap_ = costmap_ros_->getCostmap();
    // Costmap 크기 체크 추가
    if (costmap_->getSizeInCellsX() == 0) {
        RCLCPP_WARN(..., "Costmap not ready");
        return;
    }
}
```

#### 문제 2: 무한 루프 방지
**증상:** 복잡한 환경에서 알고리즘이 무한 루프에 빠짐

**해결:**
```cpp
void DStarLite::computeShortestPath() {
    int max_iterations = width_ * height_ * 10;
    int iterations = 0;
    
    while (!open_list_.empty() && iterations < max_iterations) {
        iterations++;
        // ... 알고리즘 로직
    }
}
```

#### 문제 3: Costmap 크기 변경 처리
**증상:** Costmap 크기가 변경되면 기존 데이터와 불일치

**해결:**
```cpp
bool needs_full_init = !is_initialized_ || goal_changed;
if (dstar_lite_->needsResize(current_width, current_height)) {
    dstar_lite_->resize(current_width, current_height);
    is_initialized_ = false;
    needs_full_init = true;
}
```

### 9.2 최적화 전략

1. **Goal 변경 감지**
   - Goal이 변경되지 않으면 전체 재초기화 생략
   - 휴리스틱 캐시 재활용

2. **부분 업데이트**
   - Start 위치만 변경된 경우 `setStart()`만 호출
   - 불필요한 장애물 재스캔 방지

3. **Lazy Evaluation**
   - 휴리스틱은 필요할 때만 계산
   - `h_values_` 맵에 캐싱하여 재계산 방지

4. **메모리 효율성**
   - `unordered_map` 사용으로 O(1) 접근
   - 불필요한 노드 데이터 자동 정리

---

## 10. 향후 개선 사항 (Future Improvements)

### 10.1 기능 확장

- **3D 경로 계획**: 현재 2D만 지원, 향후 3D 확장
- **다양한 휴리스틱**: Manhattan, Diagonal 등 추가 옵션
- **경로 스무딩 통합**: 별도 smoother 없이 내장 스무딩
- **동적 장애물 지원**: 실시간으로 변화하는 장애물 반영

### 10.2 성능 개선

- **병렬 처리**: 대규모 costmap에서 병렬 계산
- **더 효율적인 데이터 구조**: 메모리 사용량 최적화
- **캐싱 전략 개선**: 더 많은 중간 결과 캐싱

### 10.3 사용성 개선

- **더 많은 파라미터 옵션**: 세밀한 튜닝 가능
- **디버깅 도구**: 경로 계산 과정 시각화
- **로깅 개선**: 상세한 디버그 정보 제공

---

## 11. 참고 자료 (References)

### 11.1 논문 및 자료

1. **D* Lite 원본 논문:**
   - Koenig, S., & Likhachev, M. (2002). "D* Lite." AAAI.

2. **LPA* 알고리즘:**
   - Koenig, S., Likhachev, M., & Furcy, D. (2004). "Lifelong Planning A*." Artificial Intelligence.

3. **Nav2 문서:**
   - [Nav2 Documentation](https://navigation.ros.org/)
   - [Nav2 Plugin Development Guide](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2_planner_plugin.html)

### 11.2 관련 코드

- **Nav2 Core 인터페이스:**
  - `nav2_core::GlobalPlanner` 인터페이스 정의
  - [GitHub: nav2_core](https://github.com/ros-planning/navigation2/tree/main/nav2_core)

- **다른 플래너 구현 예시:**
  - `nav2_navfn_planner`: NavFn 플래너
  - `nav2_theta_star_planner`: Theta* 플래너

---

## 12. 부록 (Appendix)

### 12.1 코드 구조도

```
nav2_dstar_planner/
├── CMakeLists.txt
├── package.xml
├── dstar_planner_plugin.xml
├── README.md
├── include/
│   └── nav2_dstar_planner/
│       ├── dstar_lite.hpp      # 핵심 알고리즘 헤더
│       └── dstar_planner.hpp   # Nav2 플러그인 헤더
├── src/
│   ├── dstar_lite.cpp          # 핵심 알고리즘 구현
│   └── dstar_planner.cpp       # Nav2 플러그인 구현
└── test/
    └── test_dstar_planner.cpp  # 단위 테스트
```

### 12.2 클래스 다이어그램

```
┌─────────────────────┐
│   DStarPlanner      │
│  (Nav2 Plugin)      │
├─────────────────────┤
│ - costmap_ros_      │
│ - dstar_lite_       │
│ - tolerance_        │
│ + configure()       │
│ + createPlan()      │
│ + worldToMap()      │
└──────────┬──────────┘
           │ uses
           ▼
┌─────────────────────┐
│    DStarLite        │
│  (Core Algorithm)   │
├─────────────────────┤
│ - g_values_         │
│ - rhs_values_       │
│ - open_list_        │
│ + initialize()      │
│ + computePath()     │
│ + updateVertex()    │
└─────────────────────┘
```

### 12.3 주요 함수 플로우차트

```
createPlan()
    │
    ├─> worldToMap() [좌표 변환]
    │
    ├─> needsResize() [크기 체크]
    │   └─> resize() [필요시]
    │
    ├─> Goal 변경 체크
    │   ├─> 변경됨 → initialize() [전체 초기화]
    │   │           └─> updateObstacle() [장애물 반영]
    │   └─> 변경 안됨 → setStart() [부분 업데이트]
    │
    ├─> computePath() [경로 계산]
    │   └─> computeShortestPath() [최단 경로 탐색]
    │       └─> updateVertex() [정점 업데이트]
    │
    └─> createPath() [좌표 변환 및 반환]
```

### 12.4 설정 파일 예시

**navigation.yaml (전체 예시):**
```yaml
planner_server:
  ros__parameters:
    use_sim_time: True
    expected_planner_frequency: 20.0
    planner_plugins: ["DStarPlanner"]
    DStarPlanner:
      plugin: "nav2_dstar_planner/DStarPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

### 12.5 로그 분석 가이드

**일반적인 로그 메시지:**

```
[INFO] Configuring DStarPlanner with costmap size 500x500
[INFO] Found path with 150 points
[WARN] Start point is outside the costmap
[WARN] Goal point is occupied
[WARN] No path found
```

**디버깅 팁:**
- Costmap 크기가 0이면 Costmap이 아직 준비되지 않음
- "No path found"는 장애물로 막혀있거나 Goal이 유효하지 않음
- 경로 포인트 수가 비정상적으로 많으면 경로가 비효율적일 수 있음

---

## 라이선스

Apache-2.0 License

## 작성자

kimhoyun (suberkut76@gmail.com)

## 버전

1.0.0

