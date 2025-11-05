# Simple Nav - ROS 2 A\* Planner & Pure Pursuit Controller

ROS 2 Humbleで動作する差動2輪ロボット用の経路計画/経路追従パッケージ

## 概要

このパッケージには以下のコンポーネントが含まれています：

1. **Pure Pursuit Controller** (`pure_pursuit_controller`) - 経路追従コントローラー
2. **A\* Path Planner** (`a_star_planner`) - A\*ベースの経路計画ノード

## Pure Pursuit Controller

### 機能

- ゴール地点を受信すると、経路計画サービスを呼び出して経路を取得
- Pure Pursuitアルゴリズムで経路を追従
- TF2を使用してロボットの現在位置を取得
- RVizでの可視化に対応

### インターフェース

#### 購読トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目標地点 |
| `/tf` | `tf2_msgs/TFMessage` | 座標変換（TF2が内部的に購読） |

#### 配信トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令値 |
| `/current_path` | `nav_msgs/Path` | 現在追従中の経路（可視化用） |
| `/lookahead_point` | `geometry_msgs/PointStamped` | 先行点（可視化用） |

#### サービスクライアント

| サービス名 | サービス型 | 説明 |
|-----------|-----------|------|
| `/get_path` | `value_iteration2_astar_msgs/srv/GetPath` | 経路計画サービス |

### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|------------|------|-----------|------|
| `lookahead_distance` | double | 0.5 [m] | 先行点までの距離 |
| `target_linear_velocity` | double | 0.3 [m/s] | 目標並進速度 |
| `control_frequency` | double | 20.0 [Hz] | 制御ループの周波数 |
| `goal_tolerance_dist` | double | 0.1 [m] | ゴール到達判定の許容誤差 |
| `path_service_name` | string | "/get_path" | 経路計画サービス名 |
| `map_frame` | string | "map" | グローバル座標系 |
| `robot_base_frame` | string | "base_link" | ロボット座標系 |

### 動作状態

コントローラーは以下の3つの状態を持ちます：

- **IDLE（待機）**: ゴールが設定されていない状態。ロボットは停止。
- **PLANNING（計画中）**: 経路計画サービスを呼び出し中。
- **NAVIGATING（追従中）**: 経路に沿って追従制御を実行中。

## ビルド方法

```bash
cd /path/to/your/workspace
colcon build --packages-select simple_nav
source install/setup.bash
```

## 実行方法

### 1. A* Path Plannerの起動

まず、経路計画ノードを起動します：

```bash
ros2 run simple_nav a_star_planner
```

このノードは以下が必要です：
- `/costmap_2d` トピック（OccupancyGrid）から地図情報を受信
- `/get_path` サービスで経路計画を提供

### 2. Pure Pursuit Controllerの起動

次に、経路追従コントローラーを起動します：

```bash
ros2 run simple_nav pure_pursuit_controller
```

### 3. パラメータをカスタマイズして起動

```bash
ros2 run simple_nav pure_pursuit_controller --ros-args \
  -p lookahead_distance:=0.8 \
  -p target_linear_velocity:=0.5 \
  -p control_frequency:=30.0
```

### 4. パラメータファイルを使用して起動

パラメータファイル `config/pure_pursuit_params.yaml` を作成：

```yaml
pure_pursuit_controller:
  ros__parameters:
    lookahead_distance: 0.8
    target_linear_velocity: 0.5
    control_frequency: 30.0
    goal_tolerance_dist: 0.15
    path_service_name: "/get_path"
    map_frame: "map"
    robot_base_frame: "base_link"
```

起動：

```bash
ros2 run simple_nav pure_pursuit_controller --ros-args \
  --params-file config/pure_pursuit_params.yaml
```

## 使用例

### ゴール地点を送信

```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 5.0, y: 3.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 速度指令を確認

```bash
ros2 topic echo /cmd_vel
```

### 可視化（RViz）

RVizで以下のトピックを追加して可視化できます：

- `/current_path` - 追従中の経路（Path）
- `/lookahead_point` - 先行点（PointStamped）
- `/tf` - TF表示

## 前提条件

### 必要なTF

コントローラーが正常に動作するには、以下のTF変換が必要です：

```
map -> odom -> base_link
```

### 必要なサービス

経路計画サービス（`/get_path`）を提供するノードが実行されている必要があります。

## アルゴリズム

### Pure Pursuitアルゴリズム

1. **先行点の探索**: 経路上でロボットから`lookahead_distance`だけ離れた点を探索
2. **座標変換**: 先行点をロボット座標系に変換
3. **曲率の計算**: κ = 2 * y_t / L²（L: ロボットから先行点までの距離、y_t: 先行点のy座標）
4. **速度指令の決定**:
   - 並進速度: v = `target_linear_velocity`
   - 角速度: ω = v * κ

## トラブルシューティング

### ロボットが動かない

- TFが正しく配信されているか確認: `ros2 run tf2_tools view_frames`
- 経路計画サービスが起動しているか確認: `ros2 service list | grep get_path`
- ゴール地点が送信されているか確認: `ros2 topic echo /goal_pose`

### 経路計画に失敗する

- 経路計画サービスが正常に応答しているか確認
- スタート地点とゴール地点が障害物の中にないか確認

### ロボットが経路から外れる

- `lookahead_distance`を調整（大きくすると滑らかだが追従性が低下）
- `target_linear_velocity`を調整（速すぎる場合は減速）
- `control_frequency`を上げる

## ライセンス

Apache-2.0

## 作成者

keitaro (numerugon487@gmail.com)
