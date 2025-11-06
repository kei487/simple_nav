# Simple Nav - ROS 2 A\* Planner & Pure Pursuit Controller

ROS 2 Humbleで動作する差動2輪ロボット用の経路計画/経路追従パッケージ

## 概要

このパッケージには以下のコンポーネントが含まれています：

1. **Pure Pursuit Controller** (`pure_pursuit_controller`) - 経路追従コントローラー
2. **A\* Path Planner** (`a_star_planner`) - A\*ベースの経路計画ノード

## Pure Pursuit Controller

### 機能

- Pure Pursuitアルゴリズムで経路を追従
- ゴール地点を受信すると、経路計画サービスを呼び出して経路を取得
- 経路と現在地点から速度指令値を算出

### インターフェース

#### 購読トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | 目標地点 |
| `/tf` | `tf2_msgs/TFMessage` | グローバル座標系からロボット座標系までtreeが繋がっている必要有り |

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

## A\* Path Planner

### 機能

- A\*アルゴリズムで経路計画を行う

### インターフェース
#### 購読トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/costmap_2d` | `nav_msgs/OccupancyGrid` | 探索マップ |

#### 配信トピック

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/plan_path` | `nav_msgs/Path` | 探索経路（可視化用） |
| `/Planner_searched_map` | `nav_msgs/OccupancyGrid` | 探索済みマップ（可視化用） |

#### サービスサーバー

| サービス名 | サービス型 | 説明 |
|-----------|-----------|------|
| `/get_path` | `value_iteration2_astar_msgs/srv/GetPath` | 経路計画サービス |

### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|------------|------|-----------|------|
| `use_dijkstra` | bool | false | ダイクストラ法で経路計画を行うか |
| `publish_searched_map` | bool | false | デバック用 |
| `update_path_weight` | double | 0.05 |  |
| `smooth_path_weight` | double | 0.8 |  |
| `iteration_delta_threshold` | double | 1.0e-6 |  |

## インストール&ビルド方法

```bash
cd /path/to/your/workspace
cd src
git clone https://github.com/kei487/simple_nav.git
git clone https://github.com/kei487/value_iteration2_astar_msgs.git
cd /path/to/your/workspace
colcon build --packages-select simple_nav
source install/setup.bash
```

## 実行方法

```bash
ros2 launch simple_nav simple_nav.launch.py
```

## 前提条件

### 必要なTF

コントローラーが正常に動作するには、以下のTF変換が必要です：

```
map -> odom -> base_link
```

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

- TFが正しく配信されているか確認: `ros2 run rqt_tf_tree rqt_tf_tree`
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

MIT License

©Keitaro Nakamura (numerugon487@gmail.com)
