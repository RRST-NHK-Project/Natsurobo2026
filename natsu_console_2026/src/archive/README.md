# アーカイブ (LiDAR / IMU 依存の画面)

2026-08-21、機体から LiDAR と IMU を使わなくなったため、それらに依存する
コンソールのページを本体から取り除いてここへ退避した。**このディレクトリの中身は
ビルドに含まれない**（どこからも import しておらず、`.txt` のものは JS として
解釈もされない）。消さずに残してあるのは、センサを戻す判断になった時に
そのまま貼り直せるようにするため。

## 取り除いたページ

| ページ | 依存 | 状態 |
| --- | --- | --- |
| 自己位置マップ (`localization`) | `/odom`(nav_msgs) + `/localization/pose`(natsu_localization の LiDAR ICP) + `/imu` + LiDAR点群の累積 | 全部削除 |
| 壁面推定 (`wall-angle`) | `/wall_detection/angle`, `/wall_detection/filtered_points`, `/wall_detection/ransac_params` | 全部削除 |
| カゴ位置・姿勢推定 (`cage-detection`) | `/cage_detection/*` | 全部削除 |

`cage_detection` はカメラではなく **wall_detection(LiDAR) の角度・距離から自己位置を
推定する**ノード（`cage_detection/src/cage_detection_node.cpp` の冒頭コメント参照）。
LiDAR が無いと publisher 側が動かないので、このページも一緒に落とした。

## 残したもの

- **SHOOTモードのカゴ選択** (`components/BasketSelectPanel.js`) は残っている。
  7個のカゴボタン → `/manual/basket` は cage_detection と無関係に動くため。
  俯瞰図は `CageViz`（`/cage_detection/cages` を重畳表示）から、固定座標だけを描く
  `components/FieldMap.js` に置き換えた。
- **座標・姿勢管理** (`pose`) は残している。`odom_xy_yaw` はエンコーダ由来で
  LiDAR/IMU に依存しないため。

## 中身

| ファイル | 何 |
| --- | --- |
| `pages/localization-page.jsx.txt` | 自己位置マップページの JSX |
| `pages/wall-angle-page.jsx.txt` | 壁面推定ページの JSX |
| `pages/cage-detection-page.jsx.txt` | カゴ位置・姿勢推定ページの JSX |
| `app-support-code.js.txt` | 上記3ページ用に App.js から消した state / ref / 購読 / クリーンアップ / 派生値 |
| `OdomMap.js` | 自己位置マップの Canvas 描画コンポーネント（そのまま動く） |
| `CageViz.js` | カゴ俯瞰図＋受信カゴ一覧（`/cage_detection/cages` を購読する版） |

## 戻し方

1. `App.js` の `pageOrder` と `getPageLabel` にページ名を戻す
2. `app-support-code.js.txt` の各塊を、App.js の同種のコードが並ぶ場所へ戻す
3. `pages/*.jsx.txt` を他のページブロックと同じ位置に貼る
4. `OdomMap.js` / `CageViz.js` を `src/components/` へ戻して import を足す
5. 機体側で LiDAR / IMU を起動する（`natsu_console_2026/start.sh` は既定でセンサを
   起動しなくなっているので `START_SENSORS=1 ./start.sh`)
