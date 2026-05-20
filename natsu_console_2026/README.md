# 🚀 NATSUROBO 2026 - 操縦コンソール

関西夏ロボコン2026の操縦者向けコンソール（GUI）。ダミーデータで動作可能な、完全に自立したReactアプリケーションです。

## 概要

このコンソールは、3分（180秒）間のロボット競技をリアルタイムで管理します。以下の4つの主要機能を提供：

1. **タイマー & フェーズ同期インジケータ** - 6つの戦略フェーズを可視化
2. **7つのカゴ占有状況モニター** - リアルタイム占有判定
3. **Vゴール「大漁」条件チェックリスト** - 3条件の同時達成を確認
4. **残弾・得点カウンター** - 投入弾数とスコアを自動計算

## 特徴

- ✅ **ダミーデータ動作** - ROS 2未使用。100%フロントエンドで動作
- ✅ **暗い会場対応** - ダークモード基調 + ネオン色（サイバーパンク風）
- ✅ **ROS 2対応可能** - State Storeが統一されており、トピック連携が容易
- ✅ **Fallback Mode対応** - 戦略分岐を自動判定・手動トリガー可能
- ✅ **タッチ・キーボード対応** - スマートフォン・タブレットでも操作可能

## ディレクトリ構成

```
natsu_console_2026/
├── public/
│   └── index.html              # HTMLテンプレート
├── src/
│   ├── App.js                  # メインアプリケーション
│   ├── App.css                 # グローバルスタイル
│   ├── index.js                # エントリーポイント
│   ├── components/
│   │   ├── TimerPhaseController.js    # タイマー & フェーズ表示
│   │   ├── BasketMonitor.js           # 7カゴ占有モニター
│   │   ├── VGoalChecker.js            # V-Goal条件チェック
│   │   └── AmmoScoreCounter.js        # 残弾・得点カウンター
│   ├── store/
│   │   └── GameState.js        # 全体状態管理クラス
│   └── i18n/
│       └── dictionary.js       # 多言語辞書（日本語・英語）
└── package.json
```

## インストール・実行

### 前提条件

- Node.js 16.x 以上
- npm 8.x 以上

### セットアップ

```bash
cd natsu_console_2026
npm install
npm start
```

ブラウザが自動的に `http://localhost:3000` で起動します。

### ビルド

```bash
npm run build
```

`build/` フォルダに本番用のバンドルが生成されます。

## 使用方法

### 基本操作

1. **タイマーの開始**
   - 「▶ 開始」ボタンをクリック
   - 毎秒自動でカウントダウンが進みます

2. **カゴへの投入**
   - 各カゴの「+」ボタンで自チームの投入数を増加
   - 「-」ボタンで減少
   - 自動的に残弾が減少します

3. **大うなぎ・石倉のON/OFF**
   - V-Goal条件チェックリストのトグルボタンで設定
   - 3つのボタンが同時にONで V-Goal 準備完了

4. **Fallback Mode の設定**
   - Phase 3（0:35-1:00）で石倉登坂が失敗した場合、
     「⚠ 石倉未到達 (Fallback)」ボタンをクリック
   - 投入目標が 10→5本に、V-Goal時刻が 2:00→2:20に変更

### 仕様詳細

#### タイマー & フェーズ（TimerPhaseController）

6つのフェーズが自動的に時間経過に応じてハイライト：

| フェーズ | 時間 | 説明 |
|---------|------|------|
| Phase 1 | 0:00-0:10 | スタート・移動 |
| Phase 2 | 0:10-0:35 | 大うなぎ捕獲フェーズ |
| Phase 3 | 0:35-1:00 | 対角移動・石倉登坂（危険エリア） |
| Phase 4 | 1:00-1:55 | 小うなぎ連続投入（目標: 10-12本） |
| Phase 5 | 1:55-2:00 | Vゴール合図フェーズ |
| Phase 6 | 2:00-3:00 | バッファ（リトライ・逆転対応） |

#### 7つのカゴ占有状況（BasketMonitor）

各カゴは以下を管理：

- **our_count** - 自チームの投入数（+/- ボタンで調整）
- **their_count** - 相手の投入数（+/- ボタンで調整）
- **is_occupied** - 占有フラグ（our_count > their_count で true）

**カゴ一覧:**
- 緑 A ★（基準ターゲット）
- 緑 B ★（基準ターゲット）
- 最遠青（対角）★（基準ターゲット）
- 近接青 A ★（基準ターゲット）
- 近接青 B ★（基準ターゲット）
- その他青 C
- その他青 D

#### V-Goal 条件チェックリスト（VGoalChecker）

3つの条件がすべて満たされると、「🚀 大漁合図 準備完了」アラートが表示：

1. ✓ **大うなぎ捕獲** - 手動トグルで ON
2. ✓ **カゴ占有数 ≥ 5** - リアルタイム計算
3. ✓ **石倉上に登坂** - 手動トグルで ON

**Fallback Mode時の変更:**
- 目標投入数: 10本 → 5本
- V-Goal目標時刻: 2:00 → 2:20

#### 残弾・得点カウンター（AmmoScoreCounter）

**小うなぎ残弾:**
- 初期値: 20本
- 各カゴへの投入（+ボタン）で自動減少

**推定スコア（自動計算）:**

| 条件 | スコア |
|------|--------|
| 大うなぎ捕獲 | +20点 |
| 石倉登坂実績 | +5点 |
| 最遠青カゴの投入 | 1本 = 25点 |
| 緑カゴの投入 | 1本 = 20点 |
| その他青カゴの投入 | 1本 = 10点 |

## アーキテクチャ

### State Management（GameState クラス）

全体の状態を一元管理。ROS 2への対応が容易な設計：

```javascript
// GameState インスタンス
const gameState = new GameState();

// 状態を購読
gameState.subscribe((state) => {
  console.log("状態が更新されました:", state.toJSON());
});

// 状態を変更
gameState.tick();                                 // タイマー進行
gameState.setBigUnagiCaptured(true);             // 大うなぎON
gameState.updateBasketOurCount("green_a", 1);   // 投入
```

### 状態の JSON 形式

デバッグやROS 2連携用に `toJSON()` メソッド提供：

```json
{
  "elapsed_seconds": 65,
  "total_seconds": 180,
  "is_running": true,
  "current_phase": 4,
  "fallback_mode": false,
  "baskets": [
    {
      "id": "green_a",
      "our_count": 3,
      "their_count": 1,
      "is_occupied": true
    }
  ],
  "big_unagi_captured": true,
  "is_on_ishikura": true,
  "ammo_remaining": 17,
  "ammo_deployed": 3,
  "score": 85,
  "ready_for_v_goal": false
}
```

## デザイン指針

### カラースキーム

- **背景** - `#0a0e27`（深い藍色）
- **成功状態** - `#00ff88`（ネオングリーン）
- **警告状態** - `#ffff00`（ネオンイエロー）
- **エラー状態** - `#ff3366`（ネオンレッド）
- **情報状態** - `#00ffff`（ネオンシアン）
- **V-Goal** - `#ff00ff`（ネオンマゼンタ）

### フォント

- `'Courier New', monospace` - 等幅フォント（コンピュータ風）
- 大きめのフォントサイズ（暗い会場での視認性重視）

### アニメーション

- **V-Goal Alert** - パルス点滅（0.6秒周期）
- **ボタンホバー** - スケール 1.05x + 明度UP
- **現在フェーズ** - グローエフェクト

## ROS 2 連携（拡張予定）

現在はダミーデータで動作していますが、以下のように拡張可能：

```javascript
import ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

// トピックを購読
const unagiTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/unagi_status',
  messageType: 'std_msgs/Bool'
});

unagiTopic.subscribe((message) => {
  gameState.setBigUnagiCaptured(message.data);
});
```

## トラブルシューティング

### ページが真っ黒のまま表示されない

1. ブラウザの開発者ツール（F12）でコンソールエラーを確認
2. `npm install` を再実行
3. `npm start` を再実行

### ボタンが反応しない

- JavaScriptが有効になっているか確認
- ブラウザのリロード（Ctrl+R）

### スコアが計算されない

- 各コンポーネントの状態が正しく更新されているか確認
- ブラウザコンソールで `gameState.toJSON()` を実行

## ライセンス

Natsurobo 2026 チーム内での使用を想定しています。

## 作成者

GitHub Copilot - 2026年5月20日

---

**注意:** このアプリケーションは実験的です。本番環境使用前に十分なテストを実施してください。
