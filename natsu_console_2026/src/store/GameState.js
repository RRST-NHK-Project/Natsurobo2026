/**
 * GameState - 全体の状態管理
 * ROS 2への対応が容易な設計
 */
export class GameState {
  constructor() {
    this.reset();
    this.listeners = [];
  }

  reset() {
    this.elapsedSeconds = 0;
    this.totalSeconds = 180;
    this.isRunning = false;
    this.fallbackMode = false;

    // フェーズ定義
    this.phases = [
      { id: 1, start: 0, end: 10, name: "Phase 1: スタート・移動" },
      { id: 2, start: 10, end: 35, name: "Phase 2: 大うなぎ捕獲" },
      { id: 3, start: 35, end: 60, name: "Phase 3: 対角移動・石倉登坂" },
      { id: 4, start: 60, end: 115, name: "Phase 4: 小うなぎ投入" },
      { id: 5, start: 115, end: 120, name: "Phase 5: Vゴール合図" },
      { id: 6, start: 120, end: 180, name: "Phase 6: バッファ" }
    ];

    // カゴ情報（7個）
    this.baskets = [
      { id: "green_a", name: "緑 A", type: "green", our_count: 0, their_count: 0, isTarget: true },
      { id: "green_b", name: "緑 B", type: "green", our_count: 0, their_count: 0, isTarget: true },
      { id: "farthest_blue", name: "最遠青（対角）", type: "blue_farthest", our_count: 0, their_count: 0, isTarget: true },
      { id: "nearby_blue_a", name: "近接青 A", type: "blue_nearby", our_count: 0, their_count: 0, isTarget: true },
      { id: "nearby_blue_b", name: "近接青 B", type: "blue_nearby", our_count: 0, their_count: 0, isTarget: true },
      { id: "other_blue_c", name: "その他青 C", type: "blue_other", our_count: 0, their_count: 0, isTarget: false },
      { id: "other_blue_d", name: "その他青 D", type: "blue_other", our_count: 0, their_count: 0, isTarget: false }
    ];

    // V-Goal条件
    this.bigUnagiCaptured = false;
    this.isOnIshikura = false;

    // 小うなぎ情報
    this.ammoRemaining = 20;
    this.ammoDeployed = 0;

    // 現在のターゲット
    this.currentTargetId = null;

    // Fallback関連
    this.hasReachedIshikura = false; // 一度でも石倉に到達した記録
  }

  /**
   * タイマーを進める（毎秒呼び出し）
   */
  tick() {
    if (this.isRunning && this.elapsedSeconds < this.totalSeconds) {
      this.elapsedSeconds += 1;

      // Phase 3（時間 35-60秒）を過ぎて、Fallback未設定の場合、自動的にFallbackに切り替え
      if (!this.fallbackMode && this.elapsedSeconds >= 60 && !this.hasReachedIshikura) {
        this.setFallbackMode(true);
      }

      if (this.elapsedSeconds >= this.totalSeconds) {
        this.isRunning = false;
      }

      this.notifyListeners();
    }
  }

  /**
   * タイマーの開始/停止
   */
  setRunning(running) {
    this.isRunning = running;
    this.notifyListeners();
  }

  /**
   * 経過時間を設定
   */
  setElapsedSeconds(seconds) {
    this.elapsedSeconds = Math.max(0, Math.min(seconds, this.totalSeconds));
    this.notifyListeners();
  }

  /**
   * 現在のフェーズを取得
   */
  getCurrentPhase() {
    for (const phase of this.phases) {
      if (this.elapsedSeconds >= phase.start && this.elapsedSeconds < phase.end) {
        return phase;
      }
    }
    return this.phases[this.phases.length - 1];
  }

  /**
   * フェーズの進捗率（0-1）を取得
   */
  getPhaseProgress() {
    const phase = this.getCurrentPhase();
    const progress = (this.elapsedSeconds - phase.start) / (phase.end - phase.start);
    return Math.max(0, Math.min(1, progress));
  }

  /**
   * 占有状況を計算（is_occupiedフラグ）
   */
  updateBasketOccupancy() {
    this.baskets.forEach(basket => {
      basket.is_occupied = basket.our_count > basket.their_count;
    });
  }

  /**
   * 占有カゴ数を計算
   */
  getOccupiedBasketsCount() {
    this.updateBasketOccupancy();
    return this.baskets.filter(b => b.is_occupied).length;
  }

  /**
   * V-Goal条件をチェック
   */
  isReadyForVGoal() {
    this.updateBasketOccupancy();
    return (
      this.bigUnagiCaptured &&
      this.isOnIshikura &&
      this.getOccupiedBasketsCount() >= 5
    );
  }

  /**
   * Fallback Modeを設定
   */
  setFallbackMode(enabled) {
    this.fallbackMode = enabled;
    if (enabled) {
      this.hasReachedIshikura = false;
    }
    this.notifyListeners();
  }

  /**
   * 石倉到達フラグを設定
   */
  setOnIshikura(onIshikura) {
    this.isOnIshikura = onIshikura;
    if (onIshikura) {
      this.hasReachedIshikura = true;
    }
    this.notifyListeners();
  }

  /**
   * 大うなぎ捕獲フラグを設定
   */
  setBigUnagiCaptured(captured) {
    this.bigUnagiCaptured = captured;
    this.notifyListeners();
  }

  /**
   * カゴの our_count を変更
   */
  updateBasketOurCount(basketId, delta) {
    const basket = this.baskets.find(b => b.id === basketId);
    if (basket) {
      basket.our_count = Math.max(0, basket.our_count + delta);
      if (delta > 0 && this.ammoRemaining > 0) {
        this.ammoRemaining -= delta;
        this.ammoDeployed += delta;
      }
      this.updateBasketOccupancy();
      this.notifyListeners();
    }
  }

  /**
   * カゴの their_count を変更
   */
  updateBasketTheirCount(basketId, delta) {
    const basket = this.baskets.find(b => b.id === basketId);
    if (basket) {
      basket.their_count = Math.max(0, basket.their_count + delta);
      this.updateBasketOccupancy();
      this.notifyListeners();
    }
  }

  /**
   * ターゲットを選択
   */
  setCurrentTarget(basketId) {
    this.currentTargetId = basketId;
    this.notifyListeners();
  }

  /**
   * スコアを計算（仕様に基づく）
   */
  calculateScore() {
    let score = 0;

    // 大うなぎ捕獲: +20点
    if (this.bigUnagiCaptured) {
      score += 20;
    }

    // 石倉上に到達した実績: +5点
    if (this.hasReachedIshikura) {
      score += 5;
    }

    // カゴ別スコア計算
    this.baskets.forEach(basket => {
      if (basket.type === "blue_farthest") {
        // 最遠青カゴ: 1本につき 25点
        score += basket.our_count * 25;
      } else if (basket.type === "green") {
        // 緑カゴ: 1本につき 20点
        score += basket.our_count * 20;
      } else if (basket.type === "blue_nearby" || basket.type === "blue_other") {
        // その他青カゴ: 1本につき 10点
        score += basket.our_count * 10;
      }
    });

    return score;
  }

  /**
   * V-Goal目標時刻を取得（Fallback対応）
   */
  getVGoalTargetTime() {
    return this.fallbackMode ? 140 : 120; // Fallback時は2:20（140秒）、通常は2:00（120秒）
  }

  /**
   * 目標投入数を取得（Fallback対応）
   */
  getTargetDropCount() {
    return this.fallbackMode ? 5 : 10; // Fallback時は5本、通常は10-12本の中位値10本
  }

  /**
   * 状態変更を購読
   */
  subscribe(listener) {
    this.listeners.push(listener);
  }

  /**
   * 購読を解除
   */
  unsubscribe(listener) {
    this.listeners = this.listeners.filter(l => l !== listener);
  }

  /**
   * リスナーに通知
   */
  notifyListeners() {
    this.listeners.forEach(listener => listener(this));
  }

  /**
   * 状態をJSON形式で取得（デバッグ・ROS 2連携用）
   */
  toJSON() {
    return {
      elapsed_seconds: this.elapsedSeconds,
      total_seconds: this.totalSeconds,
      is_running: this.isRunning,
      current_phase: this.getCurrentPhase().id,
      fallback_mode: this.fallbackMode,
      baskets: this.baskets.map(b => ({
        id: b.id,
        our_count: b.our_count,
        their_count: b.their_count,
        is_occupied: b.is_occupied
      })),
      big_unagi_captured: this.bigUnagiCaptured,
      is_on_ishikura: this.isOnIshikura,
      ammo_remaining: this.ammoRemaining,
      ammo_deployed: this.ammoDeployed,
      score: this.calculateScore(),
      ready_for_v_goal: this.isReadyForVGoal()
    };
  }
}
