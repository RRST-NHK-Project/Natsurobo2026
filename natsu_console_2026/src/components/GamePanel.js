import React, { useEffect, useState } from "react";
import { GameState } from "../store/GameState";
import { TimerPhaseController } from "./TimerPhaseController";
import { BasketMonitor } from "./BasketMonitor";
import { VGoalChecker } from "./VGoalChecker";
import { AmmoScoreCounter } from "./AmmoScoreCounter";
import "./GamePanel.css";

/**
 * GamePanel
 * 旧 natsu_console のゲーム管理UI（タイマー / フェーズ / Vゴール / カゴ占有 /
 * 小うなぎ残弾 / スコア）を自己完結したパネルとして提供する。
 * 自前の GameState を保持し、R2スタイルのコンソール内の1タブとして描画される。
 * スタイルはすべて .game-panel-root 配下にスコープされる。
 */
export function GamePanel() {
  const [gameState] = useState(() => new GameState());
  const [, setTick] = useState(0);

  useEffect(() => {
    const id = setInterval(() => gameState.tick(), 1000);
    return () => clearInterval(id);
  }, [gameState]);

  useEffect(() => {
    const listener = () => setTick(n => n + 1);
    gameState.subscribe(listener);
    return () => gameState.unsubscribe(listener);
  }, [gameState]);

  const elapsed   = gameState.elapsedSeconds;
  const remaining = gameState.totalSeconds - elapsed;
  const mins      = Math.floor(elapsed / 60);
  const secs      = elapsed % 60;
  const timeStr   = `${mins}:${secs.toString().padStart(2, "0")}`;
  const timerClass = remaining <= 10 ? "critical" : remaining <= 30 ? "warning" : "";

  return (
    <div className="game-panel-root">
      <div className="game-console">
        {/* Header */}
        <header className="c-header">
          <span className="brand">Natsurobo 2026</span>
          <div className="header-divider" />

          <div className="timer-block">
            <span className={`timer-display ${timerClass}`}>{timeStr}</span>
            <span className="timer-total">/ 3:00</span>
          </div>

          <button
            className={`start-btn ${gameState.isRunning ? "running" : "stopped"}`}
            onClick={() => gameState.setRunning(!gameState.isRunning)}
          >
            {gameState.isRunning ? "■ STOP" : "▶ START"}
          </button>

          <div className="ml-auto" />

          <div className="header-score">
            <span className="header-score-label">Score</span>
            <span className="header-score-value">{gameState.calculateScore()} pt</span>
          </div>

          <div className="header-divider" />

          <button
            className={`fallback-chip ${gameState.fallbackMode ? "on" : "off"}`}
            onClick={() => gameState.setFallbackMode(!gameState.fallbackMode)}
          >
            ⚠ Fallback {gameState.fallbackMode ? "ON" : "OFF"}
          </button>
        </header>

        {/* Phase strip */}
        <TimerPhaseController gameState={gameState} />

        {/* Main content */}
        <main className="c-main">
          <aside className="c-left">
            <VGoalChecker gameState={gameState} />
          </aside>
          <section className="c-right">
            <BasketMonitor gameState={gameState} />
          </section>
        </main>

        {/* Footer */}
        <footer className="c-footer">
          <AmmoScoreCounter gameState={gameState} />
        </footer>
      </div>
    </div>
  );
}
