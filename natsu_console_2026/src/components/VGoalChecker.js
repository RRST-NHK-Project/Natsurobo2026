import React from "react";

export const VGoalChecker = ({ gameState }) => {
  const isReady      = gameState.isReadyForVGoal();
  const occupiedCount = gameState.getOccupiedBasketsCount();

  const targetTime = gameState.getVGoalTargetTime();
  const targetMins = Math.floor(targetTime / 60);
  const targetSecs = String(targetTime % 60).padStart(2, "0");

  return (
    <>
      {isReady && (
        <div className="vgoal-ready">
          <div className="vgoal-ready-title">大漁合図</div>
          <div className="vgoal-ready-sub">READY TO V-GOAL</div>
        </div>
      )}

      {/* 大うなぎ条件 */}
      <div
        className={`condition-card ${gameState.bigUnagiCaptured ? "checked" : ""}`}
        onClick={() => gameState.setBigUnagiCaptured(!gameState.bigUnagiCaptured)}
      >
        <div className="condition-card-row">
          <span className="condition-label">大うなぎ捕獲</span>
          <div className={`check-circle ${gameState.bigUnagiCaptured ? "checked" : ""}`}>
            {gameState.bigUnagiCaptured ? "✓" : ""}
          </div>
        </div>
      </div>

      {/* 石倉条件 */}
      <div
        className={`condition-card ${gameState.isOnishigura ? "checked" : ""}`}
        onClick={() => gameState.setOnishigura(!gameState.isOnishigura)}
      >
        <div className="condition-card-row">
          <span className="condition-label">石倉登坂</span>
          <div className={`check-circle ${gameState.isOnishigura ? "checked" : ""}`}>
            {gameState.isOnishigura ? "✓" : ""}
          </div>
        </div>
      </div>

      {/* カゴ占有条件 */}
      <div className={`condition-card ${occupiedCount >= 5 ? "checked" : ""}`}>
        <div className="condition-card-row">
          <span className="condition-label">カゴ占有</span>
          <div className={`check-circle ${occupiedCount >= 5 ? "checked" : ""}`}>
            {occupiedCount >= 5 ? "✓" : ""}
          </div>
        </div>
        <div className="dots-row">
          {Array.from({ length: 5 }).map((_, i) => (
            <div key={i} className={`dot ${i < occupiedCount ? "filled" : ""}`} />
          ))}
        </div>
        <div className={`condition-sub ${occupiedCount >= 5 ? "met" : ""}`}>
          {occupiedCount} / 5 &nbsp;{occupiedCount >= 5 ? "— 達成" : `— あと ${5 - occupiedCount} 個`}
        </div>
      </div>

      {/* V-Goal目標情報 */}
      <div className="vgoal-info-box">
        <div className="info-row">
          <span className="info-row-label">目標時刻</span>
          <span className="info-row-value">
            {targetMins}:{targetSecs}
            {gameState.fallbackMode && <span style={{ fontSize: "10px", color: "var(--orange)", marginLeft: 4 }}>FB</span>}
          </span>
        </div>
        <div className="info-row">
          <span className="info-row-label">目標投入数</span>
          <span className="info-row-value">{gameState.getTargetDropCount()} 本</span>
        </div>
      </div>
    </>
  );
};
