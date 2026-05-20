import React from "react";

export const AmmoScoreCounter = ({ gameState }) => {
  const remaining  = gameState.ammoRemaining;
  const pct        = (remaining / 20) * 100;
  const levelClass = remaining > 10 ? "full" : remaining > 5 ? "mid" : "low";
  const deployed   = gameState.ammoDeployed;
  const target     = gameState.getTargetDropCount();

  return (
    <>
      <div className="ammo-wrapper">
        <span className="ammo-label">残弾</span>
        <span className={`ammo-num ${levelClass}`}>{remaining}</span>
        <div className="ammo-track">
          <div className={`ammo-fill ${levelClass}`} style={{ width: `${pct}%` }} />
        </div>
        <span className="ammo-stats">
          投入 {deployed} / 20 本 &nbsp;·&nbsp; 目標
          {deployed >= target
            ? <span style={{ color: "var(--green)", marginLeft: 4 }}>✓ {target}本達成</span>
            : <span style={{ color: "var(--yellow)", marginLeft: 4 }}>あと {target - deployed}本</span>
          }
        </span>
      </div>

      <div className="footer-sep" />

      <div className="footer-score-block">
        <span className="footer-score-label">Score</span>
        <span className="footer-score-value">{gameState.calculateScore()} pt</span>
      </div>
    </>
  );
};
