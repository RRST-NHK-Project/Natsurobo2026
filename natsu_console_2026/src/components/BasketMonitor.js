import React from "react";

const shoot_modeCard = ({ shoot_mode, gameState }) => {
  const isOccupied = shoot_mode.our_count > shoot_mode.their_count;
  const isSelected = gameState.currentTargetId === shoot_mode.id;

  return (
    <div
      className={`shoot_mode-card ${isOccupied ? "occupied" : ""} ${isSelected ? "selected" : ""}`}
      onClick={() => gameState.setCurrentTarget(shoot_mode.id)}
    >
      <div className="shoot_mode-top-row">
        <span className="shoot_mode-name">{shoot_mode.name}</span>
        {shoot_mode.isTarget && <span className="shoot_mode-star">★</span>}
      </div>

      <div className="shoot_mode-counts">
        <div className="count-block">
          <span className={`count-num our ${shoot_mode.our_count === 0 ? "zero" : ""}`}>
            {shoot_mode.our_count}
          </span>
          <span className="count-label">自</span>
        </div>
        <span className="count-sep">/</span>
        <div className="count-block">
          <span className={`count-num their ${shoot_mode.their_count === 0 ? "zero" : ""}`}>
            {shoot_mode.their_count}
          </span>
          <span className="count-label">相</span>
        </div>
      </div>

      <div className="shoot_mode-btns" onClick={e => e.stopPropagation()}>
        <button
          className="bbt bbt-our"
          onClick={() => gameState.updateshoot_modeOurCount(shoot_mode.id, 1)}
        >
          +自
        </button>
        <button
          className="bbt bbt-their"
          onClick={() => gameState.updateshoot_modeTheirCount(shoot_mode.id, 1)}
        >
          +相
        </button>
      </div>

      <div className={`shoot_mode-status ${isOccupied ? "occupied" : "empty"}`}>
        {isOccupied ? "✓ 占有中" : "未占有"}
      </div>
    </div>
  );
};

export const shoot_modeMonitor = ({ gameState }) => {
  const targets = gameState.shoot_modes.filter(b => b.isTarget);
  const others  = gameState.shoot_modes.filter(b => !b.isTarget);

  return (
    <>
      <div className="section-label">優先ターゲット ({targets.length})</div>
      <div className="shoot_mode-grid">
        {targets.map(b => <shoot_modeCard key={b.id} shoot_mode={b} gameState={gameState} />)}
      </div>

      {others.length > 0 && (
        <>
          <div className="section-label" style={{ marginTop: 14 }}>その他 ({others.length})</div>
          <div className="shoot_mode-grid">
            {others.map(b => <shoot_modeCard key={b.id} shoot_mode={b} gameState={gameState} />)}
          </div>
        </>
      )}

      <div className="shoot_mode-grid-hint">
        占有: {gameState.getOccupiedshoot_modesCount()} / {gameState.shoot_modes.length} &nbsp;·&nbsp; カードをクリックでターゲット選択
      </div>
    </>
  );
};
