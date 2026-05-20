import React from "react";

const BasketCard = ({ basket, gameState }) => {
  const isOccupied = basket.our_count > basket.their_count;
  const isSelected = gameState.currentTargetId === basket.id;

  return (
    <div
      className={`basket-card ${isOccupied ? "occupied" : ""} ${isSelected ? "selected" : ""}`}
      onClick={() => gameState.setCurrentTarget(basket.id)}
    >
      <div className="basket-top-row">
        <span className="basket-name">{basket.name}</span>
        {basket.isTarget && <span className="basket-star">★</span>}
      </div>

      <div className="basket-counts">
        <div className="count-block">
          <span className={`count-num our ${basket.our_count === 0 ? "zero" : ""}`}>
            {basket.our_count}
          </span>
          <span className="count-label">自</span>
        </div>
        <span className="count-sep">/</span>
        <div className="count-block">
          <span className={`count-num their ${basket.their_count === 0 ? "zero" : ""}`}>
            {basket.their_count}
          </span>
          <span className="count-label">相</span>
        </div>
      </div>

      <div className="basket-btns" onClick={e => e.stopPropagation()}>
        <button
          className="bbt bbt-our"
          onClick={() => gameState.updateBasketOurCount(basket.id, 1)}
        >
          +自
        </button>
        <button
          className="bbt bbt-their"
          onClick={() => gameState.updateBasketTheirCount(basket.id, 1)}
        >
          +相
        </button>
      </div>

      <div className={`basket-status ${isOccupied ? "occupied" : "empty"}`}>
        {isOccupied ? "✓ 占有中" : "未占有"}
      </div>
    </div>
  );
};

export const BasketMonitor = ({ gameState }) => {
  const targets = gameState.baskets.filter(b => b.isTarget);
  const others  = gameState.baskets.filter(b => !b.isTarget);

  return (
    <>
      <div className="section-label">優先ターゲット ({targets.length})</div>
      <div className="basket-grid">
        {targets.map(b => <BasketCard key={b.id} basket={b} gameState={gameState} />)}
      </div>

      {others.length > 0 && (
        <>
          <div className="section-label" style={{ marginTop: 14 }}>その他 ({others.length})</div>
          <div className="basket-grid">
            {others.map(b => <BasketCard key={b.id} basket={b} gameState={gameState} />)}
          </div>
        </>
      )}

      <div className="basket-grid-hint">
        占有: {gameState.getOccupiedBasketsCount()} / {gameState.baskets.length} &nbsp;·&nbsp; カードをクリックでターゲット選択
      </div>
    </>
  );
};
