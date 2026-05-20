import React from "react";

export const TimerPhaseController = ({ gameState }) => {
  const currentPhase = gameState.getCurrentPhase();
  const progress     = gameState.getPhaseProgress();

  return (
    <div className="c-phase-strip">
      {gameState.phases.map(phase => {
        const isActive    = phase.id === currentPhase.id;
        const isCompleted = phase.id < currentPhase.id;
        const segClass    = isActive ? "active" : isCompleted ? "done" : "";

        return (
          <div
            key={phase.id}
            className={`phase-seg ${segClass}`}
            style={{
              flex: phase.end - phase.start,
              "--progress": isActive
                ? `${progress * 100}%`
                : isCompleted ? "100%" : "0%"
            }}
          >
            <span className="phase-seg-label">
              P{phase.id}
            </span>
          </div>
        );
      })}
    </div>
  );
};
