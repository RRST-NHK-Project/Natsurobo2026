import React from "react";

export function PS4ControllerPanel({
  buttons,
  axes,
  getButtonPressProps,
  getAxisPressProps,
  getTriggerPressProps,
  controllerEnabled,
  setControllerEnabled,
  onEnterFullscreen,
  onExitFullscreen,
  fullscreen,
  joyTopicInput,
  setJoyTopicInput,
  joyTopicName,
  applyJoyTopicName,
  commandValue,
  updateCommand,
  tr,
}) {
  const ps4Buttons = (
    <>
      <div className="ps-shoulder-row">
        <div className="ps-shoulder-side">
          <button className={`ps-button ps-shoulder ${buttons[6] === 1 ? "ps-active" : ""}`} {...getTriggerPressProps(6, 2)}>
            L2
          </button>
          <button className={`ps-button ps-shoulder ${buttons[4] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(4)}>
            L1
          </button>
        </div>
        <div className="ps-shoulder-side ps-shoulder-side-right">
          <button className={`ps-button ps-shoulder ${buttons[5] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(5)}>
            R1
          </button>
          <button className={`ps-button ps-shoulder ${buttons[7] === 1 ? "ps-active" : ""}`} {...getTriggerPressProps(7, 5)}>
            R2
          </button>
        </div>
      </div>

      <div className={fullscreen ? "ps-main-row-fullscreen" : "ps-main-row"}>
        <div className={fullscreen ? "dpad-grid-fullscreen" : "dpad-grid"}>
          <button className={`dpad-button ${axes[7] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(7, 1)}>
            ↑
          </button>
          <button className={`dpad-button ${axes[6] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(6, -1)}>
            ←
          </button>
          <button className={`dpad-button ${axes[6] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(6, 1)}>
            →
          </button>
          <button className={`dpad-button ${axes[7] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(7, -1)}>
            ↓
          </button>
        </div>

        {fullscreen ? null : (
          <div className="ps-system-row">
            <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
              SHARE
            </button>
            <button className={`ps-button ps-system ps-home ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
              PS
            </button>
            <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
              OPTIONS
            </button>
          </div>
        )}

        <div className={fullscreen ? "face-grid-fullscreen" : "face-grid"}>
          <button className={`ps-button ps-face ps-triangle ${buttons[2] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(2)}>
            △
          </button>
          <button className={`ps-button ps-face ps-square ${buttons[3] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(3)}>
            □
          </button>
          <button className={`ps-button ps-face ps-circle ${buttons[1] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(1)}>
            ○
          </button>
          <button className={`ps-button ps-face ps-cross ${buttons[0] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(0)}>
            ×
          </button>
        </div>
      </div>

      {fullscreen && (
        <div className="ps-system-row-fullscreen">
          <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
            SHARE
          </button>
          <button className={`ps-button ps-system ps-home ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
            PS
          </button>
          <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
            OPTIONS
          </button>
        </div>
      )}

      <div className={fullscreen ? "ps-stick-row-fullscreen" : "ps-stick-row"}>
        <button className={`ps-button ps-stick ${buttons[11] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(11)}>
          L3
        </button>
        <button className={`ps-button ps-stick ${buttons[12] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(12)}>
          R3
        </button>
      </div>

      {/* 移動(LS: axes[0]/[1])・旋回(RS: axes[3])。joy_nodeの実機マッピングに合わせ、左・上=+1 */}
      <div className={fullscreen ? "ps-main-row-fullscreen" : "ps-main-row"}>
        <div className={fullscreen ? "dpad-grid-fullscreen" : "dpad-grid"}>
          <button className={`dpad-button ${axes[1] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(1, 1)}>
            ⬆
          </button>
          <button className={`dpad-button ${axes[0] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(0, 1)}>
            ⬅
          </button>
          <button className={`dpad-button ${axes[0] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(0, -1)}>
            ➡
          </button>
          <button className={`dpad-button ${axes[1] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(1, -1)}>
            ⬇
          </button>
        </div>
        <div className="ps-rotate-column">
          <button className={`dpad-button ${axes[3] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(3, 1)}>
            ⟲
          </button>
          <button className={`dpad-button ${axes[3] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(3, -1)}>
            ⟳
          </button>
        </div>
      </div>
      <p className="connection-hint">
        {tr("移動・旋回はR2を押しながら（LS/RS相当）", "Hold R2 while moving/rotating (acts as LS/RS)")}
      </p>
    </>
  );

  if (fullscreen) {
    return (
      <div className="console-page console-page-fullscreen">
        <main className="console-card console-card-fullscreen-controller">
          <button className="fullscreen-close-button-top" onClick={onExitFullscreen}>
            {tr("✕ 通常表示に戻る", "✕ Back to Normal")}
          </button>
          <div className="ps4-panel-fullscreen">
            {ps4Buttons}
          </div>
        </main>
      </div>
    );
  }

  return (
    <section className="serial-bridge-panel">
      <section className="joy-topic-row">
        <input
          className="connection-input"
          value={joyTopicInput}
          onChange={(e) => setJoyTopicInput(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === "Enter") applyJoyTopicName();
          }}
          placeholder={tr("Joy Topic Name (例: joy)", "Joy Topic Name (e.g. joy)")}
        />
        <button className="connection-button btn-connect" onClick={applyJoyTopicName}>
          {tr("更新", "Apply")}
        </button>
      </section>

      <p className="connection-hint">{tr("現在のJoyトピック:", "Current Joy topic:")} {joyTopicName}</p>

      <section className="control-toggle-row">
        <button
          className={`toggle-button ${controllerEnabled ? "toggle-on" : "toggle-off"}`}
          onClick={() => setControllerEnabled(!controllerEnabled)}
        >
          {controllerEnabled ? tr("コントローラー: ON", "Controller: ON") : tr("コントローラー: OFF", "Controller: OFF")}
        </button>
        <button className="fullscreen-toggle-button" onClick={onEnterFullscreen}>
          {tr("全画面操作", "Fullscreen")}
        </button>
      </section>

      {controllerEnabled && (
        <div className="controller-layout">
          <section className="quick-controls-panel">
            <div className="quick-controls-row">
              <div className="control-group">
                <button
                  className="control-button control-button-emergency emergency-stop-shape"
                  onClick={() => updateCommand(0)}
                >
                  {tr("緊急停止", "Emergency Stop")}
                </button>
              </div>
            </div>
            <p className="velocity-readout">command: {commandValue}</p>
          </section>

          <section className="ps4-panel">
            <h2>{tr("PS4 コントローラーボタン", "PS4 Controller Buttons")}</h2>
            {ps4Buttons}
          </section>
        </div>
      )}

      {!controllerEnabled && (
        <div className="disabled-notice">
          {tr("コントローラーは無効化されています", "Controller is disabled")}
        </div>
      )}
    </section>
  );
}
