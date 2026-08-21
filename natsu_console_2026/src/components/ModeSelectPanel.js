import React from "react";

// 手動操作モードの直接選択。
// mc_2026 のSHAREは Drive -> Get_eel -> Shoot の3巡送りしかないので、
// 目的のモードまで何回押すか数える必要があって試合中に事故る。
// ここから /manual/mode_set (Int32: 0/1/2) を出して一発で飛べるようにする。
// 現在モードの表示は mc_2026 が定常publishしている /manual/mode を使う。
// 色は機体の状態表示LED(natsu_ir/src/ir_led_policy.py の MODE_COLORS)および
// PS4ControllerPanel のモードバッジと同じ値。片方だけ変えないこと。
export const MANUAL_MODES = [
  { code: 0, key: "DRIVE", ja: "走行", en: "Drive", color: "#58a6ff" },
  { code: 1, key: "GET_EEL", ja: "捕獲", en: "Get eel", color: "#3fb950" },
  { code: 2, key: "SHOOT", ja: "射出", en: "Shoot", color: "#f0883e" },
];

export function ModeSelectPanel({
  manualMode,
  onSelect,
  operationArmed,
  compact = false,
  tr,
}) {
  const t = tr || ((ja) => ja);

  const buttons = (
    <div style={{ display: "flex", gap: 10, flexWrap: "wrap" }}>
      {MANUAL_MODES.map((m) => {
        const active = manualMode === m.key;
        return (
          <button
            key={m.code}
            type="button"
            disabled={!operationArmed}
            onClick={() => onSelect(m.code)}
            style={{
              flex: "1 1 120px",
              minWidth: 110,
              padding: compact ? "10px 12px" : "14px 12px",
              borderRadius: 10,
              cursor: operationArmed ? "pointer" : "not-allowed",
              fontSize: compact ? 14 : 16,
              fontWeight: 700,
              opacity: operationArmed ? 1 : 0.5,
              color: active ? "#0d1117" : m.color,
              background: active ? m.color : "rgba(255,255,255,0.05)",
              border: `1px solid ${m.color}`,
              boxShadow: active ? `0 0 0 2px ${m.color}55` : "none",
              transition: "background 0.15s, color 0.15s",
            }}
          >
            {t(m.ja, m.en)}
            {active ? " ●" : ""}
          </button>
        );
      })}
    </div>
  );

  if (compact) {
    return buttons;
  }

  return (
    <section className="pose-panel">
      <h2 className="serial-packet-title">
        {t("操作モード選択", "Manual Mode")}
      </h2>
      <p className="connection-hint" style={{ margin: "0 6px 14px" }}>
        {t(
          "モードを直接選びます (/manual/mode_set)。PS4のSHAREでも順送りで切り替えられますが、こちらは一発で飛べます。射出中に切り替えると射出モーターは自動で止まります。",
          "Jump straight to a mode (/manual/mode_set). SHARE on the PS4 still cycles through them. Switching stops the shooter motors automatically."
        )}
      </p>

      {!operationArmed && (
        <div
          style={{
            margin: "0 6px 12px",
            padding: "8px 12px",
            borderRadius: 8,
            background: "rgba(248, 81, 73, 0.14)",
            border: "1px solid rgba(248, 81, 73, 0.5)",
            color: "#f85149",
            fontSize: 13,
          }}
        >
          {t("操作許可がOFFです。モードを変えるには操作許可をONにしてください。",
             "Operation is locked. Enable operation to change the mode.")}
        </div>
      )}

      {buttons}

      <div style={{ marginTop: 14, color: "#8b949e", fontSize: 13 }}>
        {t("機体の現在モード", "Robot mode")}:{" "}
        <strong style={{ color: MANUAL_MODES.find((m) => m.key === manualMode)?.color || "#8b949e" }}>
          {MANUAL_MODES.find((m) => m.key === manualMode)
            ? t(MANUAL_MODES.find((m) => m.key === manualMode).ja,
                MANUAL_MODES.find((m) => m.key === manualMode).en)
            : t("未受信", "No signal")}
        </strong>
      </div>
    </section>
  );
}
