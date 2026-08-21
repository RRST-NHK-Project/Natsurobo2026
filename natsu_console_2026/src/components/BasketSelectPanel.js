import React from "react";
import { FieldMap } from "./FieldMap";

// SHOOTモード用のカゴ直接指定パネル。
// PS4コントローラの代わりに、FieldMap(フィールド俯瞰・静的)と7個のカゴボタンを表示し、
// クリックしたカゴのセル番号(0〜6)を onSelect で /manual/basket へ送る。
export function BasketSelectPanel({
  defs,
  selectedCell,
  onSelect,
  operationArmed,
  tr,
}) {
  const t = tr || ((ja) => ja);

  return (
    <section className="pose-panel">
      <h2 className="serial-packet-title">
        {t("カゴ指定 (SHOOT)", "Basket Select (SHOOT)")}
      </h2>
      <p className="connection-hint" style={{ margin: "0 6px 14px" }}>
        {t(
          "SHOOTモード中はPS4コントローラの代わりにカゴを直接指定します。カゴを選ぶとセル番号を /manual/basket へ発行し、mc_2026 がサーボ(yaw/pitch)を照準します。",
          "In SHOOT mode, pick a basket directly instead of using the PS4 controller. Selecting a basket publishes its cell index to /manual/basket, and mc_2026 aims the servos (yaw/pitch)."
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
          {t("操作許可がOFFです。カゴを指定するには操作許可をONにしてください。",
             "Operation is locked. Enable operation to send a basket selection.")}
        </div>
      )}

      <div style={{ display: "flex", gap: 20, flexWrap: "wrap", alignItems: "flex-start" }}>
        <div style={{ flex: "0 0 auto" }}>
          <FieldMap selectedId={defs.find((d) => d.cell === selectedCell)?.id} />
        </div>

        <div style={{ flex: "1 1 260px", minWidth: 240 }}>
          <div style={{ color: "#8b949e", fontSize: 12, marginBottom: 8 }}>
            {t("カゴを選択", "Select a basket")}
          </div>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: "repeat(2, 1fr)",
              gap: 10,
            }}
          >
            {defs.map((def) => {
              const isSelected = selectedCell === def.cell;
              const isBlue = def.color === 1;
              return (
                <button
                  key={def.id}
                  type="button"
                  disabled={!operationArmed}
                  onClick={() => onSelect(def.cell)}
                  style={{
                    padding: "14px 12px",
                    borderRadius: 10,
                    cursor: operationArmed ? "pointer" : "not-allowed",
                    fontSize: 16,
                    fontWeight: 700,
                    color: "#fff",
                    opacity: operationArmed ? 1 : 0.5,
                    border: isSelected
                      ? "2px solid #f2cc60"
                      : "1px solid rgba(255,255,255,0.15)",
                    background: isSelected
                      ? (isBlue ? "rgba(56,139,253,0.55)" : "rgba(63,185,80,0.55)")
                      : (isBlue ? "rgba(56,139,253,0.22)" : "rgba(63,185,80,0.22)"),
                    boxShadow: isSelected ? "0 0 0 2px rgba(242,204,96,0.35)" : "none",
                    transition: "background 0.15s, border-color 0.15s",
                  }}
                >
                  <div>{def.label}{isSelected ? " ★" : ""}</div>
                  <div style={{ fontSize: 11, fontWeight: 400, opacity: 0.8 }}>
                    {t("セル", "cell")} {def.cell}
                  </div>
                </button>
              );
            })}
          </div>

          <div style={{ marginTop: 14, color: "#8b949e", fontSize: 13 }}>
            {t("選択中", "Selected")}:{" "}
            <strong style={{ color: "#58a6ff" }}>
              {selectedCell == null
                ? "-"
                : (defs.find((d) => d.cell === selectedCell)?.label ?? `cell ${selectedCell}`)}
            </strong>
          </div>
        </div>
      </div>
    </section>
  );
}
