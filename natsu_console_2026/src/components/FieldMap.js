import React, { useEffect, useRef } from "react";

// フィールド俯瞰図(静的)。カゴ・石倉・スタート位置を固定座標で描くだけで、
// ROSトピックは一切購読しない。SHOOTモードのカゴ選択で「どのカゴを狙うか」の
// 位置関係を目で確認するために使う。
//
// 元は CageViz.js で、cage_detection(/cage_detection/cages)の受信結果を
// 重畳表示していた。cage_detection は wall_detection(LiDAR)から自己位置を
// 出すノードで、LiDARを使わなくなったため受信部分ごと src/archive/CageViz.js
// へ退避してある。復活させるならそちらを参照。

// cage_detection.yaml と一致させること
const CAGE_DEFS = [
  { id: "green_0", color: 0, fx: 4.20, fy:  0.55, label: "G0", isFarthest: false },
  { id: "green_1", color: 0, fx: 4.20, fy: -0.55, label: "G1", isFarthest: false },
  { id: "blue_0",  color: 1, fx: 5.40, fy:  1.10, label: "B0", isFarthest: false },
  { id: "blue_1",  color: 1, fx: 5.40, fy: -1.10, label: "B1", isFarthest: true  },
  { id: "blue_2",  color: 1, fx: 4.80, fy:  0.00, label: "B2", isFarthest: false },
  { id: "blue_3",  color: 1, fx: 3.73, fy:  1.10, label: "B3", isFarthest: false },
  { id: "blue_4",  color: 1, fx: 3.73, fy: -1.10, label: "B4", isFarthest: false },
];

const ISHIKURA_X = 3.2;
const ISHIKURA_Y = 0.0;

// selectedId: 選択中のカゴID(BASKET_SELECT_DEFS の id と共通)。省略可。
export function FieldMap({ selectedId }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;

    // ビュー範囲: field_x -0.3 〜 6.5, field_y -2.5 〜 2.5
    const X_MIN = -0.3;
    const X_SPAN = 6.8;   // 6.5 - (-0.3)
    const Y_SPAN = 5.0;   // 2.5 - (-2.5)
    const scale = Math.min(H / X_SPAN, W / Y_SPAN);

    // フィールド座標 → キャンバス座標
    //   field_x (前方↑) → canvas_y (上方向)
    //   field_y (左+)   → canvas_x (左方向)
    const toC = (fx, fy) => ({
      cx: W / 2 - fy * scale,
      cy: H - (fx - X_MIN) * scale,
    });

    // 背景
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, W, H);

    // グリッド（1m 間隔）
    ctx.strokeStyle = "#1c2128";
    ctx.lineWidth = 0.5;
    for (let x = 0; x <= 7; x++) {
      const { cx: x1, cy: y1 } = toC(x, -2.5);
      const { cx: x2, cy: y2 } = toC(x, 2.5);
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    }
    for (let y = -2; y <= 2; y++) {
      const { cx: x1, cy: y1 } = toC(-0.3, y);
      const { cx: x2, cy: y2 } = toC(6.5, y);
      ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    }

    // グリッドラベル
    ctx.fillStyle = "#30363d";
    ctx.font = "9px monospace";
    ctx.textAlign = "center";
    for (let x = 0; x <= 6; x++) {
      const { cx, cy } = toC(x, 2.5);
      ctx.fillText(`${x}`, cx + 11, cy + 3);
    }
    ctx.textAlign = "left";
    ctx.fillStyle = "#30363d";
    ctx.fillText("X[m]→", 4, 11);

    // エリア2 境界（ルールブック図4: 3333.5×3333.5mm）
    const { cx: a1x, cy: a1y } = toC(3.733, -1.667);
    const { cx: a2x, cy: a2y } = toC(7.066,  1.667);
    ctx.strokeStyle = "#1a3a5c";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.strokeRect(a2x, a2y, a1x - a2x, a1y - a2y);
    ctx.setLineDash([]);
    ctx.fillStyle = "#1a3a5c";
    ctx.font = "9px monospace";
    ctx.textAlign = "center";
    const { cx: acx, cy: acy } = toC(5.1, 0);
    ctx.fillText("エリア2", acx, acy);

    // ─── カゴ描画 ──────────────────────────────────
    const CAGE_R = 13;
    for (const def of CAGE_DEFS) {
      const { cx, cy } = toC(def.fx, def.fy);
      const isGreen = def.color === 0;
      const baseColor = isGreen ? "#3fb950" : "#58a6ff";
      const isSelected = selectedId === def.id;

      // 選択中のカゴを光らせる
      if (isSelected) {
        const grd = ctx.createRadialGradient(cx, cy, CAGE_R, cx, cy, CAGE_R + 16);
        grd.addColorStop(0, isGreen ? "rgba(63,185,80,0.55)" : "rgba(88,166,255,0.55)");
        grd.addColorStop(1, "transparent");
        ctx.fillStyle = grd;
        ctx.beginPath();
        ctx.arc(cx, cy, CAGE_R + 16, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#f2cc60";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(cx, cy, CAGE_R + 7, 0, Math.PI * 2);
        ctx.stroke();
      }

      // 本体
      ctx.beginPath();
      ctx.arc(cx, cy, CAGE_R, 0, Math.PI * 2);
      if (isSelected) {
        ctx.fillStyle = isGreen ? "rgba(63,185,80,0.75)" : "rgba(88,166,255,0.65)";
        ctx.strokeStyle = baseColor;
        ctx.lineWidth = 2;
      } else {
        ctx.fillStyle = isGreen ? "rgba(63,185,80,0.32)" : "rgba(88,166,255,0.28)";
        ctx.strokeStyle = baseColor;
        ctx.lineWidth = 1.5;
      }
      ctx.fill();
      ctx.stroke();

      // ラベル
      const dispLabel = def.isFarthest ? `${def.label}★` : def.label;
      ctx.fillStyle = isSelected ? "#ffffff" : "#c9d1d9";
      ctx.font = "bold 9px monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(dispLabel, cx, cy);
      ctx.textBaseline = "alphabetic";
    }

    // 石倉
    const { cx: ix, cy: iy } = toC(ISHIKURA_X, ISHIKURA_Y);
    ctx.fillStyle = "#21262d";
    ctx.strokeStyle = "#484f58";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.roundRect(ix - 18, iy - 10, 36, 20, 4);
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#8b949e";
    ctx.font = "bold 9px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText("石倉", ix, iy);
    ctx.textBaseline = "alphabetic";

    // ロボット（スタート位置 = 原点）
    const { cx: rx, cy: ry } = toC(0, 0);
    ctx.fillStyle = "#c9d1d9";
    ctx.strokeStyle = "#58a6ff";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(rx, ry - 11);
    ctx.lineTo(rx + 8, ry + 8);
    ctx.lineTo(rx - 8, ry + 8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = "#484f58";
    ctx.font = "9px sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("START", rx, ry + 20);

    // 前方方向線
    ctx.strokeStyle = "#1c2128";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 6]);
    const { cx: fwdX, cy: fwdY } = toC(6.5, 0);
    ctx.beginPath();
    ctx.moveTo(rx, ry);
    ctx.lineTo(fwdX, fwdY);
    ctx.stroke();
    ctx.setLineDash([]);

    // 凡例
    const ly = H - 16;
    ctx.fillStyle = "#3fb950";
    ctx.beginPath(); ctx.arc(10, ly, 5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#6e7681";
    ctx.font = "9px sans-serif";
    ctx.textAlign = "left";
    ctx.fillText("緑カゴ", 19, ly + 4);
    ctx.fillStyle = "#58a6ff";
    ctx.beginPath(); ctx.arc(68, ly, 5, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#6e7681";
    ctx.fillText("青 (★=最遠+15pt)", 77, ly + 4);
  }, [selectedId]);

  return (
    <canvas
      ref={canvasRef}
      width={460}
      height={340}
      className="cage-canvas"
    />
  );
}
