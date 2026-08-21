import React, { useEffect, useRef } from "react";

/*
  OdomMap — LittleSLAM の MapDrawer (drawMapGp) 相当の自己位置マップ描画。
  summer2026_odometry が publish する /odom (nav_msgs/Odometry) から作った
  「累積点群マップ + ロボット軌跡 + 現在姿勢」を Canvas に描くビューア。

  累積・姿勢の相対化(calRelativePose 相当)・LiDAR点のグローバル変換(globalPoint 相当)は
  App.js 側(データ層)で行い、ここは描画のみを担当する。

  座標規約は LittleSLAM/gnuplot に合わせて x=右, y=上。範囲はマップ成長に合わせて自動フィット。

  props:
    pose          : { x, y, yaw }      生odom現在姿勢(原点正規化済み, m/rad)
    path          : [{ x, y }, ...]    生odom軌跡(odom フレーム)
    mapPoints     : [{ x, y }, ...]    累積点群マップ(odom フレーム, 任意)
    corrected     : { x, y, yaw }|null ICP補正後の絶対姿勢(/localization/pose, 任意)
    correctedPath : [{ x, y }, ...]    補正後の軌跡(任意)
    body          : { x, y, yaw }|null 機体現在姿勢(位置=最良自己位置, yaw=IMU方位, 任意)
    tilt          : { roll, pitch }|null IMUの傾き[rad](任意, 石倉登坂の可視化用)
    bodySize      : number             機体フットプリント1辺[m](既定0.9)

  とても読みにくいと思いますが、描画ロジックは単純で、Canvas API の基本的な使い方を踏襲。
*/

const DEFAULT_HALF = 1.0;   // データが無いときの初期表示範囲 ±1m
const MARGIN = 0.5;         // フィット時の余白[m]

export function OdomMap({ pose, path, mapPoints, corrected, correctedPath, body, tilt, bodySize = 0.9 }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width;
    const H = canvas.height;

    const p = pose || { x: 0, y: 0, yaw: 0 };
    const pts = mapPoints || [];
    const trail = path || [];
    const cTrail = correctedPath || [];

    // ── 自動フィット(成長するマップ): 全要素の AABB から正方スケールを決定 ──
    let minX = p.x, maxX = p.x, minY = p.y, maxY = p.y;
    const acc = (x, y) => {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
    };
    for (const q of trail) acc(q.x, q.y);
    for (const q of pts) acc(q.x, q.y);
    for (const q of cTrail) acc(q.x, q.y);
    if (corrected) acc(corrected.x, corrected.y);
    if (body) acc(body.x, body.y);
    acc(0, 0); // 原点(スタート)も常に含める

    let cx0 = (minX + maxX) / 2, cy0 = (minY + maxY) / 2;
    let half = Math.max((maxX - minX) / 2, (maxY - minY) / 2) + MARGIN;
    if (!isFinite(half) || half < DEFAULT_HALF) half = DEFAULT_HALF;
    const scale = Math.min(W, H) / (2 * half); // px/m (アスペクト比 1:1)

    // world(x右,y上) → canvas
    const toC = (x, y) => ({
      cx: W / 2 + (x - cx0) * scale,
      cy: H / 2 - (y - cy0) * scale,
    });

    // 背景
    ctx.fillStyle = "#0d1117";
    ctx.fillRect(0, 0, W, H);

    // グリッド(1m)
    ctx.strokeStyle = "#1c2128";
    ctx.lineWidth = 0.5;
    const gx0 = Math.floor(cx0 - half), gx1 = Math.ceil(cx0 + half);
    const gy0 = Math.floor(cy0 - half), gy1 = Math.ceil(cy0 + half);
    for (let x = gx0; x <= gx1; x++) {
      const a = toC(x, gy0), b = toC(x, gy1);
      ctx.beginPath(); ctx.moveTo(a.cx, a.cy); ctx.lineTo(b.cx, b.cy); ctx.stroke();
    }
    for (let y = gy0; y <= gy1; y++) {
      const a = toC(gx0, y), b = toC(gx1, y);
      ctx.beginPath(); ctx.moveTo(a.cx, a.cy); ctx.lineTo(b.cx, b.cy); ctx.stroke();
    }

    // ── 累積点群マップ(pcmap) ──
    ctx.fillStyle = "rgba(88,166,255,0.5)";
    for (const q of pts) {
      const { cx, cy } = toC(q.x, q.y);
      ctx.fillRect(cx - 1, cy - 1, 2, 2);
    }

    // ── 軌跡(古→新でフェード) ──
    if (trail.length > 1) {
      for (let i = 1; i < trail.length; i++) {
        const a = toC(trail[i - 1].x, trail[i - 1].y);
        const b = toC(trail[i].x, trail[i].y);
        const t = i / trail.length;
        ctx.strokeStyle = `rgba(63,185,80,${0.25 + 0.65 * t})`;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(a.cx, a.cy); ctx.lineTo(b.cx, b.cy); ctx.stroke();
      }
    }

    // ── スタート地点(原点) ──
    const o = toC(0, 0);
    ctx.fillStyle = "#484f58";
    ctx.beginPath(); ctx.arc(o.cx, o.cy, 4, 0, Math.PI * 2); ctx.fill();
    ctx.font = "9px sans-serif"; ctx.textAlign = "center";
    ctx.fillText("START", o.cx, o.cy + 16);

    // ── 補正後の軌跡(ICP, シアン) ──
    if (cTrail.length > 1) {
      ctx.strokeStyle = "#39c5cf"; ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < cTrail.length; i++) {
        const { cx, cy } = toC(cTrail[i].x, cTrail[i].y);
        if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
      }
      ctx.stroke();
    }

    // ── 機体フットプリント(bodySize角・IMU方位・傾きで色分け) ──
    const tiltDeg = tilt
      ? Math.max(Math.abs(tilt.roll), Math.abs(tilt.pitch)) * 180 / Math.PI : 0;
    const tiltCol = tiltDeg < 2 ? "#3fb950" : tiltDeg < 5 ? "#d29922" : "#f85149";
    if (body) {
      const bh = (bodySize || 0.9) / 2;
      const bc = Math.cos(body.yaw), bsn = Math.sin(body.yaw);
      // 機体座標(x前/y左)の四隅: 前左, 前右, 後右, 後左
      const corners = [[bh, bh], [bh, -bh], [-bh, -bh], [-bh, bh]];
      const cp = corners.map(([bx, by]) =>
        toC(body.x + bx * bc - by * bsn, body.y + bx * bsn + by * bc));
      ctx.beginPath();
      cp.forEach((q, i) => { if (i === 0) ctx.moveTo(q.cx, q.cy); else ctx.lineTo(q.cx, q.cy); });
      ctx.closePath();
      ctx.fillStyle = tiltCol + "22"; ctx.strokeStyle = tiltCol; ctx.lineWidth = 2;
      ctx.fill(); ctx.stroke();
      // 前面(前左→前右)を白の太線で示す
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(cp[0].cx, cp[0].cy); ctx.lineTo(cp[1].cx, cp[1].cy); ctx.stroke();
    }

    // ── 現在ロボット: 生odom(橙) ──
    const r = toC(p.x, p.y);
    const head = toC(p.x + 0.4 * Math.cos(p.yaw), p.y + 0.4 * Math.sin(p.yaw));
    ctx.strokeStyle = "#f0883e"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(r.cx, r.cy); ctx.lineTo(head.cx, head.cy); ctx.stroke();
    ctx.fillStyle = "#f0883e"; ctx.strokeStyle = "#ffb86c";
    ctx.beginPath(); ctx.arc(r.cx, r.cy, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();

    // ── 現在ロボット: ICP補正後(シアン) ──
    if (corrected) {
      const cr = toC(corrected.x, corrected.y);
      const ch = toC(corrected.x + 0.4 * Math.cos(corrected.yaw),
                     corrected.y + 0.4 * Math.sin(corrected.yaw));
      ctx.strokeStyle = "#39c5cf"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(cr.cx, cr.cy); ctx.lineTo(ch.cx, ch.cy); ctx.stroke();
      ctx.fillStyle = "#39c5cf"; ctx.strokeStyle = "#a5f3fc";
      ctx.beginPath(); ctx.arc(cr.cx, cr.cy, 6, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }

    // ── オーバーレイ(数値 / スケール) ──
    ctx.fillStyle = "#f0883e"; ctx.font = "10px monospace"; ctx.textAlign = "left";
    ctx.fillText(
      `odom  x=${p.x.toFixed(2)} y=${p.y.toFixed(2)} yaw=${(p.yaw * 180 / Math.PI).toFixed(1)}°`,
      8, 14
    );
    if (corrected) {
      ctx.fillStyle = "#39c5cf";
      ctx.fillText(
        `ICP   x=${corrected.x.toFixed(2)} y=${corrected.y.toFixed(2)} yaw=${(corrected.yaw * 180 / Math.PI).toFixed(1)}°`,
        8, 28
      );
    }
    ctx.fillStyle = "#8b949e";
    ctx.fillText(`trail=${trail.length}  map=${pts.length}  ${(2 * half).toFixed(1)}m幅`, 8, 42);

    // ── IMU 姿勢: 右上にバブルレベル + roll/pitch ──
    if (tilt) {
      const bx = W - 34, by = 34, R = 24;
      ctx.strokeStyle = "#30363d"; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(bx, by, R, 0, Math.PI * 2); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(bx - R, by); ctx.lineTo(bx + R, by);
      ctx.moveTo(bx, by - R); ctx.lineTo(bx, by + R); ctx.stroke();
      // バブル(roll→左右, pitch→上下)。±15°で円周。
      const sc = R / (15 * Math.PI / 180);
      const dx = Math.max(-R, Math.min(R, tilt.roll * sc));
      const dy = Math.max(-R, Math.min(R, -tilt.pitch * sc));
      ctx.fillStyle = tiltCol;
      ctx.beginPath(); ctx.arc(bx + dx, by + dy, 5, 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#6e7681"; ctx.font = "8px monospace"; ctx.textAlign = "center";
      ctx.fillText("傾き", bx, by + R + 9);
      ctx.fillStyle = tiltCol; ctx.font = "10px monospace"; ctx.textAlign = "left";
      ctx.fillText(
        `IMU  r=${(tilt.roll * 180 / Math.PI).toFixed(0)}° p=${(tilt.pitch * 180 / Math.PI).toFixed(0)}°`,
        8, 56
      );
    }
  }, [pose, path, mapPoints, corrected, correctedPath, body, tilt, bodySize]);

  return (
    <canvas ref={canvasRef} width={460} height={340} className="cage-canvas" />
  );
}
