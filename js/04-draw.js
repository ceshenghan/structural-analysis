/* ============================================================
   Beam & Frame Analysis – 04-draw.js
   Canvas rendering.
     • view transform (pan / zoom / fit), HiDPI canvas preparation
     • grid, members, nodes, hinges / releases, member & node labels
     • support symbols derived from supportRows() (pin, roller, fixed,
       guided, rotational), oriented away from the connected members
     • loads per load column: nodal forces / moments, member point
       loads, distributed (trapezoidal) loads, labels "P", "2q", "5"…
     • N / V / M diagrams (M drawn on the tension side = local −y),
       deformed shape, reactions, extrema labels, hover probe
     • hit testing for 05-ui (pickAt), grid snapping, rubber band
     • result formatting: fmtResNum / fmtResLin (symbols, L powers, /EI)
   Sign conventions (from 02/03): y up, moments positive counter-clockwise,
   N tension +, V positive drawn on local +y, M positive drawn on local −y.
   Depends on: 01-core.js, 02-model.js, 03-solver.js
   ============================================================ */
'use strict';

const DRAW = {
  bg: '#ffffff', grid: '#eef1f5', gridMajor: '#dbe1e9', axis: '#c3ccd8',
  member: '#333a45', memberW: 3, sel: '#1f6feb', hover: '#58a6ff',
  node: '#222', support: '#555', label: '#5b6472',
  load: '#d64541', react: '#1a9c6b', probe: '#8250df', def: COL.def,
  /* diagram colours = COL (01-core) so that they match the legend (.c-N/.c-V/.c-M/.c-def in style.css) */
  M: { stroke: COL.M, fill: 'rgba(214,40,31,0.16)' },
  V: { stroke: COL.V, fill: 'rgba(31,102,214,0.16)' },
  N: { stroke: COL.N, fill: 'rgba(46,139,87,0.16)' },
  font: '12px system-ui, "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif',
  fontSmall: '11px system-ui, "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif',
  diagPx: 60,      // pixel height of the largest diagram ordinate (× S.show.scale)
  defPx: 40,       // pixel size of the largest displacement (× S.show.defScale)
  forcePx: 46,     // arrow length of the largest point force
  qPx: 28,         // height of the largest distributed load
  momR: 14,        // radius of moment arcs
  reactPx: 40      // extra arrow length of the largest reaction
};

/* ---------------------------------------------------------------
   1. View state and transform
      S.view = { ox, oy, k, fitted }     screen = (ox + k·x, oy − k·y)
      S.show = display switches (05-ui toggles them)
   --------------------------------------------------------------- */
function viewInit() {
  if (!S.view || !(S.view.k > 0)) S.view = { ox: 0, oy: 0, k: 60, fitted: false };
  const def = {
    grid: true, nodeLabels: true, memberLabels: true, loads: true, supports: true,
    reactions: false, diagrams: ['M'], deformed: false, values: true, scale: 1, defScale: 1, mAbs: false, snap: true
  };
  S.show = Object.assign(def, S.show || {});
}
function w2s(x, y) { const v = S.view; return [v.ox + v.k * x, v.oy - v.k * y]; }
function s2w(px, py) { const v = S.view; return [(px - v.ox) / v.k, (v.oy - py) / v.k]; }

function viewFit(W, H, pad) {
  viewInit();
  pad = pad === undefined ? 100 : pad;
  const v = S.view, nds = S.model.nodes;
  if (!nds.length) { v.k = 60; v.ox = W / 2; v.oy = H / 2; v.fitted = true; return; }
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  nds.forEach(nd => { x0 = Math.min(x0, nd.x); x1 = Math.max(x1, nd.x); y0 = Math.min(y0, nd.y); y1 = Math.max(y1, nd.y); });
  const w = x1 - x0, h = y1 - y0;
  if (w < 1e-9 && h < 1e-9) v.k = 60;
  else v.k = Math.min((W - 2 * pad) / Math.max(w, 1e-9), (H - 2 * pad) / Math.max(h, 1e-9));
  if (!isFinite(v.k) || v.k <= 0) v.k = 60;
  v.k = clamp(v.k, 2, 5000);
  v.ox = W / 2 - v.k * (x0 + x1) / 2;
  v.oy = H / 2 + v.k * (y0 + y1) / 2;
  v.fitted = true;
}
function viewZoom(factor, px, py) {
  const v = S.view, [wx, wy] = s2w(px, py);
  v.k = clamp(v.k * factor, 2, 5000);
  v.ox = px - v.k * wx; v.oy = py + v.k * wy;
}
function viewPan(dx, dy) { S.view.ox += dx; S.view.oy += dy; }

/* "nice" grid step (1, 2, 5 × 10ⁿ) giving ≥ ~48 px spacing */
function gridStep() {
  const target = 48 / S.view.k;
  const p = Math.pow(10, Math.floor(Math.log10(target)));
  const m = target / p;
  return (m < 1.5 ? 1 : m < 3.5 ? 2 : m < 7.5 ? 5 : 10) * p;
}
function snapToGrid(x, y) {
  const g = gridStep() / 2;
  return [Math.round(x / g) * g, Math.round(y / g) * g];
}

/* HiDPI-aware canvas; returns a context whose unit is the CSS pixel */
function prepCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(1, canvas.clientWidth || canvas.width), H = Math.max(1, canvas.clientHeight || canvas.height);
  const pw = Math.round(W * dpr), ph = Math.round(H * dpr);
  if (canvas.width !== pw || canvas.height !== ph) { canvas.width = pw; canvas.height = ph; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, W, H };
}

/* ---------------------------------------------------------------
   2. Formatting of results (used by 04 labels and 05 tables)
      kind: 'F' force, 'M' moment, 'u' deflection (∝1/EI), 'th' rotation (∝1/EI)
   --------------------------------------------------------------- */
const _SUP = { 2: '²', 3: '³', 4: '⁴', 5: '⁵' };
function _dwLpow(p) {
  if (!p) return '';
  const a = Math.abs(p), s = 'L' + (a === 1 ? '' : (_SUP[a] || '^' + a));
  return p > 0 ? s : '/' + s;
}
function fmtResNum(v, digits) {
  if (v === null || v === undefined || !isFinite(v)) return '—';
  digits = digits || (S.model.settings && S.model.settings.digits) || 4;
  const a = Math.abs(v);
  if (a < 1e-12) return '0';
  const s = (a >= 1e6 || a < 1e-4)
    ? a.toExponential(Math.max(1, digits - 1)).replace(/\.?0+e/, 'e').replace('e+', 'e')
    : String(+a.toPrecision(digits));
  return (v < 0 ? '−' : '') + s;
}
/* result quantity as text. kind: 'F' force, 'M' moment, 'u' deflection, 'th' rotation.
   Delegates to fmtQty (01-core) so that fractions, powers of L and “/EI” follow the model settings. */
const _DW_KIND = { F: 'force', M: 'moment', u: 'defl', th: 'rot' };
function fmtResLin(lin, kind) {
  return fmtQty(Lin.from(lin), _DW_KIND[kind] || 'force', S.model.settings || {});
}
/* kind of each load symbol, inferred from where it is used: q (per length) > F (force) > M (moment) */
function symbolKinds(model, geo) {
  const rank = { q: 3, F: 2, M: 1 };
  const out = {};
  geo = geo || _dwGeometry(model, S.result && S.result.ok ? S.result : null);
  loadColumns(model).forEach(key => {
    if (key === CONST_KEY) return;
    let kind = null;
    const put = k => { if (!kind || rank[k] > rank[kind]) kind = k; };
    model.nodes.forEach(nd => {
      const f = nodeLoadVector(nd, key) || [0, 0, 0];
      if (f[0] || f[1]) put('F');
      if (f[2]) put('M');
    });
    for (const id in geo) {
      const sys = _dwSys(model, geo[id]);
      if (!sys) continue;
      (memberLocalLoads(geo[id].m, sys, key) || []).forEach(ld => put(ld.kind === 'q' ? 'q' : ld.kind === 'F' ? 'F' : 'M'));
    }
    out[key] = kind || 'F';
  });
  return out;
}

/* ---------------------------------------------------------------
   3. Geometry cache and primitives
   --------------------------------------------------------------- */
function _dwGeometry(model, res) {
  const g = {};
  model.members.forEach(m => {
    const A = nodeById(model, m.a), B = nodeById(model, m.b);
    if (!A || !B) return;
    const dx = B.x - A.x, dy = B.y - A.y, L = Math.hypot(dx, dy) || 1e-300;
    g[m.id] = { m, A, B, L, c: dx / L, s: dy / L, sys: res && res.members[m.id] ? res.members[m.id].sys : null };
  });
  return g;
}
function _dwSys(model, g) {
  if (g.sys !== null && g.sys !== undefined) return g.sys || null;
  try { g.sys = memberSystem(model, g.m); } catch (e) { g.sys = false; }
  return g.sys || null;
}
function _dwPt(g, x) { return w2s(g.A.x + g.c * x, g.A.y + g.s * x); }   // screen point at distance x from a
function _dwIs(sel, type, id) { return !!sel && sel.type === type && String(sel.id) === String(id); }

function _dwText(ctx, text, x, y, o) {
  o = o || {};
  ctx.font = o.font || DRAW.font;
  ctx.textAlign = o.align || 'center';
  ctx.textBaseline = o.baseline || 'middle';
  ctx.lineJoin = 'round';
  ctx.lineWidth = 3;
  ctx.strokeStyle = o.halo || 'rgba(255,255,255,0.88)';
  ctx.strokeText(text, x, y);
  ctx.fillStyle = o.color || '#222';
  ctx.fillText(text, x, y);
}
function _dwHead(ctx, x, y, ux, uy, size) {
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x - ux * size - uy * size * 0.42, y - uy * size + ux * size * 0.42);
  ctx.lineTo(x - ux * size + uy * size * 0.42, y - uy * size - ux * size * 0.42);
  ctx.closePath();
  ctx.fill();
}
function _dwArrow(ctx, x0, y0, x1, y1, color, head) {
  head = head || 9;
  const dx = x1 - x0, dy = y1 - y0, L = Math.hypot(dx, dy);
  if (L < 1e-6) return;
  const ux = dx / L, uy = dy / L;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.6;
  ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1 - ux * head * 0.6, y1 - uy * head * 0.6); ctx.stroke();
  _dwHead(ctx, x1, y1, ux, uy, head);
}
/* moment arc; ccwWorld = positive moment, drawn counter-clockwise as seen on screen.
   Canvas angles grow clockwise on screen (y down), so a CCW arrow runs towards decreasing angle. */
function _dwMoment(ctx, cx, cy, r, ccwWorld, color) {
  const a0 = 0.75 * Math.PI;                                  // start lower-left, open gap at the bottom
  const a1 = ccwWorld ? a0 - 1.5 * Math.PI : a0 + 1.5 * Math.PI;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 1.6;
  ctx.beginPath(); ctx.arc(cx, cy, r, a0, a1, ccwWorld); ctx.stroke();
  const ex = cx + r * Math.cos(a1), ey = cy + r * Math.sin(a1);
  const tx = ccwWorld ? Math.sin(a1) : -Math.sin(a1), ty = ccwWorld ? -Math.cos(a1) : Math.cos(a1);
  _dwHead(ctx, ex, ey, tx, ty, 8);
}
function _dwRoundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.lineTo(x + w - r, y); ctx.quadraticCurveTo(x + w, y, x + w, y + r);
  ctx.lineTo(x + w, y + h - r); ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
  ctx.lineTo(x + r, y + h); ctx.quadraticCurveTo(x, y + h, x, y + h - r);
  ctx.lineTo(x, y + r); ctx.quadraticCurveTo(x, y, x + r, y); ctx.closePath();
}

/* ---------------------------------------------------------------
   4. Grid, members, nodes
   --------------------------------------------------------------- */
function _dwGrid(ctx, W, H) {
  const g = gridStep();
  const [wx0, wyTop] = s2w(0, 0), [wx1, wyBot] = s2w(W, H);
  const i0 = Math.floor(wx0 / g), i1 = Math.ceil(wx1 / g), j0 = Math.floor(wyBot / g), j1 = Math.ceil(wyTop / g);
  ctx.lineWidth = 1;
  if (i1 - i0 < 400 && j1 - j0 < 400) {
    for (let i = i0; i <= i1; i++) {
      const x = Math.round(w2s(i * g, 0)[0]) + 0.5;
      ctx.strokeStyle = i % 5 === 0 ? DRAW.gridMajor : DRAW.grid;
      ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
    }
    for (let j = j0; j <= j1; j++) {
      const y = Math.round(w2s(0, j * g)[1]) + 0.5;
      ctx.strokeStyle = j % 5 === 0 ? DRAW.gridMajor : DRAW.grid;
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
    }
  }
  const [ox, oy] = w2s(0, 0);
  ctx.strokeStyle = DRAW.axis;
  ctx.beginPath(); ctx.moveTo(0, Math.round(oy) + 0.5); ctx.lineTo(W, Math.round(oy) + 0.5); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(Math.round(ox) + 0.5, 0); ctx.lineTo(Math.round(ox) + 0.5, H); ctx.stroke();
  const symL = !!(S.model.settings && S.model.settings.symL);
  const symMode = S.model.settings && S.model.settings.mode === 'sym';
  _dwText(ctx, `grid ${fmtResNum(g)}${symL && symMode ? ' L' : ' m'}`, W - 10, H - 26, { align: 'right', color: '#8a93a2', font: DRAW.fontSmall });
}

function _dwMembers(ctx, model, geo) {
  ctx.lineCap = 'round';
  for (const id in geo) {
    const g = geo[id];
    const [ax, ay] = w2s(g.A.x, g.A.y), [bx, by] = w2s(g.B.x, g.B.y);
    const hl = _dwIs(S.sel, 'member', id) ? DRAW.sel : _dwIs(S.hover, 'member', id) ? DRAW.hover : null;
    if (hl) {
      ctx.strokeStyle = hl; ctx.lineWidth = DRAW.memberW + 6; ctx.globalAlpha = 0.35;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.globalAlpha = 1;
    }
    ctx.strokeStyle = DRAW.member; ctx.lineWidth = DRAW.memberW;
    ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
    /* end releases (moment hinges at member ends) */
    const ux = (bx - ax), uy = (by - ay), Ls = Math.hypot(ux, uy) || 1;
    const ex = ux / Ls, ey = uy / Ls;
    const rel = (x, y) => {
      ctx.fillStyle = '#fff'; ctx.strokeStyle = DRAW.node; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    };
    const shr = (x, y) => {                                     // shear release: two bars across the member
      ctx.strokeStyle = DRAW.node; ctx.lineWidth = 2;
      for (const d of [-2.5, 2.5]) {
        ctx.beginPath(); ctx.moveTo(x + ex * d - ey * 7, y + ey * d + ex * 7); ctx.lineTo(x + ex * d + ey * 7, y + ey * d - ex * 7); ctx.stroke();
      }
    };
    const axl = (x, y) => {                                     // axial release: sleeve along the member
      ctx.strokeStyle = DRAW.node; ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(x - ex * 7 - ey * 5, y - ey * 7 + ex * 5); ctx.lineTo(x + ex * 7 - ey * 5, y + ey * 7 + ex * 5);
      ctx.moveTo(x - ex * 7 + ey * 5, y - ey * 7 - ex * 5); ctx.lineTo(x + ex * 7 + ey * 5, y + ey * 7 - ex * 5);
      ctx.moveTo(x - ex * 7 - ey * 5, y - ey * 7 + ex * 5); ctx.lineTo(x - ex * 7 + ey * 5, y - ey * 7 - ex * 5);
      ctx.stroke();
    };
    if (g.m.relA) rel(ax + ex * 10, ay + ey * 10);
    if (g.m.relB) rel(bx - ex * 10, by - ey * 10);
    if (g.m.shrA) shr(ax + ex * 20, ay + ey * 20);
    if (g.m.shrB) shr(bx - ex * 20, by - ey * 20);
    if (g.m.axA) axl(ax + ex * 30, ay + ey * 30);
    if (g.m.axB) axl(bx - ex * 30, by - ey * 30);
    if (S.show.memberLabels) {
      const exs = -g.s, eys = -g.c;                           // screen direction of local +y
      _dwText(ctx, String(id), (ax + bx) / 2 + exs * 12, (ay + by) / 2 + eys * 12, { color: DRAW.label, font: DRAW.fontSmall });
    }
  }
}

function _dwNodes(ctx, model) {
  model.nodes.forEach(nd => {
    const [sx, sy] = w2s(nd.x, nd.y);
    const hl = _dwIs(S.sel, 'node', nd.id) ? DRAW.sel : _dwIs(S.hover, 'node', nd.id) ? DRAW.hover : null;
    if (hl) {
      ctx.fillStyle = hl; ctx.globalAlpha = 0.35;
      ctx.beginPath(); ctx.arc(sx, sy, 10, 0, 2 * Math.PI); ctx.fill();
      ctx.globalAlpha = 1;
    }
    if (nd.hinge) {
      ctx.fillStyle = '#fff'; ctx.strokeStyle = DRAW.node; ctx.lineWidth = 1.6;
      ctx.beginPath(); ctx.arc(sx, sy, 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    } else {
      ctx.fillStyle = DRAW.node;
      ctx.beginPath(); ctx.arc(sx, sy, 3.2, 0, 2 * Math.PI); ctx.fill();
    }
    if (S.show.nodeLabels) _dwText(ctx, String(nd.id), sx + 7, sy - 9, { align: 'left', color: '#333', font: DRAW.fontSmall });
  });
}

/* ---------------------------------------------------------------
   5. Supports (symbol derived from the constraint rows)
   --------------------------------------------------------------- */
/* unit vector pointing away from the members connected to the node (world) */
function _dwAway(model, nd) {
  let sx = 0, sy = 0;
  nodeMembers(model, nd.id).forEach(m => {
    const other = nodeById(model, m.a === nd.id ? m.b : m.a);
    if (!other) return;
    const dx = other.x - nd.x, dy = other.y - nd.y, L = Math.hypot(dx, dy) || 1;
    sx += dx / L; sy += dy / L;
  });
  const n = Math.hypot(sx, sy);
  return n < 1e-6 ? [0, -1] : [-sx / n, -sy / n];
}
function _dwSnapDir(v) {
  if (Math.abs(v[0]) >= Math.abs(v[1])) return [v[0] >= 0 ? 1 : -1, 0];
  return [0, v[1] >= 0 ? 1 : -1];
}
/* local frame: origin at (sx,sy), local +y points along world direction n (into the support) */
function _dwFrame(ctx, sx, sy, nx, ny, fn) {
  const nsx = nx, nsy = -ny;
  const th = Math.atan2(-nsx, nsy);
  ctx.save(); ctx.translate(sx, sy); ctx.rotate(th); fn(); ctx.restore();
}
function _dwHatch(ctx, x0, x1, y, d) {
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
  ctx.lineWidth = 1;
  for (let x = x0 + d; x <= x1 + 0.01; x += 5) { ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x - d, y + d); ctx.stroke(); }
}
function _dwSupports(ctx, model) {
  model.nodes.forEach(nd => {
    const rows = supportRows(nd) || [];
    if (!rows.length) return;
    const fd = rows.filter(r => r.dir).map(r => { const L = Math.hypot(r.dir[0], r.dir[1]) || 1; return [r.dir[0] / L, r.dir[1] / L]; });
    const hasM = rows.some(r => !r.dir);
    const [sx, sy] = w2s(nd.x, nd.y);
    /* direction pointing into the support (world): ground below for ang = 0, wall on the right for 90° */
    const a = deg2rad(nd.ang || 0);
    const n = [Math.sin(a), -Math.cos(a)];
    ctx.strokeStyle = DRAW.support; ctx.fillStyle = '#fff'; ctx.lineWidth = 1.5; ctx.lineCap = 'butt';
    const tri = h => { ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(-10, h); ctx.lineTo(10, h); ctx.closePath(); ctx.fill(); ctx.stroke(); };
    const circ = (x, y, r) => { ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); };
    _dwFrame(ctx, sx, sy, n[0], n[1], () => {
      if (fd.length >= 2 && hasM) {                           // fixed
        ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(-15, 0); ctx.lineTo(15, 0); ctx.stroke();
        ctx.lineWidth = 1;
        for (let x = -9; x <= 15; x += 6) { ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x - 6, 7); ctx.stroke(); }
      } else if (fd.length >= 2) {                            // pin
        tri(16); _dwHatch(ctx, -15, 15, 17, 6);
      } else if (fd.length === 1 && !hasM) {                  // roller (translation free ⟂ dir)
        tri(14); circ(-6, 18, 3.3); circ(6, 18, 3.3); _dwHatch(ctx, -15, 15, 22, 6);
      } else if (fd.length === 1 && hasM) {                   // guided support (slider)
        ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(-12, 0); ctx.lineTo(12, 0); ctx.stroke();
        ctx.lineWidth = 1.5; circ(-6, 4.5, 3.3); circ(6, 4.5, 3.3); _dwHatch(ctx, -15, 15, 9, 6);
      } else {                                                 // rotation only
        ctx.beginPath(); ctx.rect(-7, -7, 14, 14); ctx.fill(); ctx.stroke();
        ctx.lineWidth = 1;
        for (let x = -7; x < 7; x += 4) { ctx.beginPath(); ctx.moveTo(x, 7); ctx.lineTo(x + 4, -7); ctx.stroke(); }
      }
    });
  });
}

/* ---------------------------------------------------------------
   6. Loads (one arrow set per load column; symbolic labels)
   --------------------------------------------------------------- */
function _dwLoadText(coef, key) {
  const a = Math.abs(coef), frac = !!(S.model.settings && S.model.settings.frac);
  if (key === CONST_KEY) return fmtNum(a, frac);
  if (Math.abs(a - 1) < 1e-12) return key;
  const c = fmtNum(a, frac);
  return c + (/\//.test(c) ? ' ' : '') + key;
}
function _dwForceArrow(ctx, sx, sy, fx, fy, fsc, text) {
  /* (fx,fy) world force; arrow tip at (sx,sy) */
  const mag = Math.hypot(fx, fy);
  if (!mag) return;
  const ux = fx / mag, uy = -fy / mag;                        // screen direction of the force
  const len = Math.max(16, mag * fsc);
  _dwArrow(ctx, sx - ux * len, sy - uy * len, sx, sy, DRAW.load, 9);
  if (text) _dwText(ctx, text, sx - ux * (len + 10), sy - uy * (len + 10), { color: DRAW.load });
}
function _dwDistLoad(ctx, g, ld, wk, key, qsc) {
  const x1 = ld.x1, x2 = ld.x2;
  if (!(x2 > x1)) return;
  const exs = -g.s, eys = -g.c;                                // screen direction of local +y
  const color = DRAW.load;
  const py1 = ld.py1 * wk, py2 = ld.py2 * wk;
  if (py1 || py2) {
    const n = Math.max(1, Math.round((x2 - x1) * S.view.k / 14));
    const tails = [];
    for (let i = 0; i <= n; i++) {
      const t = i / n, x = x1 + t * (x2 - x1), py = py1 + t * (py2 - py1);
      const [tx, ty] = _dwPt(g, x);
      const sg = py >= 0 ? 1 : -1, h = Math.abs(py) * qsc;
      const ux = sg * exs, uy = sg * eys;                      // force direction on screen
      const tail = [tx - ux * h, ty - uy * h];
      tails.push(tail);
      if (h > 4) _dwArrow(ctx, tail[0], tail[1], tx, ty, color, 6);
      else if (h > 0) { ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(tail[0], tail[1]); ctx.lineTo(tx, ty); ctx.stroke(); }
    }
    ctx.strokeStyle = color; ctx.lineWidth = 1.4;
    ctx.beginPath(); tails.forEach((p, i) => i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])); ctx.stroke();
    const lab = (x, py, coef) => {
      const [tx, ty] = _dwPt(g, x);
      const sg = py >= 0 ? 1 : -1, h = Math.abs(py) * qsc;
      _dwText(ctx, _dwLoadText(coef, key), tx - sg * exs * (h + 10), ty - sg * eys * (h + 10), { color });
    };
    if (Math.abs(ld.py1 - ld.py2) <= 1e-12 * (Math.abs(ld.py1) + Math.abs(ld.py2))) lab((x1 + x2) / 2, py1, ld.py1);
    else { if (ld.py1) lab(x1, py1, ld.py1); if (ld.py2) lab(x2, py2, ld.py2); }
  }
  const px1 = ld.px1 * wk, px2 = ld.px2 * wk;
  if (px1 || px2) {                                            // axial distributed load: arrows along the axis
    const n = Math.max(1, Math.round((x2 - x1) * S.view.k / 22));
    for (let i = 0; i < n; i++) {
      const t = (i + 0.5) / n, x = x1 + t * (x2 - x1), px = px1 + t * (px2 - px1);
      if (!px) continue;
      const [cx, cy] = _dwPt(g, x);
      const sg = px >= 0 ? 1 : -1, len = Math.min(12, Math.abs(px) * qsc + 6);
      const ox = exs * 7, oy = eys * 7;
      _dwArrow(ctx, cx + ox - sg * g.c * len / 2, cy + oy + sg * g.s * len / 2, cx + ox + sg * g.c * len / 2, cy + oy - sg * g.s * len / 2, color, 6);
    }
    const [mx, my] = _dwPt(g, (x1 + x2) / 2);
    _dwText(ctx, _dwLoadText(ld.px1 || ld.px2, key), mx + exs * 20, my + eys * 20, { color });
  }
}
function _dwLoads(ctx, model, geo, cols, w) {
  const nodal = [], memb = [];
  let Fmax = 0, qmax = 0;
  cols.forEach((key, ci) => {
    const wk = w[ci] || 1;
    model.nodes.forEach(nd => {
      const f = nodeLoadVector(nd, key) || [0, 0, 0];
      if (f[0] || f[1]) { nodal.push({ nd, fx: f[0] * wk, fy: f[1] * wk, coef: Math.hypot(f[0], f[1]), key }); Fmax = Math.max(Fmax, Math.hypot(f[0], f[1]) * Math.abs(wk)); }
      if (f[2]) nodal.push({ nd, mz: f[2] * wk, coef: f[2], key });
    });
    for (const id in geo) {
      const g = geo[id], sys = _dwSys(model, g);
      if (!sys) continue;
      (memberLocalLoads(g.m, sys, key) || []).forEach(ld => {
        if (ld.kind === 'q') qmax = Math.max(qmax, Math.abs(wk) * Math.max(Math.abs(ld.py1), Math.abs(ld.py2), Math.abs(ld.px1), Math.abs(ld.px2)));
        else if (ld.kind === 'F') Fmax = Math.max(Fmax, Math.abs(wk) * Math.hypot(ld.px, ld.py));
        memb.push({ g, ld, wk, key });
      });
    }
  });
  const fsc = Fmax ? DRAW.forcePx / Fmax : 0, qsc = qmax ? DRAW.qPx / qmax : 0;
  /* distributed first (behind), then point loads, then moments */
  memb.filter(o => o.ld.kind === 'q').forEach(o => _dwDistLoad(ctx, o.g, o.ld, o.wk, o.key, qsc));
  memb.filter(o => o.ld.kind === 'F').forEach(o => {
    const { g, ld, wk, key } = o;
    const px = ld.px * wk, py = ld.py * wk;
    const fx = g.c * px - g.s * py, fy = g.s * px + g.c * py;   // local → global
    const [sx, sy] = _dwPt(g, ld.x);
    _dwForceArrow(ctx, sx, sy, fx, fy, fsc, _dwLoadText(Math.hypot(ld.px, ld.py), key));
  });
  nodal.filter(o => o.fx !== undefined).forEach(o => {
    const [sx, sy] = w2s(o.nd.x, o.nd.y);
    _dwForceArrow(ctx, sx, sy, o.fx, o.fy, fsc, _dwLoadText(o.coef, o.key));
  });
  memb.filter(o => o.ld.kind !== 'q' && o.ld.kind !== 'F').forEach(o => {
    const m = o.ld.m * o.wk;
    if (!m) return;
    const [sx, sy] = _dwPt(o.g, o.ld.x);
    _dwMoment(ctx, sx, sy, DRAW.momR, m > 0, DRAW.load);
    _dwText(ctx, _dwLoadText(o.ld.m, o.key), sx + DRAW.momR + 6, sy - DRAW.momR, { align: 'left', color: DRAW.load });
  });
  nodal.filter(o => o.mz !== undefined).forEach(o => {
    const [sx, sy] = w2s(o.nd.x, o.nd.y);
    _dwMoment(ctx, sx, sy, DRAW.momR, o.mz > 0, DRAW.load);
    _dwText(ctx, _dwLoadText(o.coef, o.key), sx + DRAW.momR + 6, sy - DRAW.momR, { align: 'left', color: DRAW.load });
  });
}

/* ---------------------------------------------------------------
   7. Results: diagrams, deformed shape, reactions, probe
   --------------------------------------------------------------- */
function _dwDiagram(ctx, res, geo, key) {
  const col = DRAW[key], show = S.show;
  let ref = 0;
  for (const id in res.members) { const a = res.members[id].num[key]; for (let i = 0; i < a.length; i++) ref = Math.max(ref, Math.abs(a[i])); }
  if (!ref) return;
  const sc = DRAW.diagPx * (show.scale || 1) / ref;
  const sign = key === 'M' ? -1 : 1;                           // M on the tension side (local −y)
  const kind = key === 'M' ? 'M' : 'F';
  const labels = [];
  for (const id in res.members) {
    const mr = res.members[id], g = geo[id];
    if (!g) continue;
    const vals = mr.num[key], xs = mr.xs;
    const exs = -g.s, eys = -g.c;
    const [ax, ay] = w2s(g.A.x, g.A.y), [bx, by] = w2s(g.B.x, g.B.y);
    const pts = [];
    for (let i = 0; i < xs.length; i++) {
      const [px, py] = _dwPt(g, xs[i]);
      const o = sign * vals[i] * sc;
      pts.push([px + exs * o, py + eys * o]);
    }
    ctx.beginPath(); ctx.moveTo(ax, ay); pts.forEach(p => ctx.lineTo(p[0], p[1])); ctx.lineTo(bx, by); ctx.closePath();
    ctx.fillStyle = col.fill; ctx.fill();
    ctx.strokeStyle = col.stroke; ctx.lineWidth = 1.5; ctx.lineJoin = 'round';
    ctx.beginPath(); ctx.moveTo(ax, ay); pts.forEach(p => ctx.lineTo(p[0], p[1])); ctx.lineTo(bx, by); ctx.stroke();
    if (show.values && mr.ext) {
      const e = mr.ext[key], endLin = mr[key + 'end'];
      const cands = [e.max, e.min,
        { x: 0, val: vals[0], lin: endLin[0] },
        { x: mr.L, val: vals[vals.length - 1], lin: endLin[1] }];
      cands.forEach(c => {
        if (!c || Math.abs(c.val) <= 1e-9 * ref) return;
        if (labels.some(l => l.id === id && Math.abs(l.x - c.x) <= 0.02 * mr.L && Math.abs(l.val - c.val) <= 1e-6 * ref)) return;
        const lin = (key === 'M' && show.mAbs && c.val < 0) ? Lin.from(c.lin).neg() : c.lin;
        labels.push({ id, g, x: c.x, val: c.val, lin });
      });
    }
  }
  labels.forEach(l => {
    const [px, py] = _dwPt(l.g, l.x), exs = -l.g.s, eys = -l.g.c;
    const o = sign * l.val * sc, d = o >= 0 ? 1 : -1;
    _dwText(ctx, fmtResLin(l.lin, kind, res.symKinds), px + exs * (o + d * 11), py + eys * (o + d * 11), { color: col.stroke, font: DRAW.fontSmall });
  });
}

function _dwDeformed(ctx, res, geo) {
  let ref = 0;
  for (const id in res.members) {
    const n = res.members[id].num;
    for (let i = 0; i < n.v.length; i++) ref = Math.max(ref, Math.abs(n.v[i]), Math.abs(n.u[i]));
  }
  if (!ref) return;
  const sc = DRAW.defPx * (S.show.defScale || 1) / ref;
  for (const id in res.members) {
    const mr = res.members[id], g = geo[id];
    if (!g) continue;
    const { u, v } = mr.num, xs = mr.xs;
    /* style set per member: the value label of the previous member (_dwText) changes stroke colour and width */
    ctx.strokeStyle = DRAW.def; ctx.lineWidth = 2; ctx.lineJoin = 'round';
    ctx.beginPath();
    for (let i = 0; i < xs.length; i++) {
      const gx = g.c * u[i] - g.s * v[i], gy = g.s * u[i] + g.c * v[i];
      const [px, py] = _dwPt(g, xs[i]);
      const X = px + gx * sc, Y = py - gy * sc;
      i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
    }
    ctx.stroke();
    if (S.show.values && mr.ext && mr.ext.v.max && Math.abs(mr.ext.v.max.val) > 1e-9 * ref) {
      const e = mr.ext.v.max, i = xs.findIndex(x => Math.abs(x - e.x) <= mr.eps);
      const k = i >= 0 ? i : 0;
      const gx = g.c * u[k] - g.s * v[k], gy = g.s * u[k] + g.c * v[k];
      const [px, py] = _dwPt(g, e.x);
      const sg = (e.val >= 0 ? 1 : -1);
      _dwText(ctx, 'v = ' + fmtResLin(e.lin, 'u', res.symKinds), px + gx * sc - g.s * sg * 14, py - gy * sc - g.c * sg * 14, { color: DRAW.def, font: DRAW.fontSmall });
    }
  }
}

function _dwReactions(ctx, res, model) {
  let Fmax = 0;
  res.reactions.forEach(r => { if (r.dir) Fmax = Math.max(Fmax, Math.abs(r.num)); });
  res.reactions.forEach(r => {
    const nd = nodeById(model, r.nodeId);
    if (!nd || Math.abs(r.num) <= 1e-9 * Math.max(Fmax, res.refs.F)) return;
    const [sx, sy] = w2s(nd.x, nd.y);
    /* arrows show the actual sense of the reaction, so the label is its magnitude (signed values are in the table) */
    const text = fmtResLin(r.num < 0 ? Lin.from(r.lin).neg() : r.lin, r.dir ? 'F' : 'M', res.symKinds);
    if (r.dir) {
      const sg = r.num >= 0 ? 1 : -1;
      const ux = r.dir[0] * sg, uy = -r.dir[1] * sg;           // screen direction of the force on the structure
      const len = 18 + (Fmax ? DRAW.reactPx * Math.abs(r.num) / Fmax : 0);
      _dwArrow(ctx, sx - ux * len, sy - uy * len, sx, sy, DRAW.react, 9);
      if (S.show.values) _dwText(ctx, text, sx - ux * (len + 10), sy - uy * (len + 10), { color: DRAW.react, font: DRAW.fontSmall });
    } else {
      _dwMoment(ctx, sx, sy, DRAW.momR + 5, r.num > 0, DRAW.react);
      if (S.show.values) _dwText(ctx, text, sx - DRAW.momR - 10, sy + DRAW.momR + 6, { align: 'right', color: DRAW.react, font: DRAW.fontSmall });
    }
  });
}

function _dwProbe(ctx, res, geo, probe, W, H) {
  const g = geo[probe.memberId];
  if (!g) return;
  const x = clamp(probe.x, 0, g.L);
  const r = memberForcesAt(res, probe.memberId, x, probe.side === undefined ? 1 : probe.side);
  if (!r) return;
  const [px, py] = _dwPt(g, x);
  ctx.fillStyle = DRAW.probe; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(px, py, 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  const symL = !!(S.model.settings && S.model.settings.symL);
  const lines = [
    `Member ${probe.memberId}   x = ${fmtResNum(x)}${symL ? ' L' : ''}`,
    `N = ${fmtResLin(r.N, 'F', res.symKinds)}`,
    `V = ${fmtResLin(r.V, 'F', res.symKinds)}`,
    `M = ${fmtResLin(r.M, 'M', res.symKinds)}`
  ];
  ctx.font = DRAW.font;
  const tw = Math.max(...lines.map(t => ctx.measureText(t).width)) + 16, th = lines.length * 17 + 10;
  let bx = px + 14, by = py - th - 10;
  if (bx + tw > W - 4) bx = px - tw - 14;
  if (by < 4) by = py + 14;
  ctx.fillStyle = 'rgba(255,255,255,0.95)'; ctx.strokeStyle = DRAW.probe; ctx.lineWidth = 1;
  _dwRoundRect(ctx, bx, by, tw, th, 5); ctx.fill(); ctx.stroke();
  lines.forEach((t, i) => _dwText(ctx, t, bx + 8, by + 5 + 17 * i + 8.5, { align: 'left', color: i ? '#222' : DRAW.probe, halo: 'rgba(0,0,0,0)' }));
}

/* ---------------------------------------------------------------
   8. Hit testing (screen coordinates)
   --------------------------------------------------------------- */
function pickAt(px, py, tol) {
  tol = tol || 8;
  const model = S.model;
  let best = null;
  model.nodes.forEach(nd => {
    const [sx, sy] = w2s(nd.x, nd.y), d = Math.hypot(sx - px, sy - py);
    if (d <= tol && (!best || d < best.d)) best = { type: 'node', id: nd.id, d };
  });
  if (best) return best;
  model.members.forEach(m => {
    const A = nodeById(model, m.a), B = nodeById(model, m.b);
    if (!A || !B) return;
    const [ax, ay] = w2s(A.x, A.y), [bx, by] = w2s(B.x, B.y);
    const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
    if (!L2) return;
    const t = clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0, 1);
    const d = Math.hypot(ax + t * dx - px, ay + t * dy - py);
    if (d <= tol && (!best || d < best.d)) best = { type: 'member', id: m.id, t, x: t * Math.hypot(B.x - A.x, B.y - A.y), d };
  });
  return best;
}

/* ---------------------------------------------------------------
   9. Main entry
      opts: { fit, probe:{memberId,x,side}, rubber:{x0,y0,x1,y1}, cursor:{x,y} }  (world coords)
   --------------------------------------------------------------- */
function drawAll(canvas, opts) {
  opts = opts || {};
  viewInit();
  const { ctx, W, H } = prepCanvas(canvas);
  const model = S.model, show = S.show;
  const res = S.result && S.result.ok ? S.result : null;
  if (!S.view.fitted || opts.fit) viewFit(W, H);

  ctx.fillStyle = DRAW.bg; ctx.fillRect(0, 0, W, H);
  if (show.grid) _dwGrid(ctx, W, H);

  const geo = _dwGeometry(model, res);
  const cols = loadColumns(model), w = resultWeights(cols, S.symVals);
  const kinds = null;

  const diags = Array.isArray(show.diagrams) ? show.diagrams : (show.diagrams ? [show.diagrams] : []);
  if (res) diags.forEach(k => { if (DRAW[k] && DRAW[k].fill) _dwDiagram(ctx, res, geo, k); });
  if (res && show.deformed) _dwDeformed(ctx, res, geo);

  _dwMembers(ctx, model, geo);
  if (show.supports) _dwSupports(ctx, model);
  if (show.loads) _dwLoads(ctx, model, geo, cols, w);
  _dwNodes(ctx, model);
  if (res && show.reactions) _dwReactions(ctx, res, model);

  if (opts.rubber) {
    const [x0, y0] = w2s(opts.rubber.x0, opts.rubber.y0), [x1, y1] = w2s(opts.rubber.x1, opts.rubber.y1);
    ctx.save(); ctx.setLineDash([6, 4]); ctx.strokeStyle = DRAW.sel; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke(); ctx.restore();
  }
  if (opts.cursor) {
    const [cx, cy] = w2s(opts.cursor.x, opts.cursor.y);
    ctx.strokeStyle = DRAW.sel; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(cx - 7, cy); ctx.lineTo(cx + 7, cy); ctx.moveTo(cx, cy - 7); ctx.lineTo(cx, cy + 7); ctx.stroke();
    _dwText(ctx, `(${fmtResNum(opts.cursor.x)}, ${fmtResNum(opts.cursor.y)})`, cx + 10, cy + 12, { align: 'left', color: DRAW.sel, font: DRAW.fontSmall });
  }
  if (res && opts.probe) _dwProbe(ctx, res, geo, opts.probe, W, H);
  return { W, H, geo, kinds };
}