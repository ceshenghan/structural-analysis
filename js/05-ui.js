/* ============================================================
   Beam & Frame Analysis – 05-ui.js
   User interface bound to the elements of index.html:
     • settings (#selMode, #chkRigid, #selLen, #selFrac)
     • tools (#tools [data-tool]) and their option boxes (#toolOpts)
     • canvas interaction (#cv): add node / member, releases, supports,
       node and member loads, move, delete, select, pan, zoom
     • selection editor (#selection), member defaults (#defaults)
     • display options (#display), examples, undo, clear
     • symbol plot values (#symbols), results (#result)
     • JSON import / export, floating hint (#hint), status, legend
   Uses only the API of 01-core.js … 04-draw.js (no own data model).
   Depends on: 01-core.js, 02-model.js, 03-solver.js, 04-draw.js
   ============================================================ */
'use strict';

const UI = {
  tool: 'select',
  drag: null,          // {type:'pan',px,py} | {type:'node',id,moved}
  draft: null,         // first node id of a member being drawn
  mouse: null,         // last pointer position {px, py, wx, wy (snapped)}
  space: false,
  symKey: '',          // symbols currently shown in #symList (rebuild only when they change)
  canvas: null, hintEl: null, statusEl: null, legendEl: null
};

const TOOL_TEXT = {
  select: 'Select: click a node or member to edit it. Drag empty space to pan, wheel to zoom.',
  node: 'Add node: click empty space, or click a member to split it.',
  member: 'Add member: click the first node (or empty space), then the second one. Right-click / Esc cancels.',
  release: 'Release: click a node (all member ends), near a member end (that end only) or mid-member (inserts a node).',
  support: 'Support: click a node, or a member to insert a node there. The identical support again removes it.',
  nload: 'Node load: click a node, or a member to insert a node there. All values zero removes the load.',
  mload: 'Member load: click a member. x is measured from the member’s first node.',
  move: 'Move: drag a node.',
  delete: 'Delete: click a member or a node. An intermediate node of a straight member is merged.'
};
const OPT_BOX = { release: 'optRelease', support: 'optSupport', nload: 'optNload', mload: 'optMload' };
const REL_KEYS = { hinge: ['relA', 'relB'], shear: ['shrA', 'shrB'], axial: ['axA', 'axB'] };
const REL_FLAGS = { relA: 1, relB: 1, shrA: 1, shrB: 1, axA: 1, axB: 1 };

/* ---------------------------------------------------------------
   1. Small helpers
   --------------------------------------------------------------- */
function st() { return S.model.settings; }
function isSym() { return st().mode === 'sym'; }
function inL() { return isSym() && !!st().symL; }
function setStatus(msg, cls) {
  if (!UI.statusEl) return;
  UI.statusEl.textContent = msg || '';
  UI.statusEl.className = cls || '';
  UI.statusEl.hidden = !msg;
}
function toolStatus() { setStatus(TOOL_TEXT[UI.tool] || ''); }
function fmtX(x) { return inL() ? fmtLen(x, st()) : fmtDec(x, 5) + ' m'; }
function fmtL(lin, kind, unit = true) { return fmtQty(lin, kind, st(), { unit }); }
function linText(v) { return fmtLin(Lin.from(v), { frac: !!st().frac }); }
function inputVal(id) { const e = $(id); return e ? e.value : ''; }
function markBad(id, bad) { const e = $(id); if (e) e.classList.toggle('bad', !!bad); }
function defEI() { const v = parseFloat(inputVal('defEI')); return v > 0 ? v : 1e5; }
function defEA() { const v = parseFloat(inputVal('defEA')); return v > 0 ? v : 2e6; }
function defEIf() { return parseFactor(inputVal('defEIf')) || 1; }
/* a load value: Lin or null (marks the input) */
function readValue(id) {
  const l = parseValue(inputVal(id));
  markBad(id, !l);
  return l;
}
/* a position along a member of length L (empty → default text) */
function readPos(id, L, def) {
  const s = inputVal(id).trim() || def;
  const x = parsePos(s, L);
  const ok = x !== null && x >= -1e-9 * L && x <= L * (1 + 1e-9);
  markBad(id, !ok);
  return ok ? clamp(x, 0, L) : null;
}
/* a plain number, fractions allowed ("1/2") */
function readNumber(str) {
  const l = parseValue(str);
  return l && l.isConst() ? l.get(CONST_KEY) : null;
}

/* ---------------------------------------------------------------
   2. Model normalisation (examples / imported files / mode switch)
      numeric mode : member.EI, member.EA are actual values (kN·m², kN), settings.EI = EA = 1
      symbolic mode: member.EI is a factor of the symbol EI, axially rigid
   --------------------------------------------------------------- */
function normalizeModel(model) {
  const s = model.settings;
  if (s.mode !== 'sym') s.mode = 'num';
  const kEI = s.EI > 0 ? s.EI : 1, kEA = s.EA > 0 ? s.EA : 1;
  model.members.forEach(m => {
    if (s.mode === 'num') m.EI = (m.EI > 0 ? m.EI : 1) * kEI;
    m.EA = (m.EA > 0 ? m.EA : 1) * kEA;
    if (!(m.EA > 1)) m.EA = defEA();
    if (s.mode === 'num' && kEI === 1 && m.EI === 1) m.EI = defEI();
  });
  s.EI = 1; s.EA = 1;
  if (s.mode === 'sym') s.axial = 'rigid';
  if (s.mode === 'num') s.symL = false;
  return model;
}
function niceFactor(x) {
  const f = toFraction(x, 100, 1e-6);
  return f && f.n > 0 ? f.n / f.d : x;
}
function switchMode(mode) {
  const s = st();
  if (mode === s.mode) return;
  histPush();
  if (mode === 'sym') {
    S.model.members.forEach(m => { m.EI = niceFactor(m.EI / defEI()); });
    s.mode = 'sym'; s.axial = 'rigid'; s.symL = true; s.frac = true;
  } else {
    S.model.members.forEach(m => { m.EI = m.EI * defEI(); if (!(m.EA > 1)) m.EA = defEA(); });
    s.mode = 'num'; s.symL = false; s.frac = false;
    s.axial = $('chkRigid').checked ? 'rigid' : 'flex';
  }
  s.EI = 1; s.EA = 1;
  commit();
}

/* ---------------------------------------------------------------
   3. Solve and render pipeline
   --------------------------------------------------------------- */
function solveNow() {
  if (!S.model.members.length) { S.result = null; S.issues = []; return; }
  try { runSolve(); }
  catch (e) {
    console.error(e);
    S.result = null;
    S.issues = [{ level: 'error', msg: 'Internal error: ' + (e && e.message ? e.message : e) }];
  }
}
function drawCanvas(opts) {
  if (!UI.canvas) return;
  const o = Object.assign({}, opts || {});
  if (UI.mouse && UI.tool === 'member' && UI.draft !== null) {
    const A = nodeById(S.model, UI.draft);
    if (A) o.rubber = { x0: A.x, y0: A.y, x1: UI.mouse.wx, y1: UI.mouse.wy };
  }
  if (UI.mouse && (UI.tool === 'node' || UI.tool === 'member')) o.cursor = { x: UI.mouse.wx, y: UI.mouse.wy };
  drawAll(UI.canvas, o);
  updateLegend();
}
function renderAll(opts) {
  drawCanvas(opts);
  renderSelection();
  renderSymbols();
  renderResults();
  syncControls();
}
/* after every model change */
function commit(opts) {
  S.dirty = true;
  solveNow();
  renderAll(opts);
}

/* ---------------------------------------------------------------
   4. Settings, defaults, display options, tools
   --------------------------------------------------------------- */
function syncControls() {
  const s = st(), sym = s.mode === 'sym';
  $('selMode').value = sym ? 'sym' : 'num';
  const rig = $('chkRigid');
  rig.checked = sym || s.axial === 'rigid';
  rig.disabled = sym;
  $('selLen').value = sym && s.symL ? 'L' : 'm';
  $('selLen').disabled = !sym;
  $('selFrac').value = s.frac ? 'frac' : 'dec';
  $('defNum').hidden = sym;
  $('defSym').hidden = !sym;
  const sh = S.show;
  $('chkLoads').checked = !!sh.loads;
  $('chkReac').checked = !!sh.reactions;
  $('chkDef').checked = !!sh.deformed;
  ['N', 'V', 'M'].forEach(k => { $('chk' + k).checked = (sh.diagrams || []).includes(k); });
  $('chkVals').checked = !!sh.values;
  $('chkSnap').checked = sh.snap !== false;
  $('selMlabel').value = sh.mAbs ? 'abs' : 'signed';
  $('btnUndo').disabled = !S.hist.undo.length;
}
function bindSettings() {
  $('selMode').addEventListener('change', e => switchMode(e.target.value));
  $('chkRigid').addEventListener('change', e => {
    if (isSym()) return;
    histPush(); st().axial = e.target.checked ? 'rigid' : 'flex'; commit();
  });
  $('selLen').addEventListener('change', e => { histPush(); st().symL = e.target.value === 'L'; commit(); });
  $('selFrac').addEventListener('change', e => { st().frac = e.target.value === 'frac'; renderAll(); });
  $('btnApplyAll').addEventListener('click', () => {
    if (!S.model.members.length) return;
    histPush();
    S.model.members.forEach(m => {
      if (isSym()) m.EI = defEIf();
      else { m.EI = defEI(); m.EA = defEA(); }
    });
    commit();
    setStatus('Default stiffness applied to all members.', 'ok');
  });
  ['defEI', 'defEA'].forEach(id => $(id).addEventListener('change', e => markBad(id, !(parseFloat(e.target.value) > 0))));
  $('defEIf').addEventListener('change', () => markBad('defEIf', !parseFactor(inputVal('defEIf'))));
}
function bindDisplay() {
  const flag = (id, key) => $(id).addEventListener('change', e => { S.show[key] = e.target.checked; drawCanvas(); });
  flag('chkLoads', 'loads'); flag('chkReac', 'reactions'); flag('chkDef', 'deformed');
  flag('chkVals', 'values'); flag('chkSnap', 'snap');
  ['N', 'V', 'M'].forEach(k => $('chk' + k).addEventListener('change', () => {
    S.show.diagrams = ['N', 'V', 'M'].filter(d => $('chk' + d).checked);
    drawCanvas();
  }));
  $('rngDiag').addEventListener('input', e => { S.show.scale = Math.pow(10, +e.target.value || 0); drawCanvas(); });
  $('rngDef').addEventListener('input', e => { S.show.defScale = Math.pow(10, +e.target.value || 0); drawCanvas(); });
  $('selMlabel').addEventListener('change', e => { S.show.mAbs = e.target.value === 'abs'; drawCanvas(); });
}
function setTool(tool) {
  if (!TOOL_TEXT[tool]) return;
  UI.tool = tool; UI.draft = null; UI.drag = null;
  document.querySelectorAll('#tools [data-tool]').forEach(b => b.classList.toggle('active', b.dataset.tool === tool));
  for (const t in OPT_BOX) $(OPT_BOX[t]).classList.toggle('show', t === tool);
  UI.canvas.className = 'tool-' + tool;
  toolStatus();
  drawCanvas();
}
function syncMemberLoadOpts() {
  const t = inputVal('mlType');
  $('mlDist').hidden = !(t === 'udl' || t === 'trap');
  $('mlQ2').hidden = t !== 'trap';
  $('mlPoint').hidden = t !== 'point';
  $('mlMom').hidden = t !== 'moment';
  $('mlDirWrap').hidden = t === 'moment';
}
function bindTools() {
  document.querySelectorAll('#tools [data-tool]').forEach(b => b.addEventListener('click', () => setTool(b.dataset.tool)));
  $('mlType').addEventListener('change', syncMemberLoadOpts);
  syncMemberLoadOpts();
}

/* ---------------------------------------------------------------
   5. Editing operations
   --------------------------------------------------------------- */
/* parameter t of the point of member m nearest to world point (x,y) */
function memberParamAt(m, x, y) {
  const A = nodeById(S.model, m.a), B = nodeById(S.model, m.b);
  return distToSegment(x, y, A.x, A.y, B.x, B.y);
}
/* node at the clicked place: existing node, or a node inserted into the clicked member (caller pushed history) */
function nodeAtHit(hit) {
  if (!hit) return null;
  if (hit.type === 'node') return nodeById(S.model, hit.id);
  const m = memberById(S.model, hit.id);
  if (!m) return null;
  let t = hit.t;
  const g = memberGeom(S.model, m);
  if (S.show.snap !== false && UI.mouse) {                // snapped point projected on the member
    const p = memberParamAt(m, UI.mouse.wx, UI.mouse.wy);
    if (p.d <= 0.25 * gridStep() || p.d <= 1e-6 * g.L) t = p.t;
  }
  if (t * g.L * S.view.k < 6) return g.A;
  if ((1 - t) * g.L * S.view.k < 6) return g.B;
  const r = splitMemberAt(S.model, m, t);
  return r ? r.node : null;
}
/* member from a to b, oriented left → right (vertical: bottom → top) */
function createMember(aId, bId) {
  let A = nodeById(S.model, aId), B = nodeById(S.model, bId);
  const tol = 1e-9 * Math.max(1, Math.abs(A.x), Math.abs(B.x));
  if (B.x < A.x - tol || (Math.abs(B.x - A.x) <= tol && B.y < A.y)) { const t = A; A = B; B = t; }
  const made = addMember(S.model, A.id, B.id);
  made.forEach(m => {
    m.EI = isSym() ? defEIf() : defEI();
    m.EA = defEA();
  });
  return made;
}
/* nodes left without members are removed unless they carry a support or a load */
function removeOrphans(ids) {
  ids.forEach(id => {
    const n = nodeById(S.model, id);
    if (n && !nodeMembers(S.model, id).length && (!n.sup || n.sup === 'none') && !n.loads.length) deleteNode(S.model, id);
  });
}
/* merge the two collinear members at a plain intermediate node; releases at that node are dropped with it */
function mergeNode(id) {
  const ms = nodeMembers(S.model, id);
  if (ms.length !== 2) return false;
  const saved = ms.map(m => ({ m, f: Object.keys(REL_FLAGS).map(k => m[k]) }));
  const hinge = nodeById(S.model, id).hinge;
  ms.forEach(m => { const e = m.a === id ? 'A' : 'B'; m['rel' + e] = m['shr' + e] = m['ax' + e] = false; });
  nodeById(S.model, id).hinge = false;
  if (mergeAtNode(S.model, id)) return true;
  saved.forEach(s => Object.keys(REL_FLAGS).forEach((k, i) => { s.m[k] = s.f[i]; }));
  nodeById(S.model, id).hinge = hinge;
  return false;
}

function opNode(hit) {
  if (hit && hit.type === 'node') { S.sel = { type: 'node', id: hit.id }; renderAll(); return; }
  histPush();
  const nd = hit && hit.type === 'member' ? nodeAtHit(hit) : placeNode(S.model, UI.mouse.wx, UI.mouse.wy);
  if (nd) S.sel = { type: 'node', id: nd.id };
  commit();
}
function opMember(hit) {
  let nd = hit && hit.type === 'node' ? nodeById(S.model, hit.id) : null;
  if (!nd) {
    histPush();
    nd = hit && hit.type === 'member' ? nodeAtHit(hit) : placeNode(S.model, UI.mouse.wx, UI.mouse.wy);
    if (!nd) return;
    if (UI.draft === null) { UI.draft = nd.id; commit(); setStatus(`Member from node ${nd.id}: click the end node.`); return; }
  } else if (UI.draft === null) {
    UI.draft = nd.id; drawCanvas(); setStatus(`Member from node ${nd.id}: click the end node.`); return;
  } else histPush();
  if (nd.id === UI.draft) return;
  const made = createMember(UI.draft, nd.id);
  UI.draft = null;
  if (made.length) S.sel = { type: 'member', id: made[made.length - 1].id };
  commit();
  if (made.length) toolStatus(); else setStatus('That member already exists.', 'warn');
}
function toggleRelease(type, ends, node) {
  /* ends: [{m, end:'A'|'B'}] */
  if (!ends.length) { setStatus('There is no member end here.', 'warn'); return false; }
  if (type === 'rigid') {
    ends.forEach(e => ['rel', 'shr', 'ax'].forEach(p => { e.m[p + e.end] = false; }));
    if (node) node.hinge = false;
    return true;
  }
  const [ka, kb] = REL_KEYS[type];
  const key = e => (e.end === 'A' ? ka : kb);
  const has = e => !!e.m[key(e)] || (type === 'hinge' && node && node.hinge);
  const all = ends.every(has);
  ends.forEach(e => { e.m[key(e)] = !all; });
  if (type === 'hinge' && node) node.hinge = false;      // member-end flags carry the hinge from now on
  return true;
}
function opRelease(hit, px, py) {
  if (!hit) return;
  const type = inputVal('relType');
  histPush();
  let ok = false;
  if (hit.type === 'node') {
    const nd = nodeById(S.model, hit.id);
    ok = toggleRelease(type, nodeMembers(S.model, nd.id).map(m => ({ m, end: m.a === nd.id ? 'A' : 'B' })), nd);
  } else {
    const m = memberById(S.model, hit.id), g = memberGeom(S.model, m);
    const [ax, ay] = w2s(g.A.x, g.A.y), [bx, by] = w2s(g.B.x, g.B.y);
    const Ls = Math.hypot(bx - ax, by - ay), near = Math.min(24, 0.3 * Ls);
    if (Math.hypot(px - ax, py - ay) <= near) ok = toggleRelease(type, [{ m, end: 'A' }]);
    else if (Math.hypot(px - bx, py - by) <= near) ok = toggleRelease(type, [{ m, end: 'B' }]);
    else if (type !== 'rigid') {
      const nd = nodeAtHit(hit);
      const left = nd && (m.b === nd.id ? m : S.model.members.find(x => x.b === nd.id));
      if (left) ok = toggleRelease(type, [{ m: left, end: 'B' }]);
    }
  }
  if (!ok) { S.hist.undo.pop(); return; }
  commit();
}
function opSupport(hit) {
  if (!hit) { setStatus('Click a node or a member.', 'warn'); return; }
  const type = inputVal('supType');
  const ang = parseFloat(inputVal('supAngle')) || 0;
  histPush();
  const nd = nodeAtHit(hit);
  if (!nd) { S.hist.undo.pop(); return; }
  if (nd.sup === type && nearly(normAng(nd.ang || 0), normAng(ang))) { nd.sup = 'none'; nd.ang = 0; }
  else { nd.sup = type; nd.ang = ang; }
  S.sel = { type: 'node', id: nd.id };
  commit();
}
function opNodeLoad(hit) {
  if (!hit) { setStatus('Click a node or a member.', 'warn'); return; }
  const fx = readValue('nFx'), fy = readValue('nFy'), mz = readValue('nM');
  if (!fx || !fy || !mz) { setStatus('Invalid load value (e.g. 10, -P, 3/2 P, M0).', 'err'); return; }
  histPush();
  const nd = nodeAtHit(hit);
  if (!nd) { S.hist.undo.pop(); return; }
  const loads = [];
  if (!fx.isZero()) loads.push(newNodeLoad({ kind: 'F', val: fx, ang: 0 }));
  if (!fy.isZero()) loads.push(newNodeLoad({ kind: 'F', val: fy, ang: 90 }));
  if (!mz.isZero()) loads.push(newNodeLoad({ kind: 'M', val: mz }));
  nd.loads = loads;
  S.sel = { type: 'node', id: nd.id };
  commit();
  setStatus(loads.length ? `Load set at node ${nd.id}.` : `Loads removed from node ${nd.id}.`, 'ok');
}
function opMemberLoad(hit) {
  if (!hit || hit.type !== 'member') { setStatus('Click a member.', 'warn'); return; }
  const m = memberById(S.model, hit.id), L = memberGeom(S.model, m).L;
  const type = inputVal('mlType');
  const dir = { gy: { ang: 90, local: false }, gx: { ang: 0, local: false }, ly: { ang: 90, local: true }, lx: { ang: 0, local: true } }[inputVal('mlDir')] || { ang: 90, local: false };
  let ld = null;
  if (type === 'udl' || type === 'trap') {
    const v1 = readValue('mlV1'), v2 = type === 'trap' ? readValue('mlV2') : null;
    const x1 = readPos('mlX1', L, '0'), x2 = readPos('mlX2', L, 'L');
    if (!v1 || (type === 'trap' && !v2)) { setStatus('Invalid load intensity.', 'err'); return; }
    if (x1 === null || x2 === null || x2 - x1 <= 1e-9 * L) { setStatus('Invalid range: need 0 ≤ x₁ < x₂ ≤ L.', 'err'); return; }
    if (v1.isZero() && (!v2 || v2.isZero())) { setStatus('The load intensity is zero.', 'warn'); return; }
    ld = newMemberLoad({ kind: 'q', val: v1, val2: type === 'trap' ? v2 : null, t1: x1 / L, t2: x2 / L, ang: dir.ang, local: dir.local });
  } else if (type === 'point') {
    const P = readValue('mlP'), x = readPos('mlXp', L, 'L/2');
    if (!P || P.isZero()) { setStatus('Invalid load value.', 'err'); return; }
    if (x === null) { setStatus('Invalid position: need 0 ≤ x ≤ L.', 'err'); return; }
    ld = newMemberLoad({ kind: 'F', val: P, t: x / L, ang: dir.ang, local: dir.local });
  } else {
    const C = readValue('mlC'), x = readPos('mlXm', L, 'L/2');
    if (!C || C.isZero()) { setStatus('Invalid moment value.', 'err'); return; }
    if (x === null) { setStatus('Invalid position: need 0 ≤ x ≤ L.', 'err'); return; }
    ld = newMemberLoad({ kind: 'M', val: C, t: x / L });
  }
  histPush();
  m.loads.push(ld);
  S.sel = { type: 'member', id: m.id };
  commit();
  setStatus(`Load added to member ${m.id}.`, 'ok');
}
function opDelete(hit) {
  if (!hit) return;
  histPush();
  if (hit.type === 'node') {
    if (!mergeNode(hit.id)) {
      const others = nodeMembers(S.model, hit.id).map(m => (m.a === hit.id ? m.b : m.a));
      deleteNode(S.model, hit.id);
      removeOrphans(others);
    }
  } else {
    const m = memberById(S.model, hit.id);
    deleteMember(S.model, hit.id);
    removeOrphans([m.a, m.b]);
  }
  S.sel = null; S.hover = null;
  commit();
}
function deleteSelection() {
  if (!S.sel) return;
  opDelete({ type: S.sel.type, id: S.sel.id });
}
function undo() {
  if (!histUndo()) { setStatus('Nothing to undo.', 'warn'); return; }
  UI.draft = null; commit(); setStatus('Undone.');
}
function redo() {
  if (!histRedo()) return;
  UI.draft = null; commit(); setStatus('Redone.');
}

/* ---------------------------------------------------------------
   6. Canvas interaction
   --------------------------------------------------------------- */
function evPos(e) { const r = UI.canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
function updateMouse(e, px, py) {
  let [x, y] = s2w(px, py);
  if (S.show.snap !== false && !e.shiftKey) [x, y] = snapToGrid(x, y);
  UI.mouse = { px, py, wx: snapZero(x, 1, 1e-12), wy: snapZero(y, 1, 1e-12) };
}
function cancelOp() {
  UI.draft = null; UI.drag = null;
  UI.canvas.classList.remove('panning');
  toolStatus(); drawCanvas();
}
function bindCanvas() {
  const cv = UI.canvas;
  cv.addEventListener('pointerdown', e => {
    const [px, py] = evPos(e);
    updateMouse(e, px, py);
    if (e.button === 2) return;
    cv.setPointerCapture(e.pointerId);
    const hit = pickAt(px, py, 9);
    if (e.button === 1 || UI.space) { UI.drag = { type: 'pan', px, py }; cv.classList.add('panning'); return; }
    if (e.button !== 0) return;
    switch (UI.tool) {
      case 'select':
        if (hit) { S.sel = { type: hit.type, id: hit.id }; renderAll(); }
        else { S.sel = null; UI.drag = { type: 'pan', px, py }; cv.classList.add('panning'); renderAll(); }
        break;
      case 'move':
        if (hit && hit.type === 'node') { UI.drag = { type: 'node', id: hit.id, moved: false }; S.sel = { type: 'node', id: hit.id }; }
        else { UI.drag = { type: 'pan', px, py }; cv.classList.add('panning'); }
        break;
      case 'node': opNode(hit); break;
      case 'member': opMember(hit); break;
      case 'release': opRelease(hit, px, py); break;
      case 'support': opSupport(hit); break;
      case 'nload': opNodeLoad(hit); break;
      case 'mload': opMemberLoad(hit); break;
      case 'delete': opDelete(hit); break;
    }
  });
  cv.addEventListener('pointermove', e => {
    const [px, py] = evPos(e);
    updateMouse(e, px, py);
    const d = UI.drag;
    if (d && d.type === 'pan') { viewPan(px - d.px, py - d.py); d.px = px; d.py = py; drawCanvas(); hideHint(); return; }
    if (d && d.type === 'node') {
      const nd = nodeById(S.model, d.id);
      if (!nd || (nearly(nd.x, UI.mouse.wx) && nearly(nd.y, UI.mouse.wy))) return;
      if (!d.moved) { histPush(); d.moved = true; }
      if (moveNode(S.model, d.id, UI.mouse.wx, UI.mouse.wy)) commit();
      return;
    }
    const hit = pickAt(px, py, 9);
    const old = S.hover;
    S.hover = hit ? { type: hit.type, id: hit.id } : null;
    showHint(hit, px, py);
    if (UI.tool === 'node' || UI.tool === 'member' || JSON.stringify(old) !== JSON.stringify(S.hover)) drawCanvas();
  });
  const end = () => {
    const d = UI.drag;
    UI.drag = null;
    cv.classList.remove('panning');
    if (d && d.type === 'node' && d.moved) renderAll();
  };
  cv.addEventListener('pointerup', end);
  cv.addEventListener('pointercancel', end);
  cv.addEventListener('pointerleave', () => { S.hover = null; UI.mouse = null; hideHint(); if (!UI.drag) drawCanvas(); });
  cv.addEventListener('wheel', e => {
    e.preventDefault();
    const [px, py] = evPos(e);
    viewZoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, px, py);
    drawCanvas();
  }, { passive: false });
  cv.addEventListener('contextmenu', e => { e.preventDefault(); cancelOp(); });
}
function bindKeys() {
  document.addEventListener('keydown', e => {
    const t = e.target, tag = t && t.tagName;
    if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
    const k = e.key.toLowerCase();
    if ((e.ctrlKey || e.metaKey) && k === 'z') { e.preventDefault(); e.shiftKey ? redo() : undo(); return; }
    if ((e.ctrlKey || e.metaKey) && k === 'y') { e.preventDefault(); redo(); return; }
    if (e.key === 'Escape') { cancelOp(); return; }
    if (e.key === 'Delete') { deleteSelection(); return; }
    if (e.key === ' ' && !UI.space) { UI.space = true; e.preventDefault(); }
  });
  document.addEventListener('keyup', e => { if (e.key === ' ') UI.space = false; });
}

/* floating hint: member direction / position and, when solved, N, V, M at the pointer */
function hideHint() { if (UI.hintEl) UI.hintEl.classList.remove('show'); }
function showHint(hit, px, py) {
  const h = UI.hintEl;
  if (!h) return;
  if (!hit) { hideHint(); return; }
  let txt = '';
  if (hit.type === 'node') {
    const nd = nodeById(S.model, hit.id);
    txt = `Node ${nd.id}  (${fmtX(nd.x)}, ${fmtX(nd.y)})`;
    if (nd.sup && nd.sup !== 'none') txt += `\n${SUPPORTS[nd.sup].label} support` + (nd.ang ? `, rotation ${nd.ang}°` : '');
  } else {
    const m = memberById(S.model, hit.id), g = memberGeom(S.model, m);
    const x = clamp(hit.x, 0, g.L);
    txt = `Member ${m.id}: node ${m.a} → node ${m.b}\nL = ${fmtX(g.L)},  x = ${fmtX(x)} from node ${m.a}`;
    if (S.result && S.result.ok) {
      const r = memberForcesAt(S.result, m.id, x, 1);
      if (r) txt += `\nN = ${fmtL(r.N, 'force')}\nV = ${fmtL(r.V, 'force')}\nM = ${fmtL(r.M, 'moment')}`;
    }
  }
  h.textContent = txt;
  h.classList.add('show');
  const main = $('main'), W = main.clientWidth, H = main.clientHeight;
  let x = px + 14, y = py + 16;
  if (x + h.offsetWidth > W - 4) x = px - h.offsetWidth - 10;
  if (y + h.offsetHeight > H - 4) y = py - h.offsetHeight - 10;
  h.style.left = Math.max(2, x) + 'px';
  h.style.top = Math.max(2, y) + 'px';
}
function updateLegend() {
  const el = UI.legendEl;
  if (!el) return;
  const res = S.result && S.result.ok;
  const items = [];
  if (res) {
    (S.show.diagrams || []).forEach(k => items.push(`<span><i class="c-${k}"></i>${k}</span>`));
    if (S.show.deformed) items.push('<span><i class="c-def"></i>deformed shape</span>');
  }
  el.innerHTML = items.join('');
  el.hidden = !items.length;
}

/* ---------------------------------------------------------------
   7a. Pieces of one distributed load
   Inserting a node into a loaded member (Add node, Node load, Support or
   Release clicked mid-member) splits its distributed load into one piece per
   segment. To the user it is still one load, so the × in the editor removes
   the whole continuous load: all pieces on collinear neighbouring segments
   with the same direction and a continuous intensity. The chain stops at
   supports, so separately loaded spans of a continuous beam stay separate.
   --------------------------------------------------------------- */
function _linEq(a, b) { return Lin.from(a).sub(Lin.from(b)).isZero(1e-9); }
function _memberDir(m) {
  const A = nodeById(S.model, m.a), B = nodeById(S.model, m.b);
  const L = Math.hypot(B.x - A.x, B.y - A.y) || 1;
  return [(B.x - A.x) / L, (B.y - A.y) / L];
}
function _sameDistLoad(p, q) {
  const ap = p.ang === undefined ? -90 : p.ang, aq = q.ang === undefined ? -90 : q.ang;
  return q.kind === 'q' && nearly(ap, aq) && !!p.local === !!q.local && !!p.proj === !!q.proj;
}
function loadPieces(m, ld) {
  const out = [{ m, ld }];
  if (!ld || ld.kind !== 'q') return out;
  const walk = fwd => {
    let cm = m, cl = ld;
    for (let guard = 0; guard < 1000; guard++) {
      const lo = Math.min(cl.t1, cl.t2), hi = Math.max(cl.t1, cl.t2);
      if (fwd ? hi < 1 - 1e-9 : lo > 1e-9) return;             // piece does not reach the node
      const nodeId = fwd ? cm.b : cm.a, nd = nodeById(S.model, nodeId);
      if (!nd || (nd.sup && nd.sup !== 'none')) return;          // stop at supports
      const nb = nodeMembers(S.model, nodeId).filter(x => x !== cm);
      if (nb.length !== 1) return;                               // only plain intermediate nodes
      const n = nb[0];
      if ((fwd ? n.a : n.b) !== nodeId) return;                  // same orientation, as produced by a split
      const d0 = _memberDir(cm), d1 = _memberDir(n);
      if (Math.abs(d0[0] * d1[1] - d0[1] * d1[0]) > 1e-9 || d0[0] * d1[0] + d0[1] * d1[1] < 0) return;
      const vEnd = fwd ? (cl.val2 || cl.val) : cl.val;           // intensity at the shared node
      const nl = n.loads.find(q => _sameDistLoad(cl, q) && !out.some(o => o.ld === q) &&
        (fwd ? Math.min(q.t1, q.t2) < 1e-9 && _linEq(q.val, vEnd)
             : Math.max(q.t1, q.t2) > 1 - 1e-9 && _linEq(q.val2 || q.val, vEnd)));
      if (!nl) return;
      out.push({ m: n, ld: nl });
      cm = n; cl = nl;
    }
  };
  walk(true); walk(false);
  return out;
}

/* ---------------------------------------------------------------
   7. Selection editor (#selection)
   --------------------------------------------------------------- */
function dirText(ld) {
  const a = ld.ang === undefined ? -90 : ld.ang;
  let s;
  if (ld.local) s = a === 90 || a === -90 ? 'local ⊥' : (a === 0 || a === 180 ? 'local ∥' : `local ${a}°`);
  else s = a === 90 || a === -90 ? 'global Y' : (a === 0 || a === 180 ? 'global X' : `${a}°`);
  const neg = a === -90 || a === 180;                       // stored downwards / leftwards → flip sign for display
  return { s: s + (ld.proj ? ', per horizontal length' : ''), neg };
}
function describeNodeLoad(ld) {
  if (ld.kind === 'M') return `M = ${linText(ld.val)} (CCW +)`;
  const a = ld.ang === undefined ? -90 : ld.ang;
  if (a === 0) return `Fx = ${linText(ld.val)}`;
  if (a === 90) return `Fy = ${linText(ld.val)}`;
  if (a === -90) return `Fy = ${linText(Lin.from(ld.val).neg())}`;
  if (a === 180) return `Fx = ${linText(Lin.from(ld.val).neg())}`;
  return `F = ${linText(ld.val)} at ${a}°`;
}
function describeMemberLoad(ld, L) {
  const pos = t => fmtPos(t * L, L, true);
  if (ld.kind === 'M') return `C = ${linText(ld.val)} at x = ${pos(ld.t)}`;
  const d = dirText(ld), v = x => linText(d.neg ? Lin.from(x).neg() : x);
  if (ld.kind === 'F') return `P = ${v(ld.val)} at x = ${pos(ld.t)} (${d.s})`;
  const q = ld.val2 ? `${v(ld.val)} → ${v(ld.val2)}` : v(ld.val);
  return `q = ${q} from ${pos(ld.t1)} to ${pos(ld.t2)} (${d.s})`;
}
function renderSelection() {
  const box = $('selection');
  const sel = S.sel;
  const obj = sel ? (sel.type === 'node' ? nodeById(S.model, sel.id) : memberById(S.model, sel.id)) : null;
  if (!obj) { S.sel = null; box.innerHTML = ''; return; }
  const res = S.result && S.result.ok ? S.result : null;
  let h = '<div class="box">';
  if (sel.type === 'node') {
    const nd = obj, ms = nodeMembers(S.model, nd.id);
    h += `<div class="sel-title">Node ${nd.id}<span class="badge">${ms.length ? 'members ' + ms.map(m => m.id).join(', ') : 'not connected'}</span></div>
      <div class="kv">
        <span>x (${inL() ? 'L' : 'm'})</span><input data-f="x" value="${esc(fmtNum(nd.x, true, 8))}">
        <span>y (${inL() ? 'L' : 'm'})</span><input data-f="y" value="${esc(fmtNum(nd.y, true, 8))}">
        <span>Support</span><select data-f="sup">${SUPPORT_ORDER.map(k => `<option value="${k}"${(nd.sup === 'slider' ? 'guided' : (nd.sup || 'none')) === k ? ' selected' : ''}>${esc(SUPPORTS[k].label)}</option>`).join('')}</select>
        <span>rotation (°)</span><input data-f="ang" type="number" step="15" value="${nd.ang || 0}">
      </div>`;
    h += '<b>Loads</b>' + (nd.loads.length
      ? '<ul class="loads">' + nd.loads.map((ld, i) => `<li><span class="ld">${esc(describeNodeLoad(ld))}</span><button type="button" class="small danger" data-del="${i}" title="remove this load">×</button></li>`).join('') + '</ul>'
      : ' <span class="muted">none – use the Node load tool</span>');
    if (res && res.nodes[nd.id]) {
      const nr = res.nodes[nd.id];
      h += `<div class="kv"><span>u<sub>x</sub></span><span>${esc(fmtL(nr.u[0], 'defl'))}</span>
        <span>u<sub>y</sub></span><span>${esc(fmtL(nr.u[1], 'defl'))}</span>
        <span>θ (CCW +)</span><span>${esc(fmtL(nr.u[2], 'rot'))}</span>`;
      nr.reactions.forEach(r => { h += `<span>${esc(reactionLabel(r))}</span><span>${esc(fmtL(r.lin, r.dir ? 'force' : 'moment'))}</span>`; });
      h += '</div>';
    }
  } else {
    const m = obj, g = memberGeom(S.model, m);
    const relBox = (key, label) => `<label title="${label}"><input type="checkbox" data-f="${key}"${m[key] ? ' checked' : ''}>${label.split(' ')[0]}</label>`;
    h += `<div class="sel-title">Member ${m.id}<span class="badge">node ${m.a} → node ${m.b}</span></div>
      <div class="kv">
        <span>Length</span><span>${esc(fmtX(g.L))}</span>`;
    if (isSym()) h += `<span>EI factor</span><span><input data-f="EIf" value="${esc(fmtNum(m.EI, true))}"> × EI</span>`;
    else {
      h += `<span>EI (kN·m²)</span><input data-f="EI" type="number" class="w110" value="${m.EI}">`;
      h += `<span>EA (kN)</span><input data-f="EA" type="number" class="w110" value="${m.EA}"${st().axial === 'rigid' ? ' disabled title="axially rigid"' : ''}>`;
    }
    h += `<span>End ${m.a} (start)</span><span>${relBox('relA', 'M moment release')} ${relBox('shrA', 'V shear release')} ${relBox('axA', 'N axial release')}</span>
        <span>End ${m.b} (end)</span><span>${relBox('relB', 'M moment release')} ${relBox('shrB', 'V shear release')} ${relBox('axB', 'N axial release')}</span>
      </div>
      <button type="button" class="small" data-act="flip">reverse direction</button>`;
    h += '<br><b>Loads</b>' + (m.loads.length
      ? '<ul class="loads">' + m.loads.map((ld, i) => {
          const others = loadPieces(m, ld).slice(1).map(p => p.m.id);
          const note = others.length ? ` <span class="muted">(continues on member${others.length > 1 ? 's' : ''} ${others.join(', ')})</span>` : '';
          return `<li><span class="ld">${esc(describeMemberLoad(ld, g.L))}</span>${note}<button type="button" class="small danger" data-del="${i}" title="remove this load">×</button></li>`;
        }).join('') + '</ul>'
      : ' <span class="muted">none – use the Member load tool</span>');
  }
  h += '<button type="button" class="small danger" data-act="delete">delete ' + sel.type + '</button></div>';
  box.innerHTML = h;
}
function removeLoad(obj, idx) {
  const ld = obj.loads[idx];
  if (!ld) return;
  const pieces = S.sel.type === 'member' ? loadPieces(obj, ld) : [{ m: obj, ld }];
  histPush();
  pieces.forEach(p => { const i = p.m.loads.indexOf(p.ld); if (i >= 0) p.m.loads.splice(i, 1); });
  commit();
  setStatus(pieces.length > 1 ? `Load removed from members ${pieces.map(p => p.m.id).join(', ')}.` : 'Load removed.', 'ok');
}
function bindSelection() {
  const box = $('selection');
  box.addEventListener('change', e => {
    const f = e.target.dataset.f;
    if (!f || !S.sel) return;
    const obj = S.sel.type === 'node' ? nodeById(S.model, S.sel.id) : memberById(S.model, S.sel.id);
    if (!obj) return;
    const raw = e.target.type === 'checkbox' ? e.target.checked : e.target.value;
    histPush();
    let ok = true;
    if (f === 'x' || f === 'y') {
      const v = readNumber(raw);
      ok = v !== null && moveNode(S.model, obj.id, f === 'x' ? v : obj.x, f === 'y' ? v : obj.y);
    } else if (f === 'sup') obj.sup = raw;
    else if (f === 'ang') { const v = parseFloat(raw); ok = isFinite(v); if (ok) obj.ang = v; }
    else if (f === 'EIf') { const v = parseFactor(raw); ok = !!v; if (ok) obj.EI = v; }
    else if (f === 'EI' || f === 'EA') { const v = parseFloat(raw); ok = v > 0; if (ok) obj[f] = v; }
    else if (f in REL_FLAGS) obj[f] = !!raw;
    if (!ok) { S.hist.undo.pop(); e.target.classList.add('bad'); setStatus('Invalid value.', 'err'); return; }
    commit();
  });
  box.addEventListener('click', e => {
    const b = e.target.closest('button');
    if (!b || !S.sel) return;
    const obj = S.sel.type === 'node' ? nodeById(S.model, S.sel.id) : memberById(S.model, S.sel.id);
    if (!obj) return;
    if (b.dataset.del !== undefined) removeLoad(obj, +b.dataset.del);
    else if (b.dataset.act === 'flip') { histPush(); flipMember(S.model, obj); commit(); }
    else if (b.dataset.act === 'delete') deleteSelection();
  });
}

/* ---------------------------------------------------------------
   8. Symbol plot values (#symbols)
   --------------------------------------------------------------- */
function renderSymbols() {
  const syms = modelSymbols(S.model);
  $('symbols').hidden = !syms.length;
  const key = syms.join(',');
  if (key === UI.symKey) return;                         // keep focus while typing
  UI.symKey = key;
  syms.forEach(s => { if (S.symVals[s] === undefined) S.symVals[s] = 1; });
  $('symList').innerHTML = syms.map(s =>
    `<span class="sym-row"><b>${esc(s)}</b> = <input class="w50" data-sym="${esc(s)}" value="${esc(S.symVals[s])}"></span>`).join('');
}
function bindSymbols() {
  $('symList').addEventListener('change', e => {
    const s = e.target.dataset.sym;
    if (!s) return;
    const v = readNumber(e.target.value);
    e.target.classList.toggle('bad', v === null);
    if (v === null) return;
    S.symVals[s] = v;
    if (S.result && S.result.ok) evaluateResult(S.result, S.symVals);
    drawCanvas(); renderResults();
  });
}

/* ---------------------------------------------------------------
   9. Results (#result)
   --------------------------------------------------------------- */
function reactionLabel(r) {
  if (!r.dir) return 'M (CCW +)';
  if (r.label === 'H') return 'H (→ +)';
  if (r.label === 'V') return 'V (↑ +)';
  return `R (+ along ${fmtDec(rad2deg(Math.atan2(r.dir[1], r.dir[0])), 3)}°)`;
}
function td(lin, kind) {
  const l = Lin.from(lin).clean();
  const cls = l.isZero(1e-12) ? 'zero' : (l.isConst() ? '' : 'sym');
  return `<td class="${cls}">${esc(fmtL(l, kind))}</td>`;
}
function renderResults() {
  const box = $('result');
  const issues = S.issues || [];
  const errs = issues.filter(i => i.level === 'error'), warns = issues.filter(i => i.level === 'warn');
  let h = '';
  if (!S.model.members.length) { box.innerHTML = '<div class="summary warn">Draw members, supports and loads – or choose an example.</div>'; return; }
  if (errs.length) h += '<div class="summary err">' + errs.map(i => esc(i.msg)).join('<br>') + '</div>';
  if (warns.length) h += '<div class="summary warn">' + warns.map(i => esc(i.msg)).join('<br>') + '</div>';
  const res = S.result && S.result.ok ? S.result : null;
  if (!res) { box.innerHTML = h; return; }
  const sts = res.stats, r = res.reactions.length;
  const ns = sts.ns;
  const det = ns === 0 ? 'statically determinate' : (ns > 0 ? `statically indeterminate to degree ${ns}` : `n = ${ns} < 0 (check stability)`);
  h += `<div class="summary ok">Solved: ${sts.nMembers} members, ${sts.nNodes} nodes, ${r} reaction components – ${det}.</div>`;
  const wrap = (title, head, rows) => `<h4>${title}</h4><div class="res-wrap"><table class="res"><tr>${head.map(x => `<th>${x}</th>`).join('')}</tr>${rows.join('')}</table></div>`;

  h += wrap('Reactions', ['Node', 'Component', 'Value'],
    res.reactions.map(x => `<tr><td>${x.nodeId}</td><td>${esc(reactionLabel(x))}</td>${td(x.lin, x.dir ? 'force' : 'moment')}</tr>`));

  const mids = S.model.members.map(m => m.id).filter(id => res.members[id]);
  h += wrap('Member end forces', ['Member', 'End', 'N', 'V', 'M'],
    mids.flatMap(id => {
      const mr = res.members[id], m = memberById(S.model, id);
      return [0, 1].map(e => `<tr><td>${e ? '' : id}</td><td>${e ? m.b : m.a}</td>${td(mr.Nend[e], 'force')}${td(mr.Vend[e], 'force')}${td(mr.Mend[e], 'moment')}</tr>`);
    }));

  const at = c => c ? esc(fmtX(c.x)) : '';
  h += wrap('Extreme bending moments and deflections (x from the start node)', ['Member', 'M<sub>max</sub>', 'at x', 'M<sub>min</sub>', 'at x', '|v|<sub>max</sub>', 'at x'],
    mids.map(id => {
      const e = res.members[id].ext;
      return `<tr><td>${id}</td>${td(e.M.max.lin, 'moment')}<td>${at(e.M.max)}</td>${td(e.M.min.lin, 'moment')}<td>${at(e.M.min)}</td>${td(e.v.max.lin, 'defl')}<td>${at(e.v.max)}</td></tr>`;
    }));

  h += wrap('Node displacements', ['Node', 'u<sub>x</sub>', 'u<sub>y</sub>', 'θ (CCW +)'],
    res.nodeIds.map(id => { const u = res.nodes[id].u; return `<tr><td>${id}</td>${td(u[0], 'defl')}${td(u[1], 'defl')}${td(u[2], 'rot')}</tr>`; }));

  if (modelSymbols(S.model).length) h += '<div class="muted">Locations of extrema are found with the symbol plot values above.</div>';
  box.innerHTML = h;
}

/* ---------------------------------------------------------------
   10. Examples, view, undo, clear, JSON
   --------------------------------------------------------------- */
function fillExamples() {
  const sel = $('examples');
  const groups = [];
  EXAMPLES.forEach(e => { if (!groups.includes(e.group)) groups.push(e.group); });
  sel.innerHTML = '<option value="">— choose an example —</option>' + groups.map(g =>
    `<optgroup label="${esc(g)}">` + EXAMPLES.filter(e => e.group === g).map(e => `<option value="${e.id}">${esc(e.name)}</option>`).join('') + '</optgroup>').join('');
}
function openExample(id, push) {
  const def = exampleById(id);
  if (!def) return;
  if (push) histPush();
  setModel(normalizeModel(buildExample(def)));
  S.view.fitted = false;
  UI.draft = null; UI.symKey = '';
  commit({ fit: true });
  setStatus('Example: ' + def.name, 'ok');
}
function bindMisc() {
  $('examples').addEventListener('change', e => { if (e.target.value) openExample(e.target.value, true); });
  $('btnFit').addEventListener('click', () => drawCanvas({ fit: true }));
  $('btnUndo').addEventListener('click', undo);
  $('btnClear').addEventListener('click', () => {
    if (!S.model.nodes.length) return;
    histPush();
    const s = deepClone(st());
    setModel(newModel());
    S.model.settings = s;
    $('examples').value = '';
    UI.draft = null; UI.symKey = '';
    commit();
    setStatus('Model cleared (Ctrl+Z to undo).');
  });
  $('btnExport').addEventListener('click', () => {
    $('json').value = JSON.stringify(modelToJSON(S.model), null, 1);
    setStatus('Model exported to the text box.', 'ok');
  });
  $('btnImport').addEventListener('click', () => {
    try {
      const obj = JSON.parse($('json').value);
      if (!obj || !Array.isArray(obj.nodes) || !Array.isArray(obj.members)) throw new Error('nodes / members missing');
      const model = normalizeModel(modelFromJSON(obj));
      histPush();
      setModel(model);
      S.view.fitted = false; UI.symKey = '';
      commit({ fit: true });
      setStatus('Model imported.', 'ok');
    } catch (err) { setStatus('Import failed: ' + err.message, 'err'); }
  });
  $('btnDownload').addEventListener('click', () => {
    const blob = new Blob([JSON.stringify(modelToJSON(S.model), null, 1)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'beam-frame-model.json';
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  });
}

/* ---------------------------------------------------------------
   11. Boot
   --------------------------------------------------------------- */
function uiInit() {
  UI.canvas = $('cv');
  UI.hintEl = $('hint');
  const main = $('main');
  UI.statusEl = el('div', { id: 'status' });
  UI.legendEl = el('div', { id: 'legend', hidden: true });
  main.appendChild(UI.statusEl);
  main.appendChild(UI.legendEl);
  viewInit();
  bindSettings(); bindDisplay(); bindTools(); bindCanvas(); bindKeys();
  bindSelection(); bindSymbols(); bindMisc();
  fillExamples();
  window.addEventListener('resize', () => drawCanvas());
  $('examples').value = 'ss-q';
  openExample('ss-q', false);
  setTool('select');
}
document.addEventListener('DOMContentLoaded', uiInit);
