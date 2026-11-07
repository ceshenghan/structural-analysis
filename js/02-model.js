/* ============================================================
   Beam & Frame Analysis – 02-model.js
   Data model + global state S, undo history, supports, element
   mechanics (local stiffness, release condensation, consistent /
   fixed-end loads), editing helpers, (de)serialisation, validation
   and the example library.
   Depends on: 01-core.js
   ============================================================ */
'use strict';

/* ---------------------------------------------------------------
   1. Settings, model, global state
   --------------------------------------------------------------- */
function defaultSettings() {
  return {
    mode: 'sym',       // 'num' numeric | 'sym' symbolic (loads are symbols, results linear in them)
    frac: true,        // print coefficients as fractions
    symL: true,        // (sym) lengths are multiples of L → L-powers attached by dimensional analysis
    axial: 'rigid',    // 'rigid' (EA → ∞, enforced as constraints) | 'flex' (EA from members)
    EI: 1,             // reference EI  (num: kN·m²;  sym: the symbol EI). member.EI is a factor of it
    EA: 1,             // reference EA  (num: kN;    only used when axial === 'flex')
    nSamples: 101      // sampling points per member for diagrams
  };
}
function newModel() { return { title: '', settings: defaultSettings(), nodes: [], members: [] }; }

const S = {
  model: newModel(),
  result: null,          // filled by 03-solver
  issues: [],            // [{level:'error'|'warn', msg, ref:{type,id}}]
  sel: null,             // {type:'node'|'member', id}
  hover: null,           // same shape as sel
  tool: 'select',        // 'select' | 'node' | 'member' | 'pan'
  draft: null,           // {nodeId} first node of a member being drawn
  /* display switches – keys used by 04-draw.js and 05-ui.js */
  show: { grid: true, nodeLabels: true, memberLabels: true, loads: true, supports: true, reactions: true,
          diagrams: ['V', 'M'], deformed: true, values: true, scale: 1, defScale: 1, mAbs: false, snap: true },
  symVals: {},           // plot value of each symbol (default 1)
  view: { ox: 0, oy: 0, k: 60, fitted: false },   // screen = (ox + k·x, oy − k·y); k = px per model unit
  hist: { undo: [], redo: [], max: 100 },
  dirty: true
};

/* ---------------------------------------------------------------
   2. Supports and constraint rows
   --------------------------------------------------------------- */
/* dofs: 'x','y' global translations, 'n' translation normal to the (inclined) surface, 'r' rotation */
const SUPPORTS = {
  none:   { label: 'none',   dofs: [] },
  pin:    { label: 'pin',    dofs: ['x', 'y'] },
  roller: { label: 'roller', dofs: ['n'] },
  fixed:  { label: 'fixed',  dofs: ['x', 'y', 'r'] },
  guided: { label: 'guided (slider)', dofs: ['n', 'r'] }
};
SUPPORTS.slider = SUPPORTS.guided;          // old name, kept so that older JSON files still load
const SUPPORT_ORDER = ['none', 'pin', 'roller', 'fixed', 'guided'];

/* constraint rows of a node in its [ux, uy, rz] space.
   row.v·u = 0 ; the multiplier of the row is the reaction acting on the structure along v.
   node.ang = inclination of the support surface in degrees (0 = horizontal). */
function supportRows(node) {
  const sp = SUPPORTS[node.sup || 'none'];
  if (!sp) return [];
  let nx = -sind(node.ang || 0), ny = cosd(node.ang || 0);
  if (ny < -1e-12 || (Math.abs(ny) <= 1e-12 && nx < 0)) { nx = -nx; ny = -ny; }   // reaction + = up / right
  const rows = [];
  for (const d of sp.dofs) {
    if (d === 'x') rows.push({ v: [1, 0, 0], label: 'H', dir: [1, 0] });
    else if (d === 'y') rows.push({ v: [0, 1, 0], label: 'V', dir: [0, 1] });
    else if (d === 'n') {
      const label = Math.abs(nx) < 1e-9 ? 'V' : (Math.abs(ny) < 1e-9 ? 'H' : 'R');
      rows.push({ v: [nx, ny, 0], label, dir: [nx, ny] });
    } else if (d === 'r') rows.push({ v: [0, 0, 1], label: 'M', dir: null });
  }
  return rows;
}

/* ---------------------------------------------------------------
   3. Factories
   --------------------------------------------------------------- */
function newNode(x, y) { return { id: uid(), x, y, sup: 'none', ang: 0, hinge: false, loads: [] }; }
/* end releases: relA/relB moment (hinge), shrA/shrB shear (transverse slip), axA/axB axial (slides along member) */
function newMember(a, b) {
  return { id: uid(), a, b, EI: 1, EA: 1, relA: false, relB: false, shrA: false, shrB: false, axA: false, axB: false, loads: [] };
}

function _defaultVal(kind, mode) {
  if (mode === 'num') return 10;
  return kind === 'M' ? 'M0' : (kind === 'q' ? 'q' : 'P');
}
/* node load: {kind:'F', val, ang}  (ang = direction the force acts, deg, −90 = down)
              {kind:'M', val}       (CCW positive) */
function newNodeLoad(spec = {}) {
  const mode = spec.mode || (S.model && S.model.settings.mode);
  const kind = spec.kind === 'M' ? 'M' : 'F';
  const ld = { id: uid(), kind, val: Lin.from(spec.val !== undefined ? spec.val : _defaultVal(kind, mode)) };
  if (kind === 'F') ld.ang = spec.ang !== undefined ? spec.ang : -90;
  return ld;
}
/* member load:
   {kind:'F', val, t, ang, local}             point force at t∈[0,1] from end a
   {kind:'M', val, t}                         concentrated moment (CCW positive)
   {kind:'q', val, val2|null, t1, t2, ang, local, proj}
        linearly varying load val→val2 between t1 and t2 (val2 null → uniform)
        ang: direction of the load (deg); local=true → measured from member axis
        proj: intensity per unit horizontal projection instead of per unit length */
function newMemberLoad(spec = {}) {
  const mode = spec.mode || (S.model && S.model.settings.mode);
  const kind = spec.kind === 'F' ? 'F' : (spec.kind === 'M' ? 'M' : 'q');
  const ld = { id: uid(), kind, val: Lin.from(spec.val !== undefined ? spec.val : _defaultVal(kind, mode)) };
  if (kind === 'q') {
    ld.val2 = (spec.val2 === undefined || spec.val2 === null) ? null : Lin.from(spec.val2);
    ld.t1 = spec.t1 !== undefined ? spec.t1 : 0;
    ld.t2 = spec.t2 !== undefined ? spec.t2 : 1;
    ld.ang = spec.ang !== undefined ? spec.ang : -90;
    ld.local = !!spec.local;
    ld.proj = !!spec.proj;
  } else {
    ld.t = spec.t !== undefined ? spec.t : 0.5;
    if (kind === 'F') { ld.ang = spec.ang !== undefined ? spec.ang : -90; ld.local = !!spec.local; }
  }
  return ld;
}
function cloneLoad(ld) {
  const c = Object.assign({}, ld, { id: uid(), val: Lin.from(ld.val).clone() });
  if ('val2' in ld) c.val2 = ld.val2 ? Lin.from(ld.val2).clone() : null;
  return c;
}
function normAng(a) { a = ((a % 360) + 540) % 360 - 180; return a === -180 ? 180 : a; }

/* ---------------------------------------------------------------
   4. Look-up and editing helpers
   --------------------------------------------------------------- */
function nodeById(model, id) { return model.nodes.find(n => n.id === id) || null; }
function memberById(model, id) { return model.members.find(m => m.id === id) || null; }
function findMember(model, a, b) { return model.members.find(m => (m.a === a && m.b === b) || (m.a === b && m.b === a)) || null; }
function nodeMembers(model, nodeId) { return model.members.filter(m => m.a === nodeId || m.b === nodeId); }
function findNodeAt(model, x, y, tol = 1e-6) {
  return model.nodes.find(n => Math.abs(n.x - x) <= tol && Math.abs(n.y - y) <= tol) || null;
}
/* member whose interior passes through (x,y); returns {m, t} or null */
function findMemberThrough(model, x, y, tol = 1e-6) {
  for (const m of model.members) {
    const A = nodeById(model, m.a), B = nodeById(model, m.b);
    const d = distToSegment(x, y, A.x, A.y, B.x, B.y);
    const L = hypot(B.x - A.x, B.y - A.y);
    if (d.d <= tol * Math.max(1, L) && d.t > 1e-9 && d.t < 1 - 1e-9) return { m, t: d.t };
  }
  return null;
}
function modelBounds(model) {
  if (!model.nodes.length) return { minx: 0, maxx: 1, miny: 0, maxy: 1 };
  let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
  for (const n of model.nodes) { minx = Math.min(minx, n.x); maxx = Math.max(maxx, n.x); miny = Math.min(miny, n.y); maxy = Math.max(maxy, n.y); }
  return { minx, maxx, miny, maxy };
}

/* next free id within a list (nodes and members are numbered 1, 2, 3 … separately) */
function nextId(list) { let mx = 0; for (const o of list) mx = Math.max(mx, +o.id || 0); return mx + 1; }
/* add a node (or return the coincident existing one) */
function addNode(model, x, y) {
  const ex = findNodeAt(model, x, y);
  if (ex) return ex;
  const n = newNode(x, y);
  n.id = nextId(model.nodes);
  model.nodes.push(n);
  return n;
}
/* add a node; if it falls on the interior of a member that member is split */
function placeNode(model, x, y) {
  const ex = findNodeAt(model, x, y);
  if (ex) return ex;
  const hit = findMemberThrough(model, x, y);
  if (hit) return splitMemberAt(model, hit.m, hit.t).node;
  return addNode(model, x, y);
}
function moveNode(model, id, x, y) {
  const n = nodeById(model, id);
  if (!n) return false;
  const other = findNodeAt(model, x, y);
  if (other && other.id !== id) return false;
  n.x = x; n.y = y;
  return true;
}
/* add member(s) a→b; nodes lying on the segment split it into a chain.
   returns the array of created members */
function addMember(model, aId, bId, opts = {}) {
  if (aId === bId) return [];
  const A = nodeById(model, aId), B = nodeById(model, bId);
  if (!A || !B) return [];
  const L = hypot(B.x - A.x, B.y - A.y);
  if (L < 1e-9) return [];
  let chain = [aId];
  if (opts.split !== false) {
    const inter = [];
    for (const n of model.nodes) {
      if (n.id === aId || n.id === bId) continue;
      const d = distToSegment(n.x, n.y, A.x, A.y, B.x, B.y);
      if (d.d <= 1e-6 * Math.max(1, L) && d.t > 1e-9 && d.t < 1 - 1e-9) inter.push({ id: n.id, t: d.t });
    }
    inter.sort((p, q) => p.t - q.t);
    chain = chain.concat(inter.map(p => p.id));
  }
  chain.push(bId);
  const made = [];
  for (let i = 0; i + 1 < chain.length; i++) {
    const a = chain[i], b = chain[i + 1];
    if (findMember(model, a, b)) continue;
    const m = newMember(a, b);
    m.id = nextId(model.members);
    if (opts.props) Object.assign(m, opts.props);
    model.members.push(m);
    made.push(m);
  }
  return made;
}
function deleteMember(model, id) {
  model.members = model.members.filter(m => m.id !== id);
}
function deleteNode(model, id) {
  model.members = model.members.filter(m => m.a !== id && m.b !== id);
  model.nodes = model.nodes.filter(n => n.id !== id);
}
/* split member m at parameter t (0<t<1); loads are redistributed. returns {node, m1, m2} */
function splitMemberAt(model, m, t, node) {
  const A = nodeById(model, m.a), B = nodeById(model, m.b);
  t = clamp(t, 0, 1);
  if (t < 1e-9 || t > 1 - 1e-9) return null;
  if (!node) node = addNode(model, A.x + t * (B.x - A.x), A.y + t * (B.y - A.y));
  const m2 = newMember(node.id, m.b);
  m2.EI = m.EI; m2.EA = m.EA; m2.relB = m.relB; m2.shrB = m.shrB; m2.axB = m.axB;
  m.b = node.id; m.relB = false; m.shrB = false; m.axB = false;
  const loads1 = [], loads2 = [];
  for (const ld of m.loads) {
    if (ld.kind === 'q') {
      const t1 = Math.min(ld.t1, ld.t2), t2 = Math.max(ld.t1, ld.t2);
      const vA = ld.val, vB = ld.val2 || ld.val;
      const valAt = s => ld.val2 ? vA.add(vB.sub(vA).scale((s - t1) / (t2 - t1 || 1))) : vA;
      if (t1 < t - 1e-12) {
        const q = cloneLoad(ld);
        const e = Math.min(t2, t);
        q.t1 = t1 / t; q.t2 = e / t;
        q.val = valAt(t1); q.val2 = ld.val2 ? valAt(e) : null;
        loads1.push(q);
      }
      if (t2 > t + 1e-12) {
        const q = cloneLoad(ld);
        const s1 = Math.max(t1, t);
        q.t1 = (s1 - t) / (1 - t); q.t2 = (t2 - t) / (1 - t);
        q.val = valAt(s1); q.val2 = ld.val2 ? valAt(t2) : null;
        loads2.push(q);
      }
    } else {
      const q = cloneLoad(ld);
      if (ld.t <= t) { q.t = ld.t / t; loads1.push(q); }
      else { q.t = (ld.t - t) / (1 - t); loads2.push(q); }
    }
  }
  m.loads = loads1; m2.loads = loads2;
  m2.id = nextId(model.members);
  model.members.push(m2);
  return { node, m1: m, m2 };
}
/* reverse the local axis of a member (a↔b) keeping the physics unchanged */
function flipMember(model, m) {
  const a = m.a; m.a = m.b; m.b = a;
  for (const [p, q] of [['relA', 'relB'], ['shrA', 'shrB'], ['axA', 'axB']]) { const r = m[p]; m[p] = m[q]; m[q] = r; }
  for (const ld of m.loads) {
    if (ld.kind === 'q') {
      const t1 = 1 - ld.t2, t2 = 1 - ld.t1;
      ld.t1 = t1; ld.t2 = t2;
      if (ld.val2) { const v = ld.val; ld.val = ld.val2; ld.val2 = v; }
    } else ld.t = 1 - ld.t;
    if (ld.local && ld.ang !== undefined) ld.ang = normAng(ld.ang + 180);
  }
}
/* merge the two collinear members meeting at a plain node (no support/hinge/load/release) */
function mergeAtNode(model, nodeId) {
  const n = nodeById(model, nodeId);
  if (!n) return false;
  const ms = nodeMembers(model, nodeId);
  if (ms.length !== 2 || (n.sup && n.sup !== 'none') || n.hinge || n.loads.length) return false;
  const [m1, m2] = ms;
  const A = nodeById(model, m1.a === nodeId ? m1.b : m1.a);
  const C = nodeById(model, m2.a === nodeId ? m2.b : m2.a);
  if (!collinear(A.x, A.y, n.x, n.y, C.x, C.y)) return false;
  if ((n.x - A.x) * (C.x - n.x) + (n.y - A.y) * (C.y - n.y) <= 0) return false;   // n must lie between
  if (Math.abs(m1.EI - m2.EI) > 1e-12 || Math.abs(m1.EA - m2.EA) > 1e-12) return false;
  if (m1.b !== nodeId) flipMember(model, m1);   // m1: A → n
  if (m2.a !== nodeId) flipMember(model, m2);   // m2: n → C
  if (m1.relB || m2.relA || m1.shrB || m2.shrA || m1.axB || m2.axA) return false;
  const L1 = hypot(n.x - A.x, n.y - A.y), L2 = hypot(C.x - n.x, C.y - n.y), L = L1 + L2;
  for (const ld of m1.loads) {
    if (ld.kind === 'q') { ld.t1 *= L1 / L; ld.t2 *= L1 / L; } else ld.t *= L1 / L;
  }
  for (const ld of m2.loads) {
    if (ld.kind === 'q') { ld.t1 = (L1 + ld.t1 * L2) / L; ld.t2 = (L1 + ld.t2 * L2) / L; }
    else ld.t = (L1 + ld.t * L2) / L;
    m1.loads.push(ld);
  }
  m1.b = m2.b; m1.relB = m2.relB; m1.shrB = m2.shrB; m1.axB = m2.axB;
  model.members = model.members.filter(m => m.id !== m2.id);
  model.nodes = model.nodes.filter(x => x.id !== nodeId);
  return true;
}

/* ---------------------------------------------------------------
   5. Symbols / load columns
   --------------------------------------------------------------- */
function coefOf(v, key) { return Lin.from(v).get(key); }

/* all symbols used by the loads of the model, in display order */
function modelSymbols(model) {
  const set = new Set();
  const take = v => { if (v) Lin.from(v).symbols().forEach(s => set.add(s)); };
  model.nodes.forEach(n => n.loads.forEach(l => take(l.val)));
  model.members.forEach(m => m.loads.forEach(l => { take(l.val); take(l.val2); }));
  return Array.from(set).sort(_symOrder);
}
/* true when some load has a numeric (constant) part */
function modelHasConst(model) {
  const has = v => v && Math.abs(Lin.from(v).get(CONST_KEY)) > 0;
  return model.nodes.some(n => n.loads.some(l => has(l.val))) ||
         model.members.some(m => m.loads.some(l => has(l.val) || has(l.val2)));
}
/* load "columns": keys of the right-hand sides to be solved for */
function loadColumns(model) {
  const cols = modelSymbols(model);
  if (modelHasConst(model) || cols.length === 0) cols.unshift(CONST_KEY);
  return cols;
}

/* ---------------------------------------------------------------
   6. Element mechanics  (local DOF order: [u_a, v_a, θ_a, u_b, v_b, θ_b])
      local x along a→b, local y = x rotated +90° (CCW)
   --------------------------------------------------------------- */
function memberGeom(model, m) {
  const A = nodeById(model, m.a), B = nodeById(model, m.b);
  const dx = B.x - A.x, dy = B.y - A.y, L = hypot(dx, dy);
  return { A, B, dx, dy, L, c: L ? dx / L : 1, s: L ? dy / L : 0 };
}
/* Hermite / linear shape functions at x */
function shapeFns(x, L) {
  const xi = x / L, xi2 = xi * xi, xi3 = xi2 * xi;
  return {
    u1: 1 - xi, u2: xi,
    v1: 1 - 3 * xi2 + 2 * xi3, t1: L * (xi - 2 * xi2 + xi3),
    v2: 3 * xi2 - 2 * xi3,     t2: L * (-xi2 + xi3)
  };
}
/* d/dx of the transverse shape functions */
function shapeDer(x, L) {
  const xi = x / L, xi2 = xi * xi;
  return {
    v1: (-6 * xi + 6 * xi2) / L, t1: 1 - 4 * xi + 3 * xi2,
    v2: (6 * xi - 6 * xi2) / L,  t2: -2 * xi + 3 * xi2
  };
}
/* Euler–Bernoulli frame element, local coordinates (EA = 0 → axial rows empty) */
function kLocal(EA, EI, L) {
  const K = zeros(6, 6);
  const a = EA / L, b = 12 * EI / (L * L * L), c = 6 * EI / (L * L), d = 4 * EI / L, e = 2 * EI / L;
  K[0][0] = a;  K[0][3] = -a; K[3][0] = -a; K[3][3] = a;
  K[1][1] = b;  K[1][2] = c;  K[1][4] = -b; K[1][5] = c;
  K[2][1] = c;  K[2][2] = d;  K[2][4] = -c; K[2][5] = e;
  K[4][1] = -b; K[4][2] = -c; K[4][4] = b;  K[4][5] = -c;
  K[5][1] = c;  K[5][2] = e;  K[5][4] = -c; K[5][5] = d;
  return K;
}
/* transformation  u_local = T · u_global */
function tMatrix(c, s) {
  const T = zeros(6, 6);
  T[0][0] = c; T[0][1] = s; T[1][0] = -s; T[1][1] = c; T[2][2] = 1;
  T[3][3] = c; T[3][4] = s; T[4][3] = -s; T[4][4] = c; T[5][5] = 1;
  return T;
}

/* static condensation of released local DOFs (rel: 6 booleans).
   Kc  – 6×6 condensed matrix (released rows/cols zero)
   condenseLoad(fe) – condensed consistent load (released entries zero)
   recover(u, fe)   – fills the released entries of a local displacement vector */
function makeCondenser(K, rel) {
  /* released DOFs with zero stiffness (axial DOFs of an axially rigid member, EA = 0) cannot be
     condensed; they are handled by rigid transfer: the axial load at a released end is carried
     to the other end and the released end follows the other end (no stretching). */
  const AX = { 0: 3, 3: 0 };
  const k = [], rn = [], rz = [];
  rel.forEach((f, i) => {
    if (!f) k.push(i);
    else if (K[i].every(v => v === 0)) rz.push(i);
    else rn.push(i);
  });
  const r = rn.concat(rz);
  const Kc = copyMat(K);
  const zeroRC = idx => idx.forEach(i => { for (let j = 0; j < 6; j++) { Kc[i][j] = 0; Kc[j][i] = 0; } });
  const transfer = fe => {
    const out = fe.slice();
    rz.forEach(i => { const j = AX[i]; if (j !== undefined && !rel[j]) out[j] += out[i]; out[i] = 0; });
    return out;
  };
  const fillRz = out => { rz.forEach(i => { const j = AX[i]; if (j !== undefined && !rel[j]) out[i] = out[j]; }); return out; };
  if (rn.length === 0) {
    zeroRC(rz);
    return { Kc, r, k, rz, condenseLoad: fe => transfer(fe), recover: u => fillRz(u.slice()) };
  }
  const Krr = rn.map(i => rn.map(j => K[i][j]));
  const Krk = rn.map(i => k.map(j => K[i][j]));
  const Kkr = k.map(i => rn.map(j => K[i][j]));
  const Kinv = invert(Krr);
  if (!Kinv) {                     // degenerate (e.g. shear released at both ends): drop the released DOFs
    zeroRC(r);
    return {
      Kc, r, k, rz, degenerate: true,
      condenseLoad: fe => transfer(fe).map((v, i) => (rel[i] ? 0 : v)),
      recover: u => fillRz(u.slice())
    };
  }
  const G = matmul(Kinv, Krk);           // rn×k
  const KkrG = matmul(Kkr, G);           // k×k
  const KkrKinv = matmul(Kkr, Kinv);     // k×rn
  k.forEach((i, a) => k.forEach((j, b) => { Kc[i][j] = K[i][j] - KkrG[a][b]; }));
  zeroRC(r);
  return {
    Kc, r, k, rz,
    condenseLoad(fe0) {
      const fe = transfer(fe0);
      const corr = matvec(KkrKinv, rn.map(i => fe[i]));
      const out = fe.slice();
      k.forEach((i, a) => { out[i] = fe[i] - corr[a]; });
      r.forEach(i => { out[i] = 0; });
      return out;
    },
    recover(u, fe) {
      const uk = k.map(i => u[i]);
      const fr = rn.map(i => (fe ? fe[i] : 0));
      const ur = vadd(matvec(Kinv, fr), matvec(G, uk), -1);
      const out = u.slice();
      rn.forEach((i, a) => { out[i] = ur[a]; });
      return fillRz(out);
    }
  };
}
/* everything the solver needs about one member */
function memberSystem(model, m) {
  const g = memberGeom(model, m);
  const st = model.settings;
  const flex = st.axial === 'flex';
  const EI = (m.EI || 1) * (st.EI || 1);
  const EA = flex ? (m.EA || 1) * (st.EA || 1) : 0;
  /* local DOFs [u_a, v_a, θ_a, u_b, v_b, θ_b]: axial, shear and moment releases */
  const rel = [!!m.axA, !!m.shrA, !!(m.relA || g.A.hinge), !!m.axB, !!m.shrB, !!(m.relB || g.B.hinge)];
  const axRel = !!(m.axA || m.axB);
  const Kl = kLocal(EA, EI, g.L);
  const cond = makeCondenser(Kl, rel);
  const T = tMatrix(g.c, g.s);
  const Tt = transpose(T);
  const Kg = matmul(Tt, matmul(cond.Kc, T));
  return {
    m, A: g.A, B: g.B, L: g.L, c: g.c, s: g.s,
    EI, EA, flex, rigid: !flex && !axRel,   // an axial release removes the rigid-axial constraint
    rel, T, Tt, Kl, Kc: cond.Kc, Kg, cond,
    /* rigid-axial constraint row on global [a: ux,uy,rz | b: ux,uy,rz]; its multiplier = axial force N (tension +) */
    rigidRow: [g.c, g.s, 0, -g.c, -g.s, 0],
    toLocal(ug) { return matvec(T, ug); },
    toGlobal(ul) { return matvec(Tt, ul); }
  };
}

/* unit direction of a load in global coordinates (for drawing) */
/* cos / sin of an angle in degrees without round-off residue (cos 90° = 0 exactly) */
function cosd(d) { const v = Math.cos(deg2rad(d)); return Math.abs(v) < 1e-15 ? 0 : v; }
function sind(d) { const v = Math.sin(deg2rad(d)); return Math.abs(v) < 1e-15 ? 0 : v; }
function loadDir(ld, sys) {
  const ang = ld.ang !== undefined ? ld.ang : -90;
  const cx = cosd(ang), cy = sind(ang);
  if (ld.local && sys) return [sys.c * cx - sys.s * cy, sys.s * cx + sys.c * cy];
  return [cx, cy];
}
/* components of a load of magnitude `mag` in member-local axes */
function _dirLocal(ld, sys, mag) {
  const ang = ld.ang !== undefined ? ld.ang : -90;
  const cx = cosd(ang) * mag, cy = sind(ang) * mag;
  if (ld.local) return [cx, cy];
  const snap = v => (Math.abs(v) < 1e-14 * Math.abs(mag) ? 0 : v);
  return [snap(cx * sys.c + cy * sys.s), snap(-cx * sys.s + cy * sys.c)];
}

/* numeric span loads (local axes) of one member for load column `key`:
   {kind:'F', x, px, py} | {kind:'M', x, m} | {kind:'q', x1, x2, px1, py1, px2, py2}  */
function memberLocalLoads(m, sys, key) {
  const out = [], L = sys.L;
  for (const ld of m.loads) {
    if (ld.kind === 'F') {
      const P = coefOf(ld.val, key);
      if (!P) continue;
      const [px, py] = _dirLocal(ld, sys, P);
      out.push({ kind: 'F', x: clamp(ld.t, 0, 1) * L, px, py, ref: ld });
    } else if (ld.kind === 'M') {
      const M = coefOf(ld.val, key);
      if (!M) continue;
      out.push({ kind: 'M', x: clamp(ld.t, 0, 1) * L, m: M, ref: ld });
    } else if (ld.kind === 'q') {
      const q1 = coefOf(ld.val, key);
      const q2 = (ld.val2 === null || ld.val2 === undefined) ? q1 : coefOf(ld.val2, key);
      if (!q1 && !q2) continue;
      const t1 = clamp(Math.min(ld.t1, ld.t2), 0, 1), t2 = clamp(Math.max(ld.t1, ld.t2), 0, 1);
      if (t2 - t1 < 1e-12) continue;
      const f = ld.proj ? Math.abs(sys.c) : 1;      // per unit projected length → per unit member length
      const [cx, cy] = _dirLocal(ld, sys, f);
      out.push({ kind: 'q', x1: t1 * L, x2: t2 * L, px1: cx * q1, py1: cy * q1, px2: cx * q2, py2: cy * q2, ref: ld });
    }
  }
  return out;
}

/* consistent (fixed-end) nodal loads in local axes for the span loads of a fixed–fixed member.
   Element equation:  K u = f_nodal + fe */
function fixedEndLocal(loads, L) {
  const fe = zeros(6);
  for (const ld of loads) {
    if (ld.kind === 'F') {
      const N = shapeFns(ld.x, L);
      fe[0] += N.u1 * ld.px; fe[1] += N.v1 * ld.py; fe[2] += N.t1 * ld.py;
      fe[3] += N.u2 * ld.px; fe[4] += N.v2 * ld.py; fe[5] += N.t2 * ld.py;
    } else if (ld.kind === 'M') {
      const D = shapeDer(ld.x, L);
      fe[1] += D.v1 * ld.m; fe[2] += D.t1 * ld.m; fe[4] += D.v2 * ld.m; fe[5] += D.t2 * ld.m;
    } else if (ld.kind === 'q') {
      const span = ld.x2 - ld.x1;
      const v = integrateVec(x => {
        const N = shapeFns(x, L), s = (x - ld.x1) / span;
        const px = ld.px1 + (ld.px2 - ld.px1) * s, py = ld.py1 + (ld.py2 - ld.py1) * s;
        return [N.u1 * px, N.v1 * py, N.t1 * py, N.u2 * px, N.v2 * py, N.t2 * py];
      }, ld.x1, ld.x2, 5);
      for (let i = 0; i < 6; i++) fe[i] += v[i];
    }
  }
  return fe;
}
/* nodal load vector [Fx, Fy, M] of a node for column `key` */
function nodeLoadVector(node, key) {
  const f = [0, 0, 0];
  for (const ld of node.loads) {
    const v = coefOf(ld.val, key);
    if (!v) continue;
    if (ld.kind === 'F') { const a = ld.ang !== undefined ? ld.ang : -90; f[0] += v * cosd(a); f[1] += v * sind(a); }
    else f[2] += v;
  }
  return f;
}

/* ---------------------------------------------------------------
   7. Validation
   --------------------------------------------------------------- */
function validateModel(model) {
  const issues = [];
  const err = (msg, ref) => issues.push({ level: 'error', msg, ref });
  const warn = (msg, ref) => issues.push({ level: 'warn', msg, ref });
  const st = model.settings;

  if (!model.nodes.length || !model.members.length) { err('The model is empty: draw at least one member.'); return issues; }

  const used = new Set();
  model.members.forEach(m => {
    const A = nodeById(model, m.a), B = nodeById(model, m.b);
    if (!A || !B) { err(`Member ${m.id} refers to a missing node.`, { type: 'member', id: m.id }); return; }
    used.add(m.a); used.add(m.b);
    if (hypot(B.x - A.x, B.y - A.y) < 1e-9) err(`Member ${m.id} has zero length.`, { type: 'member', id: m.id });
    if (m.shrA && m.shrB) err(`Member ${m.id}: shear releases at both ends make it a mechanism.`, { type: 'member', id: m.id });
    if (m.axA && m.axB) err(`Member ${m.id}: axial releases at both ends make it a mechanism.`, { type: 'member', id: m.id });
    if (!(m.EI > 0)) err(`Member ${m.id}: EI must be positive.`, { type: 'member', id: m.id });
    if (st.axial === 'flex' && !(m.EA > 0)) err(`Member ${m.id}: EA must be positive.`, { type: 'member', id: m.id });
    m.loads.forEach(ld => {
      if (ld.kind === 'q') {
        if (ld.t1 < -1e-9 || ld.t2 > 1 + 1e-9 || Math.abs(ld.t2 - ld.t1) < 1e-12) err(`Member ${m.id}: invalid range of a distributed load.`, { type: 'member', id: m.id });
      } else if (ld.t < -1e-9 || ld.t > 1 + 1e-9) err(`Member ${m.id}: a concentrated load lies outside the member.`, { type: 'member', id: m.id });
      if (st.mode === 'sym' && st.symL) _checkSymbolKinds(ld, `Member ${m.id}`, { type: 'member', id: m.id }, warn);
    });
  });
  for (let i = 0; i < model.members.length; i++)
    for (let j = i + 1; j < model.members.length; j++) {
      const p = model.members[i], q = model.members[j];
      if ((p.a === q.a && p.b === q.b) || (p.a === q.b && p.b === q.a)) warn(`Members ${p.id} and ${q.id} coincide.`, { type: 'member', id: q.id });
    }

  let nSup = 0;
  model.nodes.forEach(n => {
    if (!used.has(n.id)) warn(`Node ${n.id} is not connected to any member and is ignored.`, { type: 'node', id: n.id });
    const rows = supportRows(n);
    nSup += rows.length;
    if (n.hinge && n.loads.some(l => l.kind === 'M') && !rows.some(r => r.label === 'M'))
      err(`Node ${n.id} is a hinge and cannot take a concentrated moment.`, { type: 'node', id: n.id });
    if (n.hinge && nodeMembers(model, n.id).length < 2 && n.sup !== 'fixed' && n.sup !== 'slider')
      warn(`The hinge at node ${n.id} (single member end) has no effect.`, { type: 'node', id: n.id });
    if (st.mode === 'sym' && st.symL) n.loads.forEach(ld => _checkSymbolKinds(ld, `Node ${n.id}`, { type: 'node', id: n.id }, warn));
  });
  if (nSup === 0) err('The model has no supports.');
  else if (nSup < 3) err('Fewer than 3 support reactions: the structure is unstable (mechanism).');

  if (!(st.EI > 0)) err('Reference EI must be positive.');
  return issues;
}
/* symbol/kind mismatch produces wrong powers of L in symbolic output */
function _checkSymbolKinds(ld, where, ref, warn) {
  const want = ld.kind === 'q' ? 'dist' : (ld.kind === 'M' ? 'moment' : 'force');
  const names = { dist: 'a distributed load (q…/w…)', moment: 'a moment (M…/C…)', force: 'a force' };
  const vals = [ld.val, ld.val2].filter(v => v);
  for (const v of vals)
    for (const s of Lin.from(v).symbols())
      if (symKind(s) !== want)
        warn(`${where}: symbol “${s}” is treated as ${names[symKind(s)]} but the load is ${names[want]}; the powers of L in the results may be wrong.`, ref);
}

/* ---------------------------------------------------------------
   8. Serialisation and undo history
   --------------------------------------------------------------- */
function modelToJSON(model) { return JSON.parse(JSON.stringify(model)); }   // Lin → {c:{…}}

function modelFromJSON(src) {
  const obj = typeof src === 'string' ? JSON.parse(src) : src;
  const model = newModel();
  Object.assign(model.settings, obj.settings || {});
  model.title = obj.title || '';
  let maxId = 0;
  const reviveLoad = l => {
    const ld = Object.assign({}, l);
    ld.val = Lin.from(l.val);
    if ('val2' in l) ld.val2 = (l.val2 === null || l.val2 === undefined) ? null : Lin.from(l.val2);
    if (!ld.id) ld.id = uid();
    maxId = Math.max(maxId, ld.id);
    return ld;
  };
  model.nodes = (obj.nodes || []).map(n => {
    const nd = Object.assign(newNode(0, 0), n);
    nd.loads = (n.loads || []).map(reviveLoad);
    maxId = Math.max(maxId, nd.id);
    return nd;
  });
  model.members = (obj.members || []).map(m => {
    const mb = Object.assign(newMember(0, 0), m);
    mb.loads = (m.loads || []).map(reviveLoad);
    maxId = Math.max(maxId, mb.id);
    return mb;
  });
  uidReset(maxId);
  return model;
}

function histPush() {
  const h = S.hist;
  h.undo.push(JSON.stringify(S.model));
  if (h.undo.length > h.max) h.undo.shift();
  h.redo.length = 0;
}
function histUndo() {
  const h = S.hist;
  if (!h.undo.length) return false;
  h.redo.push(JSON.stringify(S.model));
  S.model = modelFromJSON(h.undo.pop());
  S.sel = null; S.hover = null; S.draft = null; S.result = null; S.dirty = true;
  return true;
}
function histRedo() {
  const h = S.hist;
  if (!h.redo.length) return false;
  h.undo.push(JSON.stringify(S.model));
  S.model = modelFromJSON(h.redo.pop());
  S.sel = null; S.hover = null; S.draft = null; S.result = null; S.dirty = true;
  return true;
}
function setModel(model) {
  S.model = model;
  S.sel = null; S.hover = null; S.draft = null; S.result = null; S.dirty = true;
  S.symVals = {};
}

/* ---------------------------------------------------------------
   9. Example library
   --------------------------------------------------------------- */
const EXAMPLES = [
  { id: 'ss-q', group: 'Beams', name: 'Simply supported beam · UDL q',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 'q' }] }] },
  { id: 'ss-P', group: 'Beams', name: 'Simply supported beam · midspan P',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'F', val: 'P', t: 0.5 }] }] },
  { id: 'ss-M', group: 'Beams', name: 'Simply supported beam · end moment M0',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1 }],
    nodeLoads: [{ n: 1, kind: 'M', val: 'M0' }] },
  { id: 'cant-P', group: 'Beams', name: 'Cantilever · tip load P',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 1, y: 0 }],
    members: [{ a: 0, b: 1 }],
    nodeLoads: [{ n: 1, kind: 'F', val: 'P' }] },
  { id: 'cant-q', group: 'Beams', name: 'Cantilever · UDL q',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 1, y: 0 }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 'q' }] }] },
  { id: 'overhang', group: 'Beams', name: 'Overhanging beam · q and tip P',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }, { x: 1.5, y: 0 }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 'q' }] }, { a: 1, b: 2 }],
    nodeLoads: [{ n: 2, kind: 'F', val: 'P' }] },
  { id: 'propped', group: 'Beams', name: 'Propped cantilever · UDL q (indeterminate)',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 1, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 'q' }] }] },
  { id: 'ff-P', group: 'Beams', name: 'Fixed–fixed beam · midspan P',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 1, y: 0, sup: 'fixed' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'F', val: 'P', t: 0.5 }] }] },
  { id: 'cont2', group: 'Beams', name: 'Two-span continuous beam · q',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }, { x: 2, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 'q' }] }, { a: 1, b: 2, loads: [{ kind: 'q', val: 'q' }] }] },
  { id: 'tri', group: 'Beams', name: 'Simply supported beam · triangular load 0→q',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 1, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 0, val2: 'q' }] }] },
  { id: 'l-frame', group: 'Frames', name: 'L-shaped cantilever frame · P',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 0, y: 1 }, { x: 1, y: 1 }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2 }],
    nodeLoads: [{ n: 2, kind: 'F', val: 'P' }] },
  { id: 'portal', group: 'Frames', name: 'Fixed portal frame · lateral P and q',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 0, y: 1 }, { x: 1, y: 1 }, { x: 1, y: 0, sup: 'fixed' }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2, loads: [{ kind: 'q', val: 'q' }] }, { a: 2, b: 3 }],
    nodeLoads: [{ n: 1, kind: 'F', val: 'P', ang: 0 }] },
  { id: 'pin-portal', group: 'Frames', name: 'Two-hinged portal frame · q',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 0, y: 1 }, { x: 1, y: 1 }, { x: 1, y: 0, sup: 'pin' }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2, loads: [{ kind: 'q', val: 'q' }] }, { a: 2, b: 3 }] },
  { id: 'three-hinge', group: 'Frames', name: 'Three-hinged frame · q (determinate)',
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 0, y: 1 }, { x: 0.5, y: 1, hinge: true }, { x: 1, y: 1 }, { x: 1, y: 0, sup: 'pin' }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2, loads: [{ kind: 'q', val: 'q' }] }, { a: 2, b: 3, loads: [{ kind: 'q', val: 'q' }] }, { a: 3, b: 4 }] },
  { id: 'gable', group: 'Frames', name: 'Gable frame · q on rafters (per horizontal length)',
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 0, y: 1 }, { x: 1, y: 1.5 }, { x: 2, y: 1 }, { x: 2, y: 0, sup: 'fixed' }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2, loads: [{ kind: 'q', val: 'q', proj: true }] }, { a: 2, b: 3, loads: [{ kind: 'q', val: 'q', proj: true }] }, { a: 3, b: 4 }] },
  { id: 'num-incl', group: 'Numerical (kN, m)', name: 'Beam on an inclined roller',
    settings: { mode: 'num', frac: false, symL: false, EI: 2e4 },
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 6, y: 0, sup: 'roller', ang: 30 }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'F', val: 10, t: 1 / 3 }, { kind: 'q', val: 4 }] }] },
  { id: 'num-cont', group: 'Numerical (kN, m)', name: 'Two-span continuous beam',
    settings: { mode: 'num', frac: false, symL: false, EI: 2e4 },
    nodes: [{ x: 0, y: 0, sup: 'pin' }, { x: 4, y: 0, sup: 'roller' }, { x: 10, y: 0, sup: 'roller' }],
    members: [{ a: 0, b: 1, loads: [{ kind: 'q', val: 10 }] }, { a: 1, b: 2, loads: [{ kind: 'q', val: 10 }, { kind: 'F', val: 20, t: 0.5 }] }] },
  { id: 'num-frame', group: 'Numerical (kN, m)', name: 'Single-storey frame',
    settings: { mode: 'num', frac: false, symL: false, EI: 3e4 },
    nodes: [{ x: 0, y: 0, sup: 'fixed' }, { x: 0, y: 4 }, { x: 6, y: 4 }, { x: 6, y: 0, sup: 'fixed' }],
    members: [{ a: 0, b: 1 }, { a: 1, b: 2, loads: [{ kind: 'q', val: 12 }] }, { a: 2, b: 3 }],
    nodeLoads: [{ n: 1, kind: 'F', val: 15, ang: 0 }] }
];

function buildExample(def) {
  const model = newModel();
  Object.assign(model.settings, def.settings || {});
  model.title = def.name;
  const mode = model.settings.mode;
  const ids = def.nodes.map(n => {
    const nd = addNode(model, n.x, n.y);
    if (n.sup) nd.sup = n.sup;
    if (n.ang) nd.ang = n.ang;
    if (n.hinge) nd.hinge = true;
    return nd.id;
  });
  def.members.forEach(mm => {
    const made = addMember(model, ids[mm.a], ids[mm.b], { split: false });
    const m = made[0];
    if (!m) return;
    ['EI', 'EA', 'relA', 'relB'].forEach(k => { if (mm[k] !== undefined) m[k] = mm[k]; });
    (mm.loads || []).forEach(l => m.loads.push(newMemberLoad(Object.assign({ mode }, l))));
  });
  (def.nodeLoads || []).forEach(l => nodeById(model, ids[l.n]).loads.push(newNodeLoad(Object.assign({ mode }, l))));
  return model;
}
function exampleById(id) { return EXAMPLES.find(e => e.id === id) || null; }
function loadExample(id) {
  const def = exampleById(id);
  if (!def) return false;
  histPush();
  setModel(buildExample(def));
  return true;
}