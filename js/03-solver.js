/* ============================================================
   Beam & Frame Analysis – 03-solver.js
   Direct-stiffness solver.
     • DOF numbering (3 per node used by members)
     • assembly of K and of one load vector per load column
     • constraints (supports, rigid-axial rows, zero-stiffness DOFs)
       handled by a null-space method:  u = Z q,  (ZᵀKZ) q = Zᵀf
     • Lagrange multipliers  Cᵀλ = K u − f  → reactions / axial forces
     • redundant rigid-axial constraints → EA→∞ limit fallback
     • exact internal-force fields N(x), V(x), M(x) by statics from the
       member end forces; deflections by double integration of M/EI
     • extrema (for the current symbol values) and summary tables
   Depends on: 01-core.js, 02-model.js
   ============================================================ */
'use strict';

const SOLVER = {
  tolRank: 1e-9,      // relative tolerance for dependent constraint rows
  tolPivot: 1e-12,    // relative pivot tolerance (singular ⇒ mechanism)
  tolZero: 1e-12,     // relative diagonal tolerance for zero-stiffness DOFs
  tolClean: 1e-9,     // relative threshold below which coefficients are printed as 0
  rigidRatio: 1e8     // EA·L²/EI used when the rigid-axial model is over-constrained
};

/* ---------------------------------------------------------------
   1. Private linear algebra
   --------------------------------------------------------------- */
function _svDot(a, b) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; }

/* Gauss–Jordan with partial pivoting; A n×n, B n×k → X n×k, or null when singular */
function _svGaussJordan(A, B) {
  const n = A.length;
  if (!n) return [];
  const k = B[0].length;
  const M = A.map((r, i) => r.concat(B[i]));
  let scale = 0;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) scale = Math.max(scale, Math.abs(M[i][j]));
  const tol = SOLVER.tolPivot * Math.max(scale, 1e-300);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) <= tol) return null;
    if (p !== c) { const t = M[p]; M[p] = M[c]; M[c] = t; }
    const piv = M[c][c];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / piv;
      if (!f) continue;
      for (let j = c; j < n + k; j++) M[r][j] -= f * M[c][j];
    }
  }
  return M.map((r, i) => { const piv = r[i]; return r.slice(n).map(v => v / piv); });
}

/* incremental RREF of a list of dense rows.
   returns { R (independent rows in reduced echelon form), piv (pivot column of each), indep, dep } */
function _svRowSpace(rows, n) {
  const R = [], piv = [], indep = [], dep = [];
  rows.forEach((row, idx) => {
    const v = row.slice();
    let norm0 = 0;
    for (let j = 0; j < n; j++) norm0 = Math.max(norm0, Math.abs(v[j]));
    if (!norm0) { dep.push(idx); return; }
    for (let i = 0; i < R.length; i++) {
      const f = v[piv[i]];
      if (f) for (let j = 0; j < n; j++) v[j] -= f * R[i][j];
    }
    let p = -1, best = 0;
    for (let j = 0; j < n; j++) if (Math.abs(v[j]) > best) { best = Math.abs(v[j]); p = j; }
    if (best <= SOLVER.tolRank * norm0) { dep.push(idx); return; }
    const inv = 1 / v[p];
    for (let j = 0; j < n; j++) v[j] *= inv;
    for (let i = 0; i < R.length; i++) {
      const f = R[i][p];
      if (f) for (let j = 0; j < n; j++) R[i][j] -= f * v[j];
    }
    R.push(v); piv.push(p); indep.push(idx);
  });
  return { R, piv, indep, dep };
}
/* basis of { u : R u = 0 } from the RREF rows → array of n-vectors (one per free column) */
function _svNullSpace(R, piv, n) {
  const isPiv = new Array(n).fill(false);
  piv.forEach(p => { isPiv[p] = true; });
  const Z = [];
  for (let j = 0; j < n; j++) {
    if (isPiv[j]) continue;
    const z = zeros(n);
    z[j] = 1;
    for (let i = 0; i < R.length; i++) z[piv[i]] = -R[i][j];
    Z.push(z);
  }
  return Z;
}

/* ---------------------------------------------------------------
   2. Result values: column coefficients → Lin, numeric evaluation
   --------------------------------------------------------------- */
function _svLin(cols, vals, ref) {
  const tol = SOLVER.tolClean * Math.max(ref || 0, 1e-300);
  const c = {};
  cols.forEach((k, i) => { const v = vals[i]; if (Math.abs(v) > tol) c[k] = v; });
  return Lin.from({ c });
}
/* weights of the load columns for the current symbol values (CONST_KEY → 1, symbol → S.symVals or 1) */
function resultWeights(cols, symVals) {
  return cols.map(k => k === CONST_KEY ? 1 : (symVals && symVals[k] !== undefined && symVals[k] !== '' ? +symVals[k] : 1));
}
/* numeric value of a Lin for given symbol values */
function resEvalLin(lin, symVals) {
  lin = Lin.from(lin);
  let v = lin.get(CONST_KEY) || 0;
  for (const s of lin.symbols()) v += lin.get(s) * (symVals && symVals[s] !== undefined && symVals[s] !== '' ? +symVals[s] : 1);
  return v;
}

/* ---------------------------------------------------------------
   3. Constraint rows
   --------------------------------------------------------------- */
function _svConstraintRows(model, nodes, systems, dof, n) {
  const rows = [];
  nodes.forEach(nd => {
    const d = dof[nd.id];
    supportRows(nd).forEach(r => {
      const vec = zeros(n);
      vec[d[0]] = r.v[0]; vec[d[1]] = r.v[1]; vec[d[2]] = r.v[2];
      rows.push({ type: 'sup', nodeId: nd.id, label: r.label, dir: r.dir, vec });
    });
  });
  systems.forEach(sys => {
    if (!sys.rigid) return;
    const d = dof[sys.m.a].concat(dof[sys.m.b]);
    const vec = zeros(n);
    sys.rigidRow.forEach((v, i) => { vec[d[i]] += v; });
    rows.push({ type: 'axial', memberId: sys.m.id, vec });
  });
  return rows;
}

/* degree of static indeterminacy  n_s = 3m + r − 3j − h  (plane frame count) */
function staticIndeterminacy(model) {
  const used = new Set();
  model.members.forEach(m => { used.add(m.a); used.add(m.b); });
  const nodes = model.nodes.filter(nd => used.has(nd.id));
  let r = 0, h = 0;
  nodes.forEach(nd => {
    const rows = supportRows(nd);
    r += rows.length;
    const ms = nodeMembers(model, nd.id);
    const kr = ms.filter(m => nd.hinge || (m.a === nd.id ? m.relA : m.relB)).length;
    const sr = rows.some(x => x.label === 'M');
    h += (kr === ms.length && !sr && ms.length > 0) ? kr - 1 : kr;
    h += ms.filter(m => (m.a === nd.id ? (m.shrA || m.axA) : (m.shrB || m.axB))).length
       + ms.filter(m => (m.a === nd.id ? (m.shrA && m.axA) : (m.shrB && m.axB))).length;
  });
  return 3 * model.members.length + r - 3 * nodes.length - h;
}

/* ---------------------------------------------------------------
   4. Internal forces by statics (local axes, from end a)
      f  : local end forces acting on the member [Fxa,Fya,Ma,Fxb,Fyb,Mb]
      ll : memberLocalLoads(...) of one column
      N tension +, V positive when the left part tends to rotate clockwise,
      M positive when the −y (bottom) side is in tension (M(0) = −Ma, M(L) = Mb)
      side: −1 → loads located exactly at x excluded, +1 → included, 0 → interior point
   --------------------------------------------------------------- */
function _svForcesAt(f, ll, x, side, eps) {
  let N = -f[0], V = f[1], M = -f[2] + x * f[1];
  for (const ld of ll) {
    if (ld.kind === 'q') {
      const a = ld.x1, b = Math.min(ld.x2, x);
      if (b <= a) continue;
      const w = (ld.x2 - ld.x1) || 1;
      const kx = (ld.px2 - ld.px1) / w, ky = (ld.py2 - ld.py1) / w;
      const sb = b - a, d = x - a;
      const S1 = sb, S2 = sb * sb / 2, S3 = sb * sb * sb / 3;
      N -= ld.px1 * S1 + kx * S2;
      V += ld.py1 * S1 + ky * S2;
      M += d * ld.py1 * S1 + (d * ky - ld.py1) * S2 - ky * S3;
    } else {
      const inc = ld.x < x - eps || (side > 0 && ld.x < x + eps);
      if (!inc) continue;
      if (ld.kind === 'F') { N -= ld.px; V += ld.py; M += (x - ld.x) * ld.py; }
      else M -= ld.m;
    }
  }
  return [N, V, M];
}

/* sampling grid of a member: uniform points ∪ load positions; discontinuities doubled (x⁻, x⁺) */
function _svSampleGrid(L, llCols, ns, eps) {
  const raw = [], disc = [];
  for (let i = 0; i < ns; i++) raw.push(L * i / (ns - 1));
  llCols.forEach(ll => ll.forEach(ld => {
    if (ld.kind === 'q') raw.push(ld.x1, ld.x2);
    else { raw.push(ld.x); disc.push(ld.x); }
  }));
  raw.sort((a, b) => a - b);
  const xs = [], side = [];
  for (const x0 of raw) {
    const x = clamp(x0, 0, L);
    if (xs.length && x <= xs[xs.length - 1] + eps) continue;
    if (disc.some(p => Math.abs(p - x) <= eps)) { xs.push(x, x); side.push(-1, 1); }
    else { xs.push(x); side.push(1); }
  }
  return { xs, side };
}

/* complete per-member result */
function _svMemberResult(st, sys, cols, ugCols, Nlam, llCols, feCols, refs) {
  const L = sys.L;
  const ns = Math.max(2, Math.round(st.nSamples || 101));
  const eps = 1e-9 * Math.max(L, 1e-300);

  /* full local displacements and end forces per column */
  const ulCols = [], fCols = [];
  cols.forEach((_, c) => {
    const ul = sys.cond.recover(sys.toLocal(ugCols[c]), feCols[c]);
    const f = vadd(matvec(sys.Kl, ul), feCols[c], -1);          // f_el = Kl·u_full − fe
    if (Nlam) { f[0] -= Nlam[c]; f[3] += Nlam[c]; }            // rigid axial: multiplier carries N
    (sys.cond.rz || []).forEach(i => {                          // rigid member with axial release: no axial force at that end
      const j = i === 0 ? 3 : 0;
      if (!sys.rel[j]) f[j] += f[i];
      f[i] = 0;
    });
    ulCols.push(ul); fCols.push(f);
  });

  const { xs, side } = _svSampleGrid(L, llCols, ns, eps);
  const np = xs.length;
  const N = [], V = [], M = [], u = [], v = [], th = [];
  cols.forEach((_, c) => {
    const f = fCols[c], ll = llCols[c], ul = ulCols[c];
    const Ni = new Float64Array(np), Vi = new Float64Array(np), Mi = new Float64Array(np);
    const ui = new Float64Array(np), vi = new Float64Array(np), ti = new Float64Array(np);
    let I0 = 0, I1 = 0, IN = 0;                                  // ∫M, ∫sM, ∫N from 0 to x
    for (let i = 0; i < np; i++) {
      const x = xs[i];
      const r = _svForcesAt(f, ll, x, side[i], eps);
      Ni[i] = r[0]; Vi[i] = r[1]; Mi[i] = r[2];
      if (i > 0 && x > xs[i - 1] + eps) {
        const g = integrateVec(s => { const q = _svForcesAt(f, ll, s, 0, eps); return [q[2], s * q[2], q[0]]; }, xs[i - 1], x, 5);
        I0 += g[0]; I1 += g[1]; IN += g[2];
      }
      vi[i] = ul[1] + ul[2] * x + (x * I0 - I1) / sys.EI;        // EI v'' = M
      ti[i] = ul[2] + I0 / sys.EI;
      ui[i] = sys.flex ? ul[0] + IN / sys.EA : ul[0] + (ul[3] - ul[0]) * (L ? x / L : 0);
    }
    N.push(Ni); V.push(Vi); M.push(Mi); u.push(ui); v.push(vi); th.push(ti);
  });

  const fEndLin = [0, 1, 2, 3, 4, 5].map(k => _svLin(cols, fCols.map(f => f[k]), k % 3 === 2 ? refs.M : refs.F));
  return {
    id: sys.m.id, sys, L, eps, xs, side,
    N, V, M, u, v, th,                       // [col][sample]
    fCols, fEndLin, ulCols, llCols,
    Mend: [_svLin(cols, fCols.map(f => -f[2]), refs.M), _svLin(cols, fCols.map(f => f[5]), refs.M)],
    Vend: [_svLin(cols, fCols.map(f => f[1]), refs.F), _svLin(cols, fCols.map(f => -f[4]), refs.F)],
    Nend: [_svLin(cols, fCols.map(f => -f[0]), refs.F), _svLin(cols, fCols.map(f => f[3]), refs.F)],
    num: null, ext: null                     // filled by evaluateResult
  };
}

/* ---------------------------------------------------------------
   5. Main solver
   --------------------------------------------------------------- */
function solveModel(model, symVals) {
  const st = model.settings;
  const cols = loadColumns(model);
  const res = {
    ok: false, mode: st.mode, cols, issues: [], warnings: [], errors: [],
    nodes: {}, members: {}, reactions: [], stats: {}, check: null, refs: null,
    symVals: symVals || {}, nodeIds: [], dof: {}
  };
  const warn = msg => res.warnings.push(msg);
  const fail = msg => { res.errors.push(msg); return res; };

  res.issues = validateModel(model);
  if (res.issues.some(i => i.level === 'error')) return res;

  /* --- DOF numbering --- */
  const used = new Set();
  model.members.forEach(m => { used.add(m.a); used.add(m.b); });
  const nodes = model.nodes.filter(nd => used.has(nd.id));
  const dof = {};
  nodes.forEach((nd, i) => { dof[nd.id] = [3 * i, 3 * i + 1, 3 * i + 2]; });
  const n = 3 * nodes.length;
  const mdofs = m => dof[m.a].concat(dof[m.b]);
  res.nodeIds = nodes.map(nd => nd.id);
  res.dof = dof;

  /* --- reference magnitudes --- */
  let EIref = Infinity, Lmin = Infinity, Lmax = 0;
  model.members.forEach(m => {
    EIref = Math.min(EIref, (m.EI || 1) * (st.EI || 1));
    const L = memberGeom(model, m).L;
    Lmin = Math.min(Lmin, L); Lmax = Math.max(Lmax, L);
  });

  /* --- member systems; rigid-axial over-constraint check --- */
  let flexEA = 0;
  let systems = model.members.map(m => memberSystem(model, m));
  let rows = _svConstraintRows(model, nodes, systems, dof, n);
  let rs = _svRowSpace(rows.map(r => r.vec), n);
  if (rs.dep.length && systems.some(s => s.rigid)) {
    const ids = rs.dep.map(i => rows[i].type === 'axial' ? `member ${rows[i].memberId}` : `support at node ${rows[i].nodeId}`);
    flexEA = SOLVER.rigidRatio * EIref / (Lmin * Lmin);
    const proxy = Object.assign({}, model, { settings: Object.assign({}, st, { axial: 'flex', EA: flexEA }) });
    systems = model.members.map(m => memberSystem(proxy, m));
    rows = _svConstraintRows(model, nodes, systems, dof, n);
    warn(`With axially rigid members the axial forces are statically indeterminate (${ids.join(', ')}); ` +
         `they are evaluated in the limit EA → ∞ (EA·L²/EI = 1e8, EA ratios taken from the members).`);
  }

  /* --- global stiffness --- */
  const K = zeros(n, n);
  systems.forEach(sys => {
    const d = mdofs(sys.m);
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) K[d[i]][d[j]] += sys.Kg[i][j];
  });
  /* zero-stiffness DOFs (e.g. the rotation of a hinge node) are fixed to zero */
  let maxDiag = 0;
  for (let i = 0; i < n; i++) maxDiag = Math.max(maxDiag, Math.abs(K[i][i]));
  const touched = new Array(n).fill(false);
  rows.forEach(r => r.vec.forEach((val, i) => { if (val) touched[i] = true; }));
  let nZero = 0;
  for (let i = 0; i < n; i++) {
    if (!touched[i] && Math.abs(K[i][i]) <= SOLVER.tolZero * maxDiag) {
      const e = zeros(n); e[i] = 1;
      rows.push({ type: 'zero', vec: e, dofIndex: i });
      nZero++;
    }
  }
  rs = _svRowSpace(rows.map(r => r.vec), n);
  if (rs.dep.length) warn('Some constraint equations are linearly dependent; the redundant ones were ignored.');

  /* --- load vectors (one per column) --- */
  const F = cols.map(() => zeros(n));
  const LL = cols.map(() => []), FE = cols.map(() => []);
  cols.forEach((key, c) => {
    nodes.forEach(nd => {
      const fv = nodeLoadVector(nd, key), d = dof[nd.id];
      for (let k = 0; k < 3; k++) F[c][d[k]] += fv[k];
    });
    systems.forEach((sys, mi) => {
      const ll = memberLocalLoads(sys.m, sys, key);
      const fe = fixedEndLocal(ll, sys.L);
      LL[c][mi] = ll; FE[c][mi] = fe;
      const fg = sys.toGlobal(sys.cond.condenseLoad(fe));
      const d = mdofs(sys.m);
      for (let i = 0; i < 6; i++) F[c][d[i]] += fg[i];
    });
  });
  let fRef = 0;
  F.forEach(f => f.forEach(val => { fRef = Math.max(fRef, Math.abs(val)); }));
  if (!fRef) warn('The model has no non-zero loads.');
  fRef = Math.max(fRef, 1e-300);
  res.refs = { F: fRef, M: fRef * Lmax, u: fRef * Lmax * Lmax * Lmax / EIref, th: fRef * Lmax * Lmax / EIref };

  /* --- null-space solve --- */
  const Z = _svNullSpace(rs.R, rs.piv, n);
  const nf = Z.length;
  const U = cols.map(() => zeros(n));
  if (nf) {
    const KZ = Z.map(z => matvec(K, z));
    const Kr = Z.map(za => KZ.map(kz => _svDot(za, kz)));
    const Fr = Z.map(za => cols.map((_, c) => _svDot(za, F[c])));
    const Q = _svGaussJordan(Kr, Fr);
    if (!Q) {
      const ns = staticIndeterminacy(model);
      return fail('The structure is unstable (mechanism): the stiffness matrix is singular. ' + (ns < 0 ? `(degree of indeterminacy n = ${ns} < 0)` : '(check the hinges, releases and supports)'));
    }
    cols.forEach((_, c) => {
      for (let a = 0; a < nf; a++) { const q = Q[a][c]; if (q) for (let i = 0; i < n; i++) U[c][i] += q * Z[a][i]; }
    });
  }

  /* --- multipliers:  Cᵀλ = K u − f   (λ acts on the structure along the row direction) --- */
  const Rsd = cols.map((_, c) => vadd(matvec(K, U[c]), F[c], -1));
  const Ci = rs.indep.map(i => rows[i].vec);
  const CCt = Ci.map(a => Ci.map(b => _svDot(a, b)));
  const RHS = Ci.map(a => cols.map((_, c) => _svDot(a, Rsd[c])));
  const Lam = _svGaussJordan(CCt, RHS);
  if (!Lam) return fail('The constraint equations could not be solved (singular constraint matrix).');
  const lam = rows.map(() => cols.map(() => 0));
  rs.indep.forEach((ri, a) => { lam[ri] = Lam[a]; });

  /* --- node results --- */
  nodes.forEach(nd => {
    const d = dof[nd.id];
    const uCols = cols.map((_, c) => d.map(i => U[c][i]));
    res.nodes[nd.id] = {
      id: nd.id, uCols,
      u: [0, 1, 2].map(k => _svLin(cols, uCols.map(x => x[k]), k < 2 ? res.refs.u : res.refs.th)),
      reactions: []
    };
  });
  rows.forEach((r, ri) => {
    if (r.type !== 'sup') return;
    const rec = {
      nodeId: r.nodeId, label: r.label, dir: r.dir, kind: r.dir ? 'F' : 'M',
      vals: lam[ri], lin: _svLin(cols, lam[ri], r.dir ? res.refs.F : res.refs.M)
    };
    res.nodes[r.nodeId].reactions.push(rec);
    res.reactions.push(rec);
  });

  /* --- member results --- */
  systems.forEach((sys, mi) => {
    const d = mdofs(sys.m);
    const ri = rows.findIndex(r => r.type === 'axial' && r.memberId === sys.m.id);
    const Nlam = ri >= 0 ? lam[ri] : null;
    res.members[sys.m.id] = _svMemberResult(st, sys, cols,
      cols.map((_, c) => d.map(i => U[c][i])), Nlam, LL.map(a => a[mi]), FE.map(a => a[mi]), res.refs);
  });

  /* --- global equilibrium check (applied loads + reactions) --- */
  res.check = cols.map((_, c) => {
    let fx = 0, fy = 0, mz = 0;
    nodes.forEach(nd => {
      const d = dof[nd.id], f = F[c];
      fx += f[d[0]]; fy += f[d[1]]; mz += f[d[2]] + nd.x * f[d[1]] - nd.y * f[d[0]];
    });
    rows.forEach((r, ri) => {
      if (r.type !== 'sup') return;
      const nd = nodeById(model, r.nodeId), l = lam[ri][c];
      if (r.dir) { fx += l * r.dir[0]; fy += l * r.dir[1]; mz += nd.x * l * r.dir[1] - nd.y * l * r.dir[0]; }
      else mz += l;
    });
    return { fx, fy, mz };
  });
  const worst = Math.max(...res.check.map(k => Math.max(Math.abs(k.fx) / res.refs.F, Math.abs(k.fy) / res.refs.F, Math.abs(k.mz) / res.refs.M)));
  if (worst > 1e-6) warn(`Global equilibrium check: relative residual ${worst.toExponential(2)}; results may be affected by round-off.`);

  res.stats = {
    nNodes: nodes.length, nMembers: model.members.length, nDof: n, nFree: nf,
    nConstraints: rs.indep.length, nZero, nCols: cols.length,
    ns: staticIndeterminacy(model), flexEA, axial: flexEA ? 'flex(limit)' : st.axial
  };
  res.ok = true;
  evaluateResult(res, res.symVals);
  return res;
}

/* ---------------------------------------------------------------
   6. Numeric evaluation for given symbol values + extrema
      (re-run whenever S.symVals changes; cheap)
   --------------------------------------------------------------- */
function evaluateResult(res, symVals) {
  if (!res || !res.ok) return res;
  res.symVals = symVals || {};
  const cols = res.cols, w = resultWeights(cols, res.symVals), refs = res.refs;
  const comb = arrs => {
    const out = new Float64Array(arrs[0].length);
    arrs.forEach((a, c) => { if (w[c]) for (let i = 0; i < out.length; i++) out[i] += w[c] * a[i]; });
    return out;
  };
  const glob = { M: { max: null, min: null }, V: { max: null, min: null }, N: { max: null, min: null }, v: { max: null } };
  const better = (cur, cand, sign) => !cur || (sign > 0 ? cand.val > cur.val : cand.val < cur.val) ? cand : cur;

  for (const id in res.members) {
    const mr = res.members[id], xs = mr.xs, L = mr.L;
    const num = { N: comb(mr.N), V: comb(mr.V), M: comb(mr.M), u: comb(mr.u), v: comb(mr.v), th: comb(mr.th) };
    mr.num = num;
    const at = (arrs, i, ref) => ({ memberId: mr.id, x: xs[i], t: L ? xs[i] / L : 0, val: comb(arrs)[i] * 0 + num[arrs === mr.M ? 'M' : arrs === mr.V ? 'V' : arrs === mr.N ? 'N' : 'v'][i], lin: _svLin(cols, arrs.map(a => a[i]), ref) });
    const ext = {};
    for (const key of ['N', 'V', 'M']) {
      const arr = num[key], src = mr[key], ref = key === 'M' ? refs.M : refs.F;
      let imax = 0, imin = 0;
      for (let i = 1; i < arr.length; i++) { if (arr[i] > arr[imax]) imax = i; if (arr[i] < arr[imin]) imin = i; }
      let mx = at(src, imax, ref), mn = at(src, imin, ref);
      if (key === 'M') {
        /* stationary points of M: V changes sign inside a smooth interval → refine by bisection */
        const Vn = num.V;
        const Vc = x => { let s = 0; cols.forEach((_, c) => { if (w[c]) s += w[c] * _svForcesAt(mr.fCols[c], mr.llCols[c], x, 0, mr.eps)[1]; }); return s; };
        for (let i = 0; i + 1 < xs.length; i++) {
          if (!(xs[i + 1] > xs[i] + mr.eps) || !(Vn[i] * Vn[i + 1] < 0)) continue;
          let a = xs[i], b = xs[i + 1], fa = Vn[i];
          for (let it = 0; it < 60; it++) {
            const mid = 0.5 * (a + b), fm = Vc(mid);
            if (fa * fm <= 0) b = mid; else { a = mid; fa = fm; }
          }
          const xr = 0.5 * (a + b);
          const vals = cols.map((_, c) => _svForcesAt(mr.fCols[c], mr.llCols[c], xr, 0, mr.eps)[2]);
          const val = vals.reduce((s, v, c) => s + w[c] * v, 0);
          const cand = { memberId: mr.id, x: xr, t: L ? xr / L : 0, val, lin: _svLin(cols, vals, ref), stationary: true };
          mx = better(mx, cand, +1); mn = better(mn, cand, -1);
        }
      }
      ext[key] = { max: mx, min: mn };
      glob[key].max = better(glob[key].max, mx, +1);
      glob[key].min = better(glob[key].min, mn, -1);
    }
    /* deflection: largest |v| */
    let iv = 0;
    for (let i = 1; i < num.v.length; i++) if (Math.abs(num.v[i]) > Math.abs(num.v[iv])) iv = i;
    ext.v = { max: { memberId: mr.id, x: xs[iv], t: L ? xs[iv] / L : 0, val: num.v[iv], lin: _svLin(cols, mr.v.map(a => a[iv]), refs.u) } };
    if (!glob.v.max || Math.abs(ext.v.max.val) > Math.abs(glob.v.max.val)) glob.v.max = ext.v.max;
    mr.ext = ext;
  }
  /* numeric reactions / node displacements */
  res.reactions.forEach(r => { r.num = r.vals.reduce((s, v, c) => s + w[c] * v, 0); });
  for (const id in res.nodes) {
    const nr = res.nodes[id];
    nr.num = [0, 1, 2].map(k => nr.uCols.reduce((s, uc, c) => s + w[c] * uc[k], 0));
  }
  res.ext = glob;
  return res;
}

/* N, V, M at an arbitrary position x of a member (side as in _svForcesAt) */
function memberForcesAt(res, memberId, x, side = 1) {
  const mr = res && res.members[memberId];
  if (!mr) return null;
  const cols = res.cols, w = resultWeights(cols, res.symVals);
  const vals = cols.map((_, c) => _svForcesAt(mr.fCols[c], mr.llCols[c], clamp(x, 0, mr.L), side, mr.eps));
  const pick = k => vals.map(v => v[k]);
  const num = k => vals.reduce((s, v, c) => s + w[c] * v[k], 0);
  return {
    x, t: mr.L ? x / mr.L : 0,
    N: _svLin(cols, pick(0), res.refs.F), V: _svLin(cols, pick(1), res.refs.F), M: _svLin(cols, pick(2), res.refs.M),
    num: { N: num(0), V: num(1), M: num(2) }
  };
}

/* ---------------------------------------------------------------
   7. Summary tables (plain data; 05-ui renders them)
      kind tags for the formatter: 'F' force, 'M' moment, 'u' deflection (∝1/EI), 'th' rotation (∝1/EI)
   --------------------------------------------------------------- */
function resultTables(res, model) {
  if (!res || !res.ok) return null;
  const reactions = res.reactions.map(r => ({ nodeId: r.nodeId, label: r.label, kind: r.kind, lin: r.lin, num: r.num }));
  const ends = [], extrema = [];
  model.members.forEach(m => {
    const mr = res.members[m.id];
    if (!mr) return;
    ends.push({
      memberId: m.id, a: m.a, b: m.b,
      Na: mr.Nend[0], Nb: mr.Nend[1], Va: mr.Vend[0], Vb: mr.Vend[1], Ma: mr.Mend[0], Mb: mr.Mend[1]
    });
    const e = mr.ext;
    extrema.push({
      memberId: m.id,
      Mmax: e.M.max, Mmin: e.M.min, Vmax: e.V.max, Vmin: e.V.min, Nmax: e.N.max, Nmin: e.N.min, vmax: e.v.max
    });
  });
  const disp = model.nodes.filter(nd => res.nodes[nd.id]).map(nd => {
    const nr = res.nodes[nd.id];
    return { nodeId: nd.id, ux: nr.u[0], uy: nr.u[1], rz: nr.u[2], num: nr.num };
  });
  return { reactions, ends, extrema, disp, global: res.ext, stats: res.stats, check: res.check };
}

/* ---------------------------------------------------------------
   8. Entry point used by the UI
   --------------------------------------------------------------- */
function runSolve() {
  const res = solveModel(S.model, S.symVals);
  S.issues = res.issues
    .concat(res.warnings.map(msg => ({ level: 'warn', msg })))
    .concat(res.errors.map(msg => ({ level: 'error', msg })));
  S.result = res.ok ? res : null;
  S.dirty = false;
  return res;
}