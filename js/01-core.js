/* ============================================================
   Beam & Frame Analysis – 01-core.js
   Constants, numeric helpers, linear algebra, Gauss quadrature,
   fractions, symbolic linear quantities (class Lin) and formatting.
   No dependencies.  Everything here is global (classic scripts).
   ============================================================ */
'use strict';

/* ---------------------------------------------------------------
   1. Constants
   --------------------------------------------------------------- */
const EPS = 1e-9;                 // generic "is zero" tolerance (relative use)
const PIVOT_TOL = 1e-10;          // relative pivot tolerance for singular matrices

const COL = {                     // colours shared by canvas and legend (css .c-*)
  N: '#2e8b57', V: '#1f66d6', M: '#d6281f', def: '#8e44ad',
  member: '#222222', memberHover: '#2d6cdf', memberSel: '#e67e22',
  node: '#1a3a6b', nodeFill: '#ffffff',
  support: '#444444', release: '#ffffff',
  load: '#c0392b', reaction: '#0a7d4f',
  grid: '#eceff3', gridMajor: '#d8dce3', axis: '#b8bec8',
  text: '#333333', textLight: '#777777',
  rubber: '#2d6cdf'
};

const UNI = {
  minus: '−', times: '·', dot: '·', inf: '∞', deg: '°',
  sup: ['⁰', '¹', '²', '³', '⁴', '⁵', '⁶', '⁷', '⁸', '⁹', '⁻']
};

/* length dimension (power of L) of a result of a given kind in symbolic mode,
   relative to a unit *force* symbol:  force→L⁰, moment→L¹, rot→L²/EI, defl→L³/EI */
const KIND_LEN_DIM = { force: 0, moment: 1, rot: 2, defl: 3, length: 1, dist: -1, none: 0 };
const KIND_UNIT    = { force: 'kN', moment: 'kN·m', rot: 'rad', defl: 'm', length: 'm', dist: 'kN/m', none: '' };

/* ---------------------------------------------------------------
   2. Small numeric helpers
   --------------------------------------------------------------- */
function clamp(x, a, b) { return x < a ? a : (x > b ? b : x); }
function sgn(x) { return x > 0 ? 1 : (x < 0 ? -1 : 0); }
function deg2rad(d) { return d * Math.PI / 180; }
function rad2deg(r) { return r * 180 / Math.PI; }
function hypot(x, y) { return Math.sqrt(x * x + y * y); }
function nearly(a, b, tol = 1e-9) { return Math.abs(a - b) <= tol * Math.max(1, Math.abs(a), Math.abs(b)); }
function isZero(x, ref = 1, tol = 1e-9) { return Math.abs(x) <= tol * Math.max(1, Math.abs(ref)); }
function roundTo(x, step) { return step > 0 ? Math.round(x / step) * step : x; }
function snapZero(x, ref = 1, tol = 1e-9) { return isZero(x, ref, tol) ? 0 : x; }

/* superscript for integer powers, e.g. sup(2) → "²", sup(-1) → "⁻¹" */
function sup(n) {
  n = Math.round(n);
  if (n === 1) return '';
  let s = n < 0 ? UNI.sup[10] : '';
  for (const ch of String(Math.abs(n))) s += UNI.sup[+ch];
  return s;
}

/* unique ids for nodes / members */
let _uid = 1;
function uid() { return _uid++; }
function uidReset(min) { if (min >= _uid) _uid = min + 1; }

function deepClone(o) { return JSON.parse(JSON.stringify(o)); }

/* ---------------------------------------------------------------
   3. Fractions and number formatting
   --------------------------------------------------------------- */
/* best rational approximation n/d of x with d ≤ maxDen (continued fractions);
   returns {n,d} or null when no fraction matches within tol */
function toFraction(x, maxDen = 10000, tol = 1e-9) {
  if (!isFinite(x)) return null;
  const sign = x < 0 ? -1 : 1;
  x = Math.abs(x);
  if (x < tol) return { n: 0, d: 1 };
  let h0 = 0, h1 = 1, k0 = 1, k1 = 0, b = x;
  for (let i = 0; i < 64; i++) {
    const a = Math.floor(b);
    const h2 = a * h1 + h0, k2 = a * k1 + k0;
    if (k2 > maxDen) break;
    h0 = h1; h1 = h2; k0 = k1; k1 = k2;
    if (Math.abs(x - h1 / k1) <= tol * Math.max(1, x)) return { n: sign * h1, d: k1 };
    const r = b - a;
    if (r < 1e-14) break;
    b = 1 / r;
  }
  return null;
}

/* decimal with `sig` significant digits, trailing zeros removed */
function fmtDec(x, sig = 4) {
  if (!isFinite(x)) return String(x);
  if (Math.abs(x) < 1e-12) return '0';
  const a = Math.abs(x);
  let s;
  if (a >= 1e7 || a < 1e-5) {
    s = x.toExponential(sig - 1);
    s = s.replace(/\.?0+e/, 'e');
  } else {
    const dec = Math.max(0, sig - 1 - Math.floor(Math.log10(a)));
    s = x.toFixed(Math.min(dec, 10));
    if (s.indexOf('.') >= 0) s = s.replace(/0+$/, '').replace(/\.$/, '');
  }
  if (s === '-0') s = '0';
  return s;
}

/* number as fraction (if requested and possible) or decimal */
function fmtNum(x, frac = false, sig = 4) {
  if (frac) {
    const f = toFraction(x);
    if (f) {
      if (f.n === 0) return '0';
      return f.d === 1 ? String(f.n) : f.n + '/' + f.d;
    }
  }
  return fmtDec(x, sig);
}

/* ---------------------------------------------------------------
   4. Symbols: classification and dimensional information
   --------------------------------------------------------------- */
const CONST_KEY = '1';            // key of the constant (numeric) part of a Lin
const RESERVED_SYMBOLS = ['L', 'EI', 'EA', 'E', 'I', 'A', 'x', 'X', 'y', 'Y'];

/* kind of a load symbol from its first letter:
   M…, C… → concentrated moment, q…, w… → distributed load, anything else → force */
function symKind(name) {
  if (name === CONST_KEY) return 'force';
  if (/^[MmCc]/.test(name)) return 'moment';
  if (/^[qw]/.test(name)) return 'dist';
  return 'force';
}
/* power of L carried by the symbol itself relative to a force: P→0, q→−1, M0→+1 */
function symLenDim(name) { return KIND_LEN_DIM[symKind(name)]; }

/* ---------------------------------------------------------------
   5. class Lin – a linear combination  c1·1 + cP·P + cq·q + …
      Every load value and every result is a Lin.
   --------------------------------------------------------------- */
class Lin {
  constructor(obj) {
    this.c = {};
    if (obj) for (const k in obj) if (obj[k] !== 0 && isFinite(obj[k])) this.c[k] = obj[k];
  }
  static zero() { return new Lin(); }
  static num(x) { return new Lin(x ? { [CONST_KEY]: x } : null); }
  static sym(name, k = 1) { return new Lin({ [name]: k }); }
  /* accept number | Lin | string | {c:{…}} (from JSON) */
  static from(v) {
    if (v instanceof Lin) return v;
    if (typeof v === 'number') return Lin.num(v);
    if (typeof v === 'string') { const l = parseValue(v); return l || Lin.zero(); }
    if (v && typeof v === 'object' && v.c) return new Lin(v.c);
    return Lin.zero();
  }
  clone() { return new Lin(this.c); }
  get(k) { return this.c[k] || 0; }
  keys() { return Object.keys(this.c); }
  symbols() { return Object.keys(this.c).filter(k => k !== CONST_KEY); }
  isZero(tol = 1e-12) {
    for (const k in this.c) if (Math.abs(this.c[k]) > tol) return false;
    return true;
  }
  isConst() { return this.symbols().length === 0; }
  add(o, s = 1) {
    o = Lin.from(o);
    const r = this.clone();
    for (const k in o.c) {
      const v = (r.c[k] || 0) + s * o.c[k];
      if (Math.abs(v) < 1e-14) delete r.c[k]; else r.c[k] = v;
    }
    return r;
  }
  sub(o) { return this.add(o, -1); }
  scale(s) {
    const r = new Lin();
    if (s === 0) return r;
    for (const k in this.c) r.c[k] = this.c[k] * s;
    return r;
  }
  neg() { return this.scale(-1); }
  /* numeric value for plotting; `values` maps symbol → plot value (default 1) */
  eval(values) {
    let s = 0;
    for (const k in this.c) s += this.c[k] * (k === CONST_KEY ? 1 : (values && values[k] !== undefined ? values[k] : 1));
    return s;
  }
  /* remove round-off: coefficients that are tiny relative to the largest one */
  clean(tol = 1e-10) {
    let mx = 0;
    for (const k in this.c) mx = Math.max(mx, Math.abs(this.c[k]));
    const r = new Lin();
    for (const k in this.c) if (Math.abs(this.c[k]) > tol * Math.max(1, mx)) r.c[k] = this.c[k];
    return r;
  }
  toJSON() { return { c: this.c }; }
}

/* sum of an array of Lin (or numbers) */
function linSum(arr) { let r = Lin.zero(); for (const a of arr) r = r.add(a); return r; }

/* ---------------------------------------------------------------
   6. Parsing of load values and positions
   --------------------------------------------------------------- */
/* value:  "P" "-2P" "3/2 q" "M0" "P/2" "0.5*q" "12" "-3.5" "2e3" ""(=0)
   returns Lin or null if invalid */
const RE_VAL = /^([+-]?)\s*(\d*\.?\d+(?:e[+-]?\d+)?)?\s*(?:\/\s*(\d*\.?\d+))?\s*\*?\s*([A-Za-z][A-Za-z0-9_]*)?\s*(?:\/\s*(\d*\.?\d+))?$/i;

function parseValue(str) {
  if (str === undefined || str === null) return Lin.zero();
  const s = String(str).trim().replace(/−/g, '-');
  if (s === '') return Lin.zero();
  const m = RE_VAL.exec(s);
  if (!m) return null;
  const [, sign, coef, den1, sym, den2] = m;
  if (coef === undefined && sym === undefined) return null;
  let k = coef !== undefined ? parseFloat(coef) : 1;
  if (den1 !== undefined) { const d = parseFloat(den1); if (!d) return null; k /= d; }
  if (den2 !== undefined) { const d = parseFloat(den2); if (!d) return null; k /= d; }
  if (sign === '-') k = -k;
  if (!isFinite(k)) return null;
  if (sym) {
    if (RESERVED_SYMBOLS.indexOf(sym) >= 0) return null;
    return Lin.sym(sym, k);
  }
  return Lin.num(k);
}

/* position along a member: "0" "2.5" "L" "L/2" "0.25L" "3L/4" "2/3 L"
   L = numeric member length; returns number or null */
const RE_POS = /^([+-]?)\s*(\d*\.?\d+)?\s*(?:\/\s*(\d*\.?\d+))?\s*\*?\s*(L)?\s*(?:\/\s*(\d*\.?\d+))?$/;

function parsePos(str, L) {
  if (str === undefined || str === null) return null;
  const s = String(str).trim().replace(/−/g, '-');
  if (s === '') return null;
  const m = RE_POS.exec(s);
  if (!m) return null;
  const [, sign, coef, den1, hasL, den2] = m;
  if (coef === undefined && !hasL) return null;
  let v = coef !== undefined ? parseFloat(coef) : 1;
  if (den1 !== undefined) { const d = parseFloat(den1); if (!d) return null; v /= d; }
  if (den2 !== undefined) { const d = parseFloat(den2); if (!d) return null; v /= d; }
  if (hasL) v *= L;
  if (sign === '-') v = -v;
  return isFinite(v) ? v : null;
}

/* describe a position as a fraction of L when it is a nice fraction ("L/2", "3L/4"), else number */
function fmtPos(x, L, frac = true) {
  if (L > 0 && frac) {
    const f = toFraction(x / L, 24, 1e-9);
    if (f) {
      if (f.n === 0) return '0';
      if (f.d === 1) return f.n === 1 ? 'L' : f.n + 'L';
      return (f.n === 1 ? 'L' : f.n + 'L') + '/' + f.d;
    }
  }
  return fmtDec(x, 4);
}

/* numeric factor like "2", "1/2", "0.5" (EI factor in symbolic mode); returns number or null */
function parseFactor(str) {
  const l = parseValue(str);
  if (!l || !l.isConst()) return null;
  const v = l.get(CONST_KEY);
  return v > 0 ? v : null;
}

/* ---------------------------------------------------------------
   7. Formatting of Lin quantities
   --------------------------------------------------------------- */
/* order of terms: constant first, then symbols by kind (force, dist, moment), then alphabetically */
function _symOrder(a, b) {
  if (a === CONST_KEY) return -1;
  if (b === CONST_KEY) return 1;
  const ka = ['force', 'dist', 'moment'].indexOf(symKind(a));
  const kb = ['force', 'dist', 'moment'].indexOf(symKind(b));
  return ka !== kb ? ka - kb : a.localeCompare(b);
}

/* power of L as text: 0 → "", 1 → "L", 2 → "L²", −1 → "/L" */
function fmtLpow(p) {
  if (p === 0) return '';
  if (p > 0) return 'L' + sup(p);
  return '/L' + sup(-p);
}

/*  fmtLin(lin, opts)
    opts.frac    – coefficients as fractions
    opts.symL    – attach powers of L by dimensional analysis
    opts.lenDim  – length dimension of the quantity (KIND_LEN_DIM)
    opts.overEI  – divide the whole expression by EI
    opts.sig     – significant digits for decimals                           */
function fmtLin(lin, opts = {}) {
  lin = Lin.from(lin).clean();
  const frac = !!opts.frac, symL = !!opts.symL, lenDim = opts.lenDim || 0, sig = opts.sig || 4;
  const keys = lin.keys().sort(_symOrder);
  if (keys.length === 0) return '0';
  const terms = [];
  for (const k of keys) {
    const v = lin.c[k];
    const neg = v < 0;
    const a = Math.abs(v);
    let coef = fmtNum(a, frac, sig);
    let tail = '';
    if (k !== CONST_KEY) tail += k;
    if (symL) tail += fmtLpow(lenDim - symLenDim(k));
    let txt;
    if (tail === '') txt = coef;
    else if (coef === '1') txt = tail;
    else txt = coef + (/^\d/.test(tail) ? UNI.times : ' ') + tail;
    terms.push({ neg, txt });
  }
  let s = '';
  terms.forEach((t, i) => {
    if (i === 0) s += (t.neg ? UNI.minus : '') + t.txt;
    else s += (t.neg ? ' ' + UNI.minus + ' ' : ' + ') + t.txt;
  });
  if (opts.overEI) s = (terms.length > 1 ? '(' + s + ')' : s) + '/EI';
  return s;
}

/*  fmtQty(value, kind, M)
    value – number | Lin ;  kind – 'force' 'moment' 'rot' 'defl' 'length' 'dist'
    M     – model settings {mode:'num'|'sym', frac:bool, symL:bool}
    opts  – {unit:bool, sig:int}
    Numeric mode  → "12.5 kN" (symbols kept if the user used them: "0.375 q")
    Symbolic mode → "3/8 qL − 1/4 P", "(1/8 PL² + 1/24 qL³)/EI"                 */
function fmtQty(value, kind, M, opts = {}) {
  const lin = Lin.from(value).clean();
  const symMode = M && M.mode === 'sym';
  const symL = symMode && !!(M && M.symL);
  const overEI = symMode && (kind === 'rot' || kind === 'defl');
  let s = fmtLin(lin, { frac: !!(M && M.frac), symL, lenDim: KIND_LEN_DIM[kind] || 0, overEI, sig: opts.sig });
  if (opts.unit && !symMode && lin.isConst() && KIND_UNIT[kind]) s += ' ' + KIND_UNIT[kind];
  return s;
}

/* plain length (coordinate) formatting: "3", "2.5" or "1/2 L" when lengths are in L */
function fmtLen(x, M) {
  if (M && M.symL && M.mode === 'sym') {
    const c = fmtNum(x, !!M.frac);
    return c === '1' ? 'L' : (c === '0' ? '0' : c + (M.frac && /\//.test(c) ? ' ' : '') + 'L');
  }
  return fmtDec(x, 5);
}

/* ---------------------------------------------------------------
   8. Dense linear algebra (arrays of arrays, row major)
   --------------------------------------------------------------- */
function zeros(n, m) {
  const A = new Array(n);
  for (let i = 0; i < n; i++) A[i] = m === undefined ? 0 : new Array(m).fill(0);
  return A;
}
function eye(n) { const A = zeros(n, n); for (let i = 0; i < n; i++) A[i][i] = 1; return A; }
function copyMat(A) { return A.map(r => r.slice()); }
function transpose(A) {
  const n = A.length, m = n ? A[0].length : 0;
  const T = zeros(m, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < m; j++) T[j][i] = A[i][j];
  return T;
}
function matmul(A, B) {
  const n = A.length, k = B.length, m = k ? B[0].length : 0;
  const C = zeros(n, m);
  for (let i = 0; i < n; i++) {
    const Ai = A[i], Ci = C[i];
    for (let p = 0; p < k; p++) {
      const a = Ai[p]; if (a === 0) continue;
      const Bp = B[p];
      for (let j = 0; j < m; j++) Ci[j] += a * Bp[j];
    }
  }
  return C;
}
function matvec(A, x) { return A.map(r => { let s = 0; for (let j = 0; j < r.length; j++) s += r[j] * x[j]; return s; }); }
function dot(a, b) { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; }
function vadd(a, b, s = 1) { return a.map((v, i) => v + s * b[i]); }
function vscale(a, s) { return a.map(v => v * s); }
function vnorm(a) { return Math.sqrt(dot(a, a)); }
function matMaxAbs(A) { let m = 0; for (const r of A) for (const v of r) m = Math.max(m, Math.abs(v)); return m; }

/* solve A X = B for several right-hand sides (B: n×k). Gaussian elimination with
   partial pivoting. Returns X (n×k) or null when A is singular (relative tolerance). */
function solveMulti(A, B, tol = PIVOT_TOL) {
  const n = A.length;
  if (n === 0) return [];
  const k = B[0].length;
  const M = A.map((r, i) => r.concat(B[i]));
  const scale = matMaxAbs(A) || 1;
  for (let c = 0; c < n; c++) {
    let p = c, best = Math.abs(M[c][c]);
    for (let r = c + 1; r < n; r++) { const v = Math.abs(M[r][c]); if (v > best) { best = v; p = r; } }
    if (best <= tol * scale) return null;
    if (p !== c) { const t = M[p]; M[p] = M[c]; M[c] = t; }
    const piv = M[c][c], rowc = M[c];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / piv;
      if (f === 0) continue;
      const row = M[r];
      for (let j = c; j < n + k; j++) row[j] -= f * rowc[j];
    }
  }
  const X = zeros(n, k);
  for (let i = 0; i < n; i++) for (let j = 0; j < k; j++) X[i][j] = M[i][n + j] / M[i][i];
  return X;
}

/* solve A x = b (single right-hand side); null when singular */
function solve(A, b, tol = PIVOT_TOL) {
  const X = solveMulti(A, b.map(v => [v]), tol);
  return X ? X.map(r => r[0]) : null;
}

/* inverse; null when singular */
function invert(A, tol = PIVOT_TOL) { return solveMulti(A, eye(A.length), tol); }

/* reduced row echelon form with partial pivoting.
   returns {R, pivots (column index of each pivot row), rank, free (non-pivot columns)} */
function rref(A, tol = PIVOT_TOL) {
  const R = copyMat(A);
  const n = R.length, m = n ? R[0].length : 0;
  const scale = matMaxAbs(R) || 1;
  const pivots = [];
  let row = 0;
  for (let c = 0; c < m && row < n; c++) {
    let p = row, best = Math.abs(R[row][c]);
    for (let r = row + 1; r < n; r++) { const v = Math.abs(R[r][c]); if (v > best) { best = v; p = r; } }
    if (best <= tol * scale) continue;
    if (p !== row) { const t = R[p]; R[p] = R[row]; R[row] = t; }
    const piv = R[row][c], rr = R[row];
    for (let j = 0; j < m; j++) rr[j] /= piv;
    for (let r = 0; r < n; r++) {
      if (r === row) continue;
      const f = R[r][c];
      if (f === 0) continue;
      const rw = R[r];
      for (let j = 0; j < m; j++) rw[j] -= f * rr[j];
    }
    pivots.push(c);
    row++;
  }
  const free = [];
  for (let c = 0; c < m; c++) if (pivots.indexOf(c) < 0) free.push(c);
  return { R, pivots, rank: pivots.length, free };
}

/* orthonormal-ish basis of the null space of C (p×n): returns Z (n×k), C·Z = 0.
   Built from the RREF: one column per free variable. */
function nullSpace(C, n, tol = PIVOT_TOL) {
  if (!C.length) return eye(n);
  const { R, pivots, free } = rref(C, tol);
  const Z = zeros(n, free.length);
  free.forEach((fc, j) => {
    Z[fc][j] = 1;
    pivots.forEach((pc, i) => { Z[pc][j] = -R[i][fc]; });
  });
  return Z;
}

/* rows of C that are linearly independent (indices), via RREF of Cᵀ */
function independentRows(C, tol = PIVOT_TOL) {
  if (!C.length) return [];
  return rref(transpose(C), tol).pivots;
}

/* ---------------------------------------------------------------
   9. Gauss–Legendre quadrature on [a,b]
   --------------------------------------------------------------- */
const GL = {
  2: [[-0.5773502691896257, 1], [0.5773502691896257, 1]],
  3: [[0, 0.8888888888888888], [-0.7745966692414834, 0.5555555555555556], [0.7745966692414834, 0.5555555555555556]],
  4: [[-0.3399810435848563, 0.6521451548625461], [0.3399810435848563, 0.6521451548625461],
      [-0.8611363115940526, 0.3478548451374538], [0.8611363115940526, 0.3478548451374538]],
  5: [[0, 0.5688888888888889], [-0.5384693101056831, 0.4786286704993665], [0.5384693101056831, 0.4786286704993665],
      [-0.9061798459386640, 0.2369268850561891], [0.9061798459386640, 0.2369268850561891]],
  6: [[-0.2386191860831969, 0.4679139345726910], [0.2386191860831969, 0.4679139345726910],
      [-0.6612093864662645, 0.3607615730481386], [0.6612093864662645, 0.3607615730481386],
      [-0.9324695142031521, 0.1713244923791704], [0.9324695142031521, 0.1713244923791704]]
};
/* ∫ₐᵇ f(x) dx  – exact for polynomials of degree ≤ 2n−1 (n = 5 → degree 9) */
function integrate(f, a, b, n = 5) {
  const pts = GL[n] || GL[5];
  const h = (b - a) / 2, c = (a + b) / 2;
  let s = 0;
  for (const [t, w] of pts) s += w * f(c + h * t);
  return s * h;
}
/* vector-valued version: f returns an array */
function integrateVec(f, a, b, n = 5) {
  const pts = GL[n] || GL[5];
  const h = (b - a) / 2, c = (a + b) / 2;
  let s = null;
  for (const [t, w] of pts) {
    const v = f(c + h * t);
    if (!s) s = new Array(v.length).fill(0);
    for (let i = 0; i < v.length; i++) s[i] += w * v[i];
  }
  return s ? s.map(v => v * h) : [];
}

/* ---------------------------------------------------------------
   10. Geometry helpers (used by render and ui)
   --------------------------------------------------------------- */
/* distance from point P to segment AB and the parameter t∈[0,1] of the foot */
function distToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const L2 = dx * dx + dy * dy;
  let t = L2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0;
  t = clamp(t, 0, 1);
  const fx = ax + t * dx, fy = ay + t * dy;
  return { d: hypot(px - fx, py - fy), t, x: fx, y: fy };
}
/* are three points collinear (for merging segments) */
function collinear(ax, ay, bx, by, cx, cy, tol = 1e-6) {
  const cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  const scale = Math.max(hypot(bx - ax, by - ay), hypot(cx - ax, cy - ay), 1);
  return Math.abs(cross) <= tol * scale * scale;
}

/* ---------------------------------------------------------------
   11. DOM helpers (used by ui)
   --------------------------------------------------------------- */
function $(id) { return document.getElementById(id); }
function esc(s) {
  return String(s).replace(/[&<>"']/g, ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]));
}
/* el('div', {class:'box', onclick:fn, html:'…'}, [children…]) */
function el(tag, attrs = {}, children = []) {
  const e = document.createElement(tag);
  for (const k in attrs) {
    const v = attrs[k];
    if (v === undefined || v === null || v === false) continue;
    if (k === 'class') e.className = v;
    else if (k === 'html') e.innerHTML = v;
    else if (k === 'text') e.textContent = v;
    else if (k.startsWith('on') && typeof v === 'function') e.addEventListener(k.slice(2), v);
    else if (k === 'value') e.value = v;
    else if (k === 'checked') e.checked = !!v;
    else if (v === true) e.setAttribute(k, '');
    else e.setAttribute(k, v);
  }
  for (const c of [].concat(children)) {
    if (c === null || c === undefined) continue;
    e.appendChild(typeof c === 'string' ? document.createTextNode(c) : c);
  }
  return e;
}