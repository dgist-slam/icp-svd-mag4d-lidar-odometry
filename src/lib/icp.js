/**
 * Point-to-Point ICP (Iterative Closest Point) in JavaScript
 * Runs entirely in the browser — no backend needed.
 *
 * Frame-by-frame API: compute one frame at a time as user advances.
 *
 * Reference: Besl & McKay (1992), rotation via SVD (Arun et al. 1987)
 */

import { svd3x3, det3x3 } from './svd3';

// ── SE(3) helpers (exported for use by other modules) ─────────

export function poseToSE3(p) {
  const { x, y, z, qx, qy, qz, qw } = p;
  const xx=qx*qx, yy=qy*qy, zz=qz*qz;
  const xy=qx*qy, xz=qx*qz, yz=qy*qz;
  const wx=qw*qx, wy=qw*qy, wz=qw*qz;
  return [
    [1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy),   x],
    [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx),   y],
    [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy), z],
    [0,           0,           0,           1],
  ];
}

export function se3Inverse(T) {
  const R00=T[0][0],R01=T[0][1],R02=T[0][2];
  const R10=T[1][0],R11=T[1][1],R12=T[1][2];
  const R20=T[2][0],R21=T[2][1],R22=T[2][2];
  const tx=T[0][3], ty=T[1][3], tz=T[2][3];
  return [
    [R00, R10, R20, -(R00*tx+R10*ty+R20*tz)],
    [R01, R11, R21, -(R01*tx+R11*ty+R21*tz)],
    [R02, R12, R22, -(R02*tx+R12*ty+R22*tz)],
    [0, 0, 0, 1],
  ];
}

export function se3Multiply(A, B) {
  const C = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 4; j++)
      for (let k = 0; k < 4; k++)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

export function se3ToPoseDict(T) {
  const R = [[T[0][0],T[0][1],T[0][2]], [T[1][0],T[1][1],T[1][2]], [T[2][0],T[2][1],T[2][2]]];
  const t = [T[0][3], T[1][3], T[2][3]];
  const tr = R[0][0] + R[1][1] + R[2][2];
  let qx, qy, qz, qw;
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    qw = 0.25 / s;
    qx = (R[2][1]-R[1][2]) * s;
    qy = (R[0][2]-R[2][0]) * s;
    qz = (R[1][0]-R[0][1]) * s;
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = 2*Math.sqrt(1+R[0][0]-R[1][1]-R[2][2]);
    qw = (R[2][1]-R[1][2])/s; qx = 0.25*s;
    qy = (R[0][1]+R[1][0])/s; qz = (R[0][2]+R[2][0])/s;
  } else if (R[1][1] > R[2][2]) {
    const s = 2*Math.sqrt(1+R[1][1]-R[0][0]-R[2][2]);
    qw = (R[0][2]-R[2][0])/s; qx = (R[0][1]+R[1][0])/s;
    qy = 0.25*s; qz = (R[1][2]+R[2][1])/s;
  } else {
    const s = 2*Math.sqrt(1+R[2][2]-R[0][0]-R[1][1]);
    qw = (R[1][0]-R[0][1])/s; qx = (R[0][2]+R[2][0])/s;
    qy = (R[1][2]+R[2][1])/s; qz = 0.25*s;
  }
  const qn = Math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
  return {
    x: t[0], y: t[1], z: t[2],
    qx: qx/qn, qy: qy/qn, qz: qz/qn, qw: qw/qn,
  };
}

// ── Point filtering ───────────────────────────────────────────

/** Filter out [0,0,0] invalid points, returns Float64Array (N x 3 flat) */
export function filterNonZeroFlat(points) {
  const valid = [];
  for (let i = 0; i < points.length; i++) {
    const [x, y, z] = points[i];
    if (x !== 0 || y !== 0 || z !== 0) {
      valid.push(x, y, z);
    }
  }
  return new Float64Array(valid);
}

// ── ICP internals ─────────────────────────────────────────────

/** Brute-force nearest neighbors with early exit */
function nearestNeighbors(src, tgt, nSrc, nTgt) {
  const indices = new Int32Array(nSrc);
  const sqDists = new Float64Array(nSrc);

  for (let i = 0; i < nSrc; i++) {
    const sx = src[i*3], sy = src[i*3+1], sz = src[i*3+2];
    let minD = Infinity, minJ = 0;
    for (let j = 0; j < nTgt; j++) {
      const dx = sx - tgt[j*3];
      const d2x = dx * dx;
      if (d2x >= minD) continue;
      const dy = sy - tgt[j*3+1];
      const d2xy = d2x + dy * dy;
      if (d2xy >= minD) continue;
      const dz = sz - tgt[j*3+2];
      const d = d2xy + dz * dz;
      if (d < minD) { minD = d; minJ = j; }
    }
    indices[i] = minJ;
    sqDists[i] = minD;
  }
  return { indices, sqDists };
}

/**
 * Single ICP step: find correspondences + compute optimal R,t via SVD
 * Returns detailed intermediate values for educational display.
 */
function icpStep(src, tgt, nSrc, nTgt, maxDistSq) {
  const { indices, sqDists } = nearestNeighbors(src, tgt, nSrc, nTgt);

  // Adaptive inlier selection (matches Python reference)
  // First try: use maxDistSq threshold
  let inlierMask = new Uint8Array(nSrc);
  let inlierCount = 0;
  for (let i = 0; i < nSrc; i++) {
    if (sqDists[i] < maxDistSq) { inlierMask[i] = 1; inlierCount++; }
  }

  // Fallback: if too few inliers (<30%), use 80th percentile of distances
  const minInlierRatio = 0.3;
  if (inlierCount < minInlierRatio * nSrc) {
    // Find 80th percentile of squared distances
    const sorted = Array.from(sqDists).sort((a, b) => a - b);
    const p80 = sorted[Math.floor(nSrc * 0.8)];
    inlierMask = new Uint8Array(nSrc);
    inlierCount = 0;
    for (let i = 0; i < nSrc; i++) {
      if (sqDists[i] < p80) { inlierMask[i] = 1; inlierCount++; }
    }
  }

  const srcIn = [], tgtIn = [];
  for (let i = 0; i < nSrc; i++) {
    if (inlierMask[i]) {
      srcIn.push(src[i*3], src[i*3+1], src[i*3+2]);
      tgtIn.push(tgt[indices[i]*3], tgt[indices[i]*3+1], tgt[indices[i]*3+2]);
    }
  }
  const nIn = srcIn.length / 3;
  if (nIn < 10) return null;

  // Centroids
  let scx=0, scy=0, scz=0, tcx=0, tcy=0, tcz=0;
  for (let i = 0; i < nIn; i++) {
    scx += srcIn[i*3]; scy += srcIn[i*3+1]; scz += srcIn[i*3+2];
    tcx += tgtIn[i*3]; tcy += tgtIn[i*3+1]; tcz += tgtIn[i*3+2];
  }
  scx /= nIn; scy /= nIn; scz /= nIn;
  tcx /= nIn; tcy /= nIn; tcz /= nIn;

  // Cross-covariance H = sum (p_i - p_bar)(q_i - q_bar)^T
  const H = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < nIn; i++) {
    const px = srcIn[i*3]-scx, py = srcIn[i*3+1]-scy, pz = srcIn[i*3+2]-scz;
    const qx = tgtIn[i*3]-tcx, qy = tgtIn[i*3+1]-tcy, qz = tgtIn[i*3+2]-tcz;
    H[0][0] += px*qx; H[0][1] += px*qy; H[0][2] += px*qz;
    H[1][0] += py*qx; H[1][1] += py*qy; H[1][2] += py*qz;
    H[2][0] += pz*qx; H[2][1] += pz*qy; H[2][2] += pz*qz;
  }

  // SVD: H = U S V^T  →  R = V U^T
  const { U, S, V, Vt } = svd3x3(H);

  let R, t;

  // Guard against degenerate SVD (H ≈ 0 → SVD produces garbage R)
  // This happens when source and target are already well-aligned (near convergence)
  if (S[0] < 1e-6) {
    // Degenerate: use identity rotation, centroid-shift translation
    R = [[1,0,0],[0,1,0],[0,0,1]];
    t = [tcx - scx, tcy - scy, tcz - scz];
  } else {
    // Compute R = V * U^T
    const Ut = [
      [U[0][0],U[1][0],U[2][0]],
      [U[0][1],U[1][1],U[2][1]],
      [U[0][2],U[1][2],U[2][2]],
    ];

    R = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        for (let k = 0; k < 3; k++)
          R[i][j] += V[i][k] * Ut[k][j];

    // Ensure proper rotation (det(R) = +1, not reflection)
    if (det3x3(R) < 0) {
      for (let i = 0; i < 3; i++) V[i][2] *= -1;
      R = [[0,0,0],[0,0,0],[0,0,0]];
      for (let i = 0; i < 3; i++)
        for (let j = 0; j < 3; j++)
          for (let k = 0; k < 3; k++)
            R[i][j] += V[i][k] * Ut[k][j];
    }

    // t = q_bar - R * p_bar
    t = [
      tcx - (R[0][0]*scx + R[0][1]*scy + R[0][2]*scz),
      tcy - (R[1][0]*scx + R[1][1]*scy + R[1][2]*scz),
      tcz - (R[2][0]*scx + R[2][1]*scy + R[2][2]*scz),
    ];
  }

  // Compute mean squared error after transform
  let mse = 0;
  for (let i = 0; i < nIn; i++) {
    const px = srcIn[i*3], py = srcIn[i*3+1], pz = srcIn[i*3+2];
    const rx = R[0][0]*px + R[0][1]*py + R[0][2]*pz + t[0];
    const ry = R[1][0]*px + R[1][1]*py + R[1][2]*pz + t[1];
    const rz = R[2][0]*px + R[2][1]*py + R[2][2]*pz + t[2];
    const dx = rx - tgtIn[i*3], dy = ry - tgtIn[i*3+1], dz = rz - tgtIn[i*3+2];
    mse += dx*dx + dy*dy + dz*dz;
  }
  mse /= nIn;

  return {
    R, t, H, U, S, Vt, V,
    srcCentroid: [scx, scy, scz],
    tgtCentroid: [tcx, tcy, tcz],
    nCorrespondences: nIn,
    mse,
  };
}

/**
 * Run full ICP between source and target point clouds.
 */
export function icpPointToPoint(source, target, params) {
  const { maxIter = 50, maxDist = 1.0, tolerance = 1e-6 } = params;
  const nSrc = source.length / 3;
  const nTgt = target.length / 3;
  const maxDistSq = maxDist * maxDist;

  // Working copy of source points
  const src = new Float64Array(source);
  let R_total = [[1,0,0],[0,1,0],[0,0,1]];
  let t_total = [0, 0, 0];

  // Apply initial transform if provided (motion prediction)
  if (params.initialR && params.initialT) {
    const iR = params.initialR, iT = params.initialT;
    for (let i = 0; i < nSrc; i++) {
      const x = src[i*3], y = src[i*3+1], z = src[i*3+2];
      src[i*3]   = iR[0][0]*x + iR[0][1]*y + iR[0][2]*z + iT[0];
      src[i*3+1] = iR[1][0]*x + iR[1][1]*y + iR[1][2]*z + iT[1];
      src[i*3+2] = iR[2][0]*x + iR[2][1]*y + iR[2][2]*z + iT[2];
    }
    R_total = iR.map(r => [...r]);
    t_total = [...iT];
  }

  let prevError = Infinity;
  let lastStep = null;

  for (let iter = 0; iter < maxIter; iter++) {
    const step = icpStep(src, target, nSrc, nTgt, maxDistSq);
    if (!step) break;
    lastStep = step;

    // Convergence check BEFORE applying step (protects R_total from degenerate updates)
    if (Math.abs(prevError - step.mse) < tolerance) {
      return {
        R: R_total, t: t_total,
        iterations: iter + 1, converged: true, finalError: step.mse,
        nSource: nSrc, nTarget: nTgt,
        detail: step,
      };
    }
    prevError = step.mse;

    // Apply step transform to source points
    for (let i = 0; i < nSrc; i++) {
      const x = src[i*3], y = src[i*3+1], z = src[i*3+2];
      src[i*3]   = step.R[0][0]*x + step.R[0][1]*y + step.R[0][2]*z + step.t[0];
      src[i*3+1] = step.R[1][0]*x + step.R[1][1]*y + step.R[1][2]*z + step.t[1];
      src[i*3+2] = step.R[2][0]*x + step.R[2][1]*y + step.R[2][2]*z + step.t[2];
    }

    // Accumulate: R_total = step.R @ R_total, t_total = step.R @ t_total + step.t
    const newR = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        for (let k = 0; k < 3; k++)
          newR[i][j] += step.R[i][k] * R_total[k][j];
    R_total = newR;

    const newT = [
      step.R[0][0]*t_total[0] + step.R[0][1]*t_total[1] + step.R[0][2]*t_total[2] + step.t[0],
      step.R[1][0]*t_total[0] + step.R[1][1]*t_total[1] + step.R[1][2]*t_total[2] + step.t[1],
      step.R[2][0]*t_total[0] + step.R[2][1]*t_total[1] + step.R[2][2]*t_total[2] + step.t[2],
    ];
    t_total = newT;
  }

  return {
    R: R_total, t: t_total,
    iterations: maxIter, converged: false, finalError: prevError,
    nSource: nSrc, nTarget: nTgt,
    detail: lastStep,
  };
}

// ── GT Correspondence Pre-computation ─────────────────────────

/** Transform flat point array by SE3 matrix, returns new Float64Array */
function transformFlat(pts, T) {
  const n = pts.length / 3;
  const out = new Float64Array(pts.length);
  for (let i = 0; i < n; i++) {
    const x = pts[i*3], y = pts[i*3+1], z = pts[i*3+2];
    out[i*3]   = T[0][0]*x + T[0][1]*y + T[0][2]*z + T[0][3];
    out[i*3+1] = T[1][0]*x + T[1][1]*y + T[1][2]*z + T[1][3];
    out[i*3+2] = T[2][0]*x + T[2][1]*y + T[2][2]*z + T[2][3];
  }
  return out;
}

/**
 * Pre-compute GT correspondences for all consecutive scan pairs.
 * Uses GT poses to transform both scans to world frame, then finds NN.
 * Returns array indexed by frame index (corrData[i] = correspondences for pair i↔i-1).
 * corrData[0] = null (no previous frame).
 */
export function precomputeAllCorrespondences(vizData) {
  const corrData = [null]; // frame 0 has no pair

  for (let i = 1; i < vizData.length; i++) {
    const srcLocal = filterNonZeroFlat(vizData[i].points);
    const tgtLocal = filterNonZeroFlat(vizData[i-1].points);
    const nSrc = srcLocal.length / 3;
    const nTgt = tgtLocal.length / 3;

    // Transform both to world frame using GT poses
    const T_src = poseToSE3(vizData[i].pose);
    const T_tgt = poseToSE3(vizData[i-1].pose);
    const srcWorld = transformFlat(srcLocal, T_src);
    const tgtWorld = transformFlat(tgtLocal, T_tgt);

    // Brute-force NN: for each source point (world), find nearest target (world)
    const nnIndices = new Int32Array(nSrc);
    const nnDists = new Float64Array(nSrc);

    for (let si = 0; si < nSrc; si++) {
      const sx = srcWorld[si*3], sy = srcWorld[si*3+1], sz = srcWorld[si*3+2];
      let minD = Infinity, minJ = 0;
      for (let tj = 0; tj < nTgt; tj++) {
        const dx = sx - tgtWorld[tj*3];
        const d2x = dx * dx;
        if (d2x >= minD) continue;
        const dy = sy - tgtWorld[tj*3+1];
        const d2xy = d2x + dy * dy;
        if (d2xy >= minD) continue;
        const dz = sz - tgtWorld[tj*3+2];
        const d = d2xy + dz * dz;
        if (d < minD) { minD = d; minJ = tj; }
      }
      nnIndices[si] = minJ;
      nnDists[si] = Math.sqrt(minD); // store Euclidean distance (not squared)
    }

    corrData.push({
      srcLocal,   // Float64Array, flat (nSrc*3)
      tgtLocal,   // Float64Array, flat (nTgt*3)
      nSrc,
      nTgt,
      nnIndices,  // Int32Array[nSrc] — index into tgt for each src point
      nnDists,    // Float64Array[nSrc] — Euclidean distance in world frame
      T_src,      // SE3 matrix for src (GT pose)
      T_tgt,      // SE3 matrix for tgt (GT pose)
    });
  }

  return corrData;
}

// ── Frame-by-frame API ────────────────────────────────────────

/**
 * Initialize ICP state from frame 0.
 * Returns the state object and the first frame result.
 */
export function initIcpState(vizData) {
  const gt0 = vizData[0].pose;
  const T0 = poseToSE3(gt0);
  const T0inv = se3Inverse(T0);

  // Frame 0: normalized GT = identity, ICP = identity
  const gtNorm0 = se3Multiply(T0inv, T0); // = identity
  const pts0 = filterNonZeroFlat(vizData[0].points);

  const frame0 = {
    scanIndex: vizData[0].scanIndex,
    elapsed: vizData[0].elapsed,
    gtPose: se3ToPoseDict(gtNorm0),
    icpPose: { x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1 },
    icpDetail: null,
    nPoints: pts0.length / 3,
  };

  return {
    T0inv,
    icpGlobal: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    frames: [frame0],
  };
}

/**
 * Compute relative pose for the next frame using pre-computed GT correspondences.
 * Filters correspondences by distance threshold, then single-shot SVD.
 * Mutates state (icpGlobal, frames).
 * Returns the new frame result, or null if no more frames.
 */
export function computeNextFrameWithGtCorr(vizData, state, corrData, corrDistThreshold) {
  const i = state.frames.length; // index of next frame to compute
  if (i >= vizData.length) return null;

  // Normalize GT pose: T0^{-1} * T_gt[i]
  const gtPose = poseToSE3(vizData[i].pose);
  const gtNorm = se3Multiply(state.T0inv, gtPose);
  const gtPoseDict = se3ToPoseDict(gtNorm);

  // Get pre-computed correspondences for this pair
  const corr = corrData[i];
  const { srcLocal, tgtLocal, nSrc, nTgt, nnIndices, nnDists } = corr;

  // Filter correspondences by distance threshold
  const srcIn = [], tgtIn = [];
  let nInliers = 0;
  for (let si = 0; si < nSrc; si++) {
    if (nnDists[si] < corrDistThreshold) {
      srcIn.push(srcLocal[si*3], srcLocal[si*3+1], srcLocal[si*3+2]);
      const ti = nnIndices[si];
      tgtIn.push(tgtLocal[ti*3], tgtLocal[ti*3+1], tgtLocal[ti*3+2]);
      nInliers++;
    }
  }

  let R, t, H, U, S, V, Vt, srcCentroid, tgtCentroid, mse;

  if (nInliers < 3) {
    // Too few correspondences — use identity transform
    R = [[1,0,0],[0,1,0],[0,0,1]];
    t = [0, 0, 0];
    H = [[0,0,0],[0,0,0],[0,0,0]];
    U = [[1,0,0],[0,1,0],[0,0,1]];
    S = [0, 0, 0];
    V = [[1,0,0],[0,1,0],[0,0,1]];
    Vt = [[1,0,0],[0,1,0],[0,0,1]];
    srcCentroid = [0,0,0];
    tgtCentroid = [0,0,0];
    mse = 0;
  } else {
    // Compute centroids
    let scx=0, scy=0, scz=0, tcx=0, tcy=0, tcz=0;
    for (let k = 0; k < nInliers; k++) {
      scx += srcIn[k*3]; scy += srcIn[k*3+1]; scz += srcIn[k*3+2];
      tcx += tgtIn[k*3]; tcy += tgtIn[k*3+1]; tcz += tgtIn[k*3+2];
    }
    scx /= nInliers; scy /= nInliers; scz /= nInliers;
    tcx /= nInliers; tcy /= nInliers; tcz /= nInliers;
    srcCentroid = [scx, scy, scz];
    tgtCentroid = [tcx, tcy, tcz];

    // Cross-covariance H = sum (p_i - p_bar)(q_i - q_bar)^T
    H = [[0,0,0],[0,0,0],[0,0,0]];
    for (let k = 0; k < nInliers; k++) {
      const px = srcIn[k*3]-scx, py = srcIn[k*3+1]-scy, pz = srcIn[k*3+2]-scz;
      const qx = tgtIn[k*3]-tcx, qy = tgtIn[k*3+1]-tcy, qz = tgtIn[k*3+2]-tcz;
      H[0][0] += px*qx; H[0][1] += px*qy; H[0][2] += px*qz;
      H[1][0] += py*qx; H[1][1] += py*qy; H[1][2] += py*qz;
      H[2][0] += pz*qx; H[2][1] += pz*qy; H[2][2] += pz*qz;
    }

    // SVD: H = U S V^T → R = V U^T
    const svdResult = svd3x3(H);
    U = svdResult.U; S = svdResult.S; V = svdResult.V; Vt = svdResult.Vt;

    if (S[0] < 1e-6) {
      R = [[1,0,0],[0,1,0],[0,0,1]];
      t = [tcx - scx, tcy - scy, tcz - scz];
    } else {
      const Ut = [
        [U[0][0],U[1][0],U[2][0]],
        [U[0][1],U[1][1],U[2][1]],
        [U[0][2],U[1][2],U[2][2]],
      ];
      R = [[0,0,0],[0,0,0],[0,0,0]];
      for (let a = 0; a < 3; a++)
        for (let b = 0; b < 3; b++)
          for (let c = 0; c < 3; c++)
            R[a][b] += V[a][c] * Ut[c][b];

      if (det3x3(R) < 0) {
        for (let a = 0; a < 3; a++) V[a][2] *= -1;
        R = [[0,0,0],[0,0,0],[0,0,0]];
        for (let a = 0; a < 3; a++)
          for (let b = 0; b < 3; b++)
            for (let c = 0; c < 3; c++)
              R[a][b] += V[a][c] * Ut[c][b];
      }

      t = [
        tcx - (R[0][0]*scx + R[0][1]*scy + R[0][2]*scz),
        tcy - (R[1][0]*scx + R[1][1]*scy + R[1][2]*scz),
        tcz - (R[2][0]*scx + R[2][1]*scy + R[2][2]*scz),
      ];
    }

    // Compute MSE after transform
    mse = 0;
    for (let k = 0; k < nInliers; k++) {
      const px = srcIn[k*3], py = srcIn[k*3+1], pz = srcIn[k*3+2];
      const rx = R[0][0]*px + R[0][1]*py + R[0][2]*pz + t[0];
      const ry = R[1][0]*px + R[1][1]*py + R[1][2]*pz + t[1];
      const rz = R[2][0]*px + R[2][1]*py + R[2][2]*pz + t[2];
      const dx = rx - tgtIn[k*3], dy = ry - tgtIn[k*3+1], dz = rz - tgtIn[k*3+2];
      mse += dx*dx + dy*dy + dz*dz;
    }
    mse /= nInliers;
  }

  // Compose T_rel into global trajectory
  const T_rel = [
    [R[0][0], R[0][1], R[0][2], t[0]],
    [R[1][0], R[1][1], R[1][2], t[1]],
    [R[2][0], R[2][1], R[2][2], t[2]],
    [0, 0, 0, 1],
  ];
  state.icpGlobal = se3Multiply(state.icpGlobal, T_rel);

  const frame = {
    scanIndex: vizData[i].scanIndex,
    elapsed: vizData[i].elapsed,
    gtPose: gtPoseDict,
    icpPose: se3ToPoseDict(state.icpGlobal),
    icpDetail: {
      iterations: 1,
      converged: true,
      finalError: mse,
      nSource: nSrc,
      nTarget: nTgt,
      nCorrespondences: nInliers,
      srcCentroid,
      tgtCentroid,
      H, U, S, Vt,
      R, t,
    },
    nPoints: nSrc,
  };

  state.frames.push(frame);
  return frame;
}
