/**
 * Robust 3x3 SVD using analytical eigenvalue computation
 * Used for ICP optimal rotation estimation (Arun et al. 1987)
 *
 * Uses closed-form eigenvalue solution for 3x3 symmetric matrices
 * (Kopp 2008, "Efficient numerical diagonalization of Hermitian 3x3 matrices")
 */

/**
 * Compute eigenvalues of a 3x3 symmetric matrix using Cardano/trigonometric method.
 * Returns eigenvalues in descending order.
 */
function symmetricEigenvalues3x3(A) {
  const a00 = A[0][0], a01 = A[0][1], a02 = A[0][2];
  const a11 = A[1][1], a12 = A[1][2], a22 = A[2][2];

  const tr = a00 + a11 + a22;
  const m = tr / 3;

  // Shifted matrix K = A - m*I (for numerical stability)
  const k00 = a00 - m, k11 = a11 - m, k22 = a22 - m;

  // q = det(K)/2, p = sum of squares of K elements / 6
  const q_val = (k00 * (k11 * k22 - a12 * a12)
               - a01 * (a01 * k22 - a12 * a02)
               + a02 * (a01 * a12 - k11 * a02)) / 2;

  const p = (k00 * k00 + k11 * k11 + k22 * k22
            + 2 * (a01 * a01 + a02 * a02 + a12 * a12)) / 6;

  if (p < 1e-30) {
    // Matrix is essentially m * I
    return [m, m, m];
  }

  const p32 = p * Math.sqrt(p); // p^(3/2)
  let phi = q_val / p32;
  // Clamp for numerical stability
  phi = Math.max(-1, Math.min(1, phi));
  phi = Math.acos(phi) / 3;

  const sq2p = 2 * Math.sqrt(p);
  const eig1 = m + sq2p * Math.cos(phi);
  const eig2 = m + sq2p * Math.cos(phi - 2 * Math.PI / 3);
  const eig3 = m + sq2p * Math.cos(phi + 2 * Math.PI / 3);

  // Sort descending
  const eigs = [eig1, eig2, eig3];
  eigs.sort((a, b) => b - a);
  return eigs;
}

/**
 * Compute eigenvector for a symmetric 3x3 matrix given an eigenvalue.
 * Uses cross product of two rows of (A - λI) to find the eigenvector.
 */
function eigenvector3x3(A, lambda) {
  // B = A - lambda * I
  const b00 = A[0][0] - lambda, b01 = A[0][1], b02 = A[0][2];
  const b10 = A[1][0], b11 = A[1][1] - lambda, b12 = A[1][2];
  const b20 = A[2][0], b21 = A[2][1], b22 = A[2][2] - lambda;

  // Cross products of rows
  const r01 = [b00 * b11 - b01 * b10, b00 * b12 - b02 * b10, b01 * b12 - b02 * b11]; // simplified
  // Actually: cross(row0, row1)
  const c01 = [
    b01 * b12 - b02 * b11,
    b02 * b10 - b00 * b12,
    b00 * b11 - b01 * b10,
  ];
  const c02 = [
    b01 * b22 - b02 * b21,
    b02 * b20 - b00 * b22,
    b00 * b21 - b01 * b20,
  ];
  const c12 = [
    b11 * b22 - b12 * b21,
    b12 * b20 - b10 * b22,
    b10 * b21 - b11 * b20,
  ];

  // Pick the cross product with the largest magnitude
  const n01 = c01[0]*c01[0] + c01[1]*c01[1] + c01[2]*c01[2];
  const n02 = c02[0]*c02[0] + c02[1]*c02[1] + c02[2]*c02[2];
  const n12 = c12[0]*c12[0] + c12[1]*c12[1] + c12[2]*c12[2];

  let v;
  if (n01 >= n02 && n01 >= n12) {
    const len = Math.sqrt(n01);
    v = len > 1e-15 ? [c01[0]/len, c01[1]/len, c01[2]/len] : null;
  } else if (n02 >= n12) {
    const len = Math.sqrt(n02);
    v = len > 1e-15 ? [c02[0]/len, c02[1]/len, c02[2]/len] : null;
  } else {
    const len = Math.sqrt(n12);
    v = len > 1e-15 ? [c12[0]/len, c12[1]/len, c12[2]/len] : null;
  }

  return v;
}

/**
 * Full eigendecomposition of a 3x3 symmetric matrix.
 * Returns { values: [λ1, λ2, λ3], vectors: [[v1],[v2],[v3]] } (column vectors stored as rows for V[row][col])
 */
function symmetricEigen3x3(A) {
  const eigenvalues = symmetricEigenvalues3x3(A);

  // Compute eigenvectors
  const vecs = [];
  for (let i = 0; i < 3; i++) {
    let v = eigenvector3x3(A, eigenvalues[i]);
    if (!v) {
      // Fallback: use standard basis
      v = [0, 0, 0];
      v[i] = 1;
    }
    vecs.push(v);
  }

  // Orthogonalize (Gram-Schmidt) to handle near-degenerate eigenvalues
  // v1 is already normalized
  // v2 = v2 - (v2·v1)*v1, then normalize
  let dot = vecs[1][0]*vecs[0][0] + vecs[1][1]*vecs[0][1] + vecs[1][2]*vecs[0][2];
  vecs[1] = [vecs[1][0] - dot*vecs[0][0], vecs[1][1] - dot*vecs[0][1], vecs[1][2] - dot*vecs[0][2]];
  let len = Math.sqrt(vecs[1][0]*vecs[1][0] + vecs[1][1]*vecs[1][1] + vecs[1][2]*vecs[1][2]);
  if (len > 1e-15) {
    vecs[1] = [vecs[1][0]/len, vecs[1][1]/len, vecs[1][2]/len];
  } else {
    // v1 and v2 were parallel, generate perpendicular
    const ax = Math.abs(vecs[0][0]), ay = Math.abs(vecs[0][1]), az = Math.abs(vecs[0][2]);
    let perp;
    if (ax <= ay && ax <= az) perp = [1, 0, 0];
    else if (ay <= az) perp = [0, 1, 0];
    else perp = [0, 0, 1];
    dot = perp[0]*vecs[0][0] + perp[1]*vecs[0][1] + perp[2]*vecs[0][2];
    vecs[1] = [perp[0] - dot*vecs[0][0], perp[1] - dot*vecs[0][1], perp[2] - dot*vecs[0][2]];
    len = Math.sqrt(vecs[1][0]*vecs[1][0] + vecs[1][1]*vecs[1][1] + vecs[1][2]*vecs[1][2]);
    vecs[1] = [vecs[1][0]/len, vecs[1][1]/len, vecs[1][2]/len];
  }

  // v3 = v1 × v2 (guaranteed orthogonal)
  vecs[2] = [
    vecs[0][1]*vecs[1][2] - vecs[0][2]*vecs[1][1],
    vecs[0][2]*vecs[1][0] - vecs[0][0]*vecs[1][2],
    vecs[0][0]*vecs[1][1] - vecs[0][1]*vecs[1][0],
  ];

  // Return as column-vectors in a matrix: V[row][col] = vecs[col][row]
  const V = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      V[i][j] = vecs[j][i];

  return { values: eigenvalues, vectors: V };
}

// ── Matrix helpers ───────────────────────────────────────────

function mat3Mult(A, B) {
  const C = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++)
        C[i][j] += A[i][k] * B[k][j];
  return C;
}

function mat3MtM(M) {
  const R = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++)
        R[i][j] += M[k][i] * M[k][j];
  return R;
}

/**
 * SVD of a 3x3 matrix M.
 * Returns { U, S, V, Vt } where M = U * diag(S) * Vt
 */
export function svd3x3(M) {
  const MtM = mat3MtM(M);
  const { values: eigenvals, vectors: Vs } = symmetricEigen3x3(MtM);

  // Singular values = sqrt of eigenvalues
  const S = eigenvals.map(e => Math.sqrt(Math.max(0, e)));

  // U = M * V * diag(1/S) for columns where S > 0
  const MV = mat3Mult(M, Vs);
  const U = [[0,0,0],[0,0,0],[0,0,0]];
  let validCols = 0;

  for (let j = 0; j < 3; j++) {
    if (S[j] > 1e-10) {
      const inv_s = 1.0 / S[j];
      for (let i = 0; i < 3; i++) U[i][j] = MV[i][j] * inv_s;
      validCols++;
    }
  }

  // Handle rank-deficient cases: fill missing U columns
  if (validCols === 2) {
    // Third column = cross product of first two
    const u0 = [U[0][0], U[1][0], U[2][0]];
    const u1 = [U[0][1], U[1][1], U[2][1]];
    const u2 = [
      u0[1]*u1[2] - u0[2]*u1[1],
      u0[2]*u1[0] - u0[0]*u1[2],
      u0[0]*u1[1] - u0[1]*u1[0],
    ];
    U[0][2] = u2[0]; U[1][2] = u2[1]; U[2][2] = u2[2];
  } else if (validCols <= 1) {
    // Fall back to identity-like
    if (validCols === 1) {
      // Pick perpendicular vectors to U[:,0]
      const u0 = [U[0][0], U[1][0], U[2][0]];
      const ax = Math.abs(u0[0]), ay = Math.abs(u0[1]), az = Math.abs(u0[2]);
      let perp;
      if (ax <= ay && ax <= az) perp = [1, 0, 0];
      else if (ay <= az) perp = [0, 1, 0];
      else perp = [0, 0, 1];
      const dot = perp[0]*u0[0] + perp[1]*u0[1] + perp[2]*u0[2];
      let u1 = [perp[0]-dot*u0[0], perp[1]-dot*u0[1], perp[2]-dot*u0[2]];
      const len = Math.sqrt(u1[0]*u1[0]+u1[1]*u1[1]+u1[2]*u1[2]);
      u1 = [u1[0]/len, u1[1]/len, u1[2]/len];
      U[0][1] = u1[0]; U[1][1] = u1[1]; U[2][1] = u1[2];
      const u2 = [u0[1]*u1[2]-u0[2]*u1[1], u0[2]*u1[0]-u0[0]*u1[2], u0[0]*u1[1]-u0[1]*u1[0]];
      U[0][2] = u2[0]; U[1][2] = u2[1]; U[2][2] = u2[2];
    } else {
      U[0][0]=1; U[1][1]=1; U[2][2]=1;
    }
  }

  const Vt = [
    [Vs[0][0], Vs[1][0], Vs[2][0]],
    [Vs[0][1], Vs[1][1], Vs[2][1]],
    [Vs[0][2], Vs[1][2], Vs[2][2]],
  ];
  return { U, S, V: Vs, Vt };
}

export function det3x3(M) {
  return M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
       - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
       + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
}
