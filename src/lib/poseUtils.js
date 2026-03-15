/**
 * SE(3) Pose Utilities + Colormap
 */

/** Quaternion {qx,qy,qz,qw} → row-major 4x4 */
export function quaternionToMatrix4(pose) {
  const { x, y, z, qx, qy, qz, qw } = pose;
  const xx=qx*qx, yy=qy*qy, zz=qz*qz;
  const xy=qx*qy, xz=qx*qz, yz=qy*qz;
  const wx=qw*qx, wy=qw*qy, wz=qw*qz;
  return [
    1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy),   x,
    2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx),   y,
    2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy), z,
    0, 0, 0, 1,
  ];
}

/** Transform array of [x,y,z] points by row-major mat4 → Float32Array */
export function transformPoints(points, mat4) {
  const result = new Float32Array(points.length * 3);
  for (let i = 0; i < points.length; i++) {
    const [px, py, pz] = points[i];
    const idx = i * 3;
    result[idx]     = mat4[0]*px + mat4[1]*py + mat4[2]*pz  + mat4[3];
    result[idx + 1] = mat4[4]*px + mat4[5]*py + mat4[6]*pz  + mat4[7];
    result[idx + 2] = mat4[8]*px + mat4[9]*py + mat4[10]*pz + mat4[11];
  }
  return result;
}

/** Filter out [0,0,0] */
export function filterNonZero(points) {
  return points.filter(([x,y,z]) => x !== 0 || y !== 0 || z !== 0);
}

/** Format pose for display */
export function formatPose(pose) {
  if (!pose) return '';
  return `(${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.z.toFixed(2)})`;
}

/** Euclidean distance between two poses */
export function poseDist(a, b) {
  if (!a || !b) return 0;
  const dx = a.x-b.x, dy = a.y-b.y, dz = a.z-b.z;
  return Math.sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * Jet colormap: t in [0,1] → [r,g,b] in [0,1]
 * blue → cyan → green → yellow → red
 */
export function jetColor(t) {
  t = Math.max(0, Math.min(1, t));
  return [
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 3))),
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 2))),
    Math.max(0, Math.min(1, 1.5 - Math.abs(4*t - 1))),
  ];
}

/** Format a 3x3 matrix row for display */
export function fmt3x3Row(row) {
  return row.map(v => (v >= 0 ? ' ' : '') + v.toFixed(3)).join('  ');
}

/** Format a 3-vector for display */
export function fmtVec3(v) {
  return `(${v.map(x => x.toFixed(3)).join(', ')})`;
}
