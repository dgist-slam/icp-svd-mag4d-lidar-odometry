import React, { useMemo, useRef, useEffect } from 'react'
import { quaternionToMatrix4 } from '../lib/poseUtils'

/**
 * Renders correspondence lines between matched points of current scan pair.
 * Lines are semi-transparent gray, drawn in normalized GT world frame.
 */
export default function CorrespondenceLines({
  vizData, icpFrames, frameIdx, corrData, corrDistThreshold,
}) {
  const geoRef = useRef()

  const positions = useMemo(() => {
    if (frameIdx < 1 || !corrData || !corrData[frameIdx]) return null

    const corr = corrData[frameIdx]
    const { srcLocal, tgtLocal, nSrc, nnIndices, nnDists } = corr

    // Use normalized GT poses (same frame as PointCloud with poseKey="gtPose")
    const srcPose = icpFrames[frameIdx].gtPose
    const tgtPose = icpFrames[frameIdx - 1].gtPose
    const matSrc = quaternionToMatrix4(srcPose) // row-major flat 16
    const matTgt = quaternionToMatrix4(tgtPose)

    const pts = []

    for (let si = 0; si < nSrc; si++) {
      if (nnDists[si] >= corrDistThreshold) continue

      const ti = nnIndices[si]

      // Transform source point by normalized GT pose
      const sx = srcLocal[si*3], sy = srcLocal[si*3+1], sz = srcLocal[si*3+2]
      const swx = matSrc[0]*sx + matSrc[1]*sy + matSrc[2]*sz  + matSrc[3]
      const swy = matSrc[4]*sx + matSrc[5]*sy + matSrc[6]*sz  + matSrc[7]
      const swz = matSrc[8]*sx + matSrc[9]*sy + matSrc[10]*sz + matSrc[11]

      // Transform target point by normalized GT pose
      const tx = tgtLocal[ti*3], ty = tgtLocal[ti*3+1], tz = tgtLocal[ti*3+2]
      const twx = matTgt[0]*tx + matTgt[1]*ty + matTgt[2]*tz  + matTgt[3]
      const twy = matTgt[4]*tx + matTgt[5]*ty + matTgt[6]*tz  + matTgt[7]
      const twz = matTgt[8]*tx + matTgt[9]*ty + matTgt[10]*tz + matTgt[11]

      pts.push(swx, swy, swz, twx, twy, twz)
    }

    return pts.length > 0 ? new Float32Array(pts) : null
  }, [frameIdx, corrData, corrDistThreshold, icpFrames])

  // Force buffer update when data changes
  useEffect(() => {
    const geo = geoRef.current
    if (geo && geo.attributes.position) {
      geo.attributes.position.needsUpdate = true
      geo.computeBoundingSphere()
    }
  }, [positions])

  if (!positions) return null

  const geoKey = `corr-${positions.length}`

  return (
    <lineSegments>
      <bufferGeometry ref={geoRef} key={geoKey}>
        <bufferAttribute attach="attributes-position"
          array={positions} count={positions.length / 3} itemSize={3} />
      </bufferGeometry>
      <lineBasicMaterial color="#ffff55" transparent opacity={0.6} depthWrite={false} />
    </lineSegments>
  )
}
