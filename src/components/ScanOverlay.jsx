import React, { useMemo, useRef, useEffect } from 'react'
import { quaternionToMatrix4, transformPoints, filterNonZero } from '../lib/poseUtils'

/**
 * Renders a single scan's point cloud at a given pose with a solid color.
 * Used for query scan (red) and target scan (blue) overlays.
 */
export default function ScanOverlay({
  vizData, icpFrames, frameIdx, poseKey, color, pointSize, opacity,
}) {
  const geoRef = useRef()

  const positions = useMemo(() => {
    if (frameIdx < 0 || !vizData[frameIdx]) return null
    const pose = icpFrames[frameIdx][poseKey]
    const mat = quaternionToMatrix4(pose)
    const validPts = filterNonZero(vizData[frameIdx].points)
    if (validPts.length === 0) return null
    const transformed = transformPoints(validPts, mat)
    return new Float32Array(transformed)
  }, [vizData, icpFrames, frameIdx, poseKey])

  useEffect(() => {
    const geo = geoRef.current
    if (geo && geo.attributes.position) {
      geo.attributes.position.needsUpdate = true
      geo.computeBoundingSphere()
    }
  }, [positions])

  if (!positions) return null

  const geoKey = `scan-${frameIdx}-${positions.length}`

  return (
    <points>
      <bufferGeometry ref={geoRef} key={geoKey}>
        <bufferAttribute attach="attributes-position"
          array={positions} count={positions.length / 3} itemSize={3} />
      </bufferGeometry>
      <pointsMaterial
        color={color}
        size={pointSize * 0.15}
        transparent opacity={opacity}
        sizeAttenuation depthWrite={false}
      />
    </points>
  )
}
