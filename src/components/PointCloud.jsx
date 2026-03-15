import React, { useMemo, useRef, useEffect } from 'react'
import { quaternionToMatrix4, transformPoints, filterNonZero, jetColor } from '../lib/poseUtils'

/**
 * Renders accumulated point cloud up to frameIdx with height-based jet coloring.
 * Current frame (query scan) and previous frame (reference scan) get distinct colors.
 */
export default function PointCloud({
  vizData, icpFrames, frameIdx,
  poseKey, pointSize, opacity,
  heightRange, tint, solidColor,
}) {
  const geoRef = useRef()

  const { positions, colors } = useMemo(() => {
    const allPos = []
    const allCol = []
    const [zMin, zMax] = heightRange

    for (let i = 0; i <= frameIdx; i++) {
      const pose = icpFrames[i][poseKey]
      const mat = quaternionToMatrix4(pose)
      const validPts = filterNonZero(vizData[i].points)
      if (validPts.length === 0) continue

      const transformed = transformPoints(validPts, mat)

      const isQueryScan = (i === frameIdx)
      const isPrevScan = (i === frameIdx - 1 && frameIdx > 0)

      for (let j = 0; j < validPts.length; j++) {
        let r, g, b

        if (solidColor) {
          // Single solid color for entire cloud (e.g. GT map = white)
          const age = (frameIdx - i) / Math.max(frameIdx, 1)
          const brightness = 0.4 + 0.6 * (1 - age)
          r = solidColor[0] * brightness
          g = solidColor[1] * brightness
          b = solidColor[2] * brightness
        } else if (isQueryScan) {
          // Query scan (current): bright cyan
          r = 0.1; g = 1.0; b = 0.9
        } else if (isPrevScan) {
          // Previous/reference scan: bright magenta/pink
          r = 1.0; g = 0.3; b = 0.7
        } else {
          // Older frames: jet colormap with age-based dimming
          const wz = transformed[j * 3 + 2]
          const t = (wz - zMin) / (zMax - zMin)
          const jet = jetColor(t)
          const age = (frameIdx - i) / Math.max(frameIdx, 1)
          const brightness = 0.3 + 0.3 * (1 - age)
          r = jet[0] * brightness
          g = jet[1] * brightness
          b = jet[2] * brightness

          if (tint) {
            r = r * 0.6 + tint[0] * 0.4 * brightness
            g = g * 0.6 + tint[1] * 0.4 * brightness
            b = b * 0.6 + tint[2] * 0.4 * brightness
          }
        }

        allPos.push(transformed[j*3], transformed[j*3+1], transformed[j*3+2])
        allCol.push(r, g, b)
      }
    }
    return {
      positions: new Float32Array(allPos),
      colors: new Float32Array(allCol),
    }
  }, [vizData, icpFrames, frameIdx, poseKey, heightRange, tint, solidColor])

  // Force Three.js to re-upload buffer data to GPU when arrays change
  useEffect(() => {
    const geo = geoRef.current
    if (geo) {
      if (geo.attributes.position) geo.attributes.position.needsUpdate = true
      if (geo.attributes.color) geo.attributes.color.needsUpdate = true
      geo.computeBoundingSphere()
    }
  }, [positions, colors])

  if (positions.length === 0) return null

  // Key forces full geometry recreation when point count changes
  // (needsUpdate alone can't handle buffer size changes)
  const geoKey = `${poseKey}-${positions.length}`

  return (
    <points>
      <bufferGeometry ref={geoRef} key={geoKey}>
        <bufferAttribute attach="attributes-position"
          array={positions} count={positions.length / 3} itemSize={3} />
        <bufferAttribute attach="attributes-color"
          array={colors} count={colors.length / 3} itemSize={3} />
      </bufferGeometry>
      <pointsMaterial
        size={pointSize * 0.15}
        vertexColors transparent opacity={opacity}
        sizeAttenuation depthWrite={false}
      />
    </points>
  )
}
