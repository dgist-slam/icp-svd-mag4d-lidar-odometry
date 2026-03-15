import React, { useMemo } from 'react'
import * as THREE from 'three'

/**
 * Renders the trajectory line and current pose marker
 */
export default function Trajectory({ frames, frameIdx, poseKey, color, lineWidth }) {
  const { lineGeometry, currentPos } = useMemo(() => {
    const points = []
    for (let i = 0; i <= frameIdx; i++) {
      const pose = frames[i][poseKey]
      points.push(new THREE.Vector3(pose.x, pose.y, pose.z))
    }

    const geometry = new THREE.BufferGeometry().setFromPoints(points)
    const current = points[points.length - 1]

    return { lineGeometry: geometry, currentPos: current }
  }, [frames, frameIdx, poseKey])

  return (
    <group>
      {/* Trajectory line */}
      <line geometry={lineGeometry}>
        <lineBasicMaterial color={color} linewidth={lineWidth} />
      </line>

      {/* Current position sphere */}
      <mesh position={currentPos}>
        <sphereGeometry args={[0.25, 16, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Current position axes helper */}
      <group position={currentPos}>
        <axesHelper args={[1.5]} />
      </group>

      {/* Start marker */}
      {frameIdx > 0 && (
        <mesh position={[frames[0][poseKey].x, frames[0][poseKey].y, frames[0][poseKey].z]}>
          <boxGeometry args={[0.3, 0.3, 0.3]} />
          <meshBasicMaterial color="#4a4" wireframe />
        </mesh>
      )}
    </group>
  )
}
