import React, { useMemo, useRef } from 'react'
import * as THREE from 'three'
import { Line2 } from 'three/examples/jsm/lines/Line2.js'
import { LineMaterial } from 'three/examples/jsm/lines/LineMaterial.js'
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js'
import { extend, useThree } from '@react-three/fiber'

extend({ Line2, LineMaterial, LineGeometry })

/**
 * Renders the trajectory line and current pose marker
 */
export default function Trajectory({ frames, frameIdx, poseKey, color }) {
  const { size } = useThree()

  const { lineGeometry, currentPos } = useMemo(() => {
    const positions = []
    for (let i = 0; i <= frameIdx; i++) {
      const pose = frames[i][poseKey]
      positions.push(pose.x, pose.y, pose.z)
    }

    const geometry = new LineGeometry()
    geometry.setPositions(positions)

    const current = new THREE.Vector3(
      frames[frameIdx][poseKey].x,
      frames[frameIdx][poseKey].y,
      frames[frameIdx][poseKey].z
    )

    return { lineGeometry: geometry, currentPos: current }
  }, [frames, frameIdx, poseKey])

  return (
    <group>
      {/* Trajectory line (fat line) */}
      <line2 geometry={lineGeometry}>
        <lineMaterial
          color={color}
          linewidth={3}
          resolution={[size.width, size.height]}
        />
      </line2>

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
