import React, { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, GizmoHelper, GizmoViewport } from '@react-three/drei'
import PointCloud from './PointCloud'
import Trajectory from './Trajectory'
import CorrespondenceLines from './CorrespondenceLines'
import ScanOverlay from './ScanOverlay'

export default function Viewer3D({
  vizData, icpFrames, frameIdx,
  showGtMap, showIcpMap, showGtTraj, showIcpTraj,
  showCorrespondences, corrData, corrDistThreshold,
  showQueryScan, showTargetScan, scanPointSize,
  pointSize, pointOpacity,
}) {
  // Camera target: center of GT trajectory (all frames, normalized)
  const center = useMemo(() => {
    if (!icpFrames || icpFrames.length === 0) return [0, 0, 0]
    let sx = 0, sy = 0, sz = 0
    for (const f of icpFrames) {
      sx += f.gtPose.x; sy += f.gtPose.y; sz += f.gtPose.z
    }
    const n = icpFrames.length
    return [sx / n, sy / n, sz / n]
  }, [icpFrames])

  // Compute global height range for colormap across all frames (GT)
  const heightRange = useMemo(() => {
    let zMin = Infinity, zMax = -Infinity
    // Sample a few frames to estimate range
    for (let i = 0; i < icpFrames.length; i += 5) {
      const pose = icpFrames[i].gtPose
      // Rough estimate: point z + pose z
      zMin = Math.min(zMin, pose.z - 5)
      zMax = Math.max(zMax, pose.z + 20)
    }
    if (zMin === Infinity) { zMin = -5; zMax = 20; }
    return [zMin, zMax]
  }, [icpFrames])

  return (
    <Canvas
      camera={{
        position: [center[0] + 15, center[1] - 15, center[2] + 25],
        up: [0, 0, 1],
        fov: 50, near: 0.1, far: 500,
      }}
      gl={{ antialias: true }}
      style={{ background: '#0b0b14' }}
    >
      <ambientLight intensity={0.5} />

      {/* GT Point Cloud (white, no coloring) */}
      {showGtMap && (
        <PointCloud
          vizData={vizData}
          icpFrames={icpFrames}
          frameIdx={frameIdx}
          poseKey="gtPose"
          pointSize={pointSize}
          opacity={pointOpacity}
          heightRange={heightRange}
          solidColor={[0.85, 0.85, 0.85]}
        />
      )}

      {/* ICP Point Cloud */}
      {showIcpMap && (
        <PointCloud
          vizData={vizData}
          icpFrames={icpFrames}
          frameIdx={frameIdx}
          poseKey="icpPose"
          pointSize={pointSize}
          opacity={pointOpacity}
          heightRange={heightRange}
          tint={null}
        />
      )}

      {/* GT Trajectory */}
      {showGtTraj && (
        <Trajectory
          frames={icpFrames}
          frameIdx={frameIdx}
          poseKey="gtPose"
          color="#5588ff"
        />
      )}

      {/* ICP Trajectory */}
      {showIcpTraj && (
        <Trajectory
          frames={icpFrames}
          frameIdx={frameIdx}
          poseKey="icpPose"
          color="#ffaa44"
        />
      )}

      {/* Correspondence Lines */}
      {showCorrespondences && frameIdx > 0 && (
        <CorrespondenceLines
          vizData={vizData}
          icpFrames={icpFrames}
          frameIdx={frameIdx}
          corrData={corrData}
          corrDistThreshold={corrDistThreshold}
        />
      )}

      {/* Query Scan overlay (red) */}
      {showQueryScan && (
        <ScanOverlay vizData={vizData} icpFrames={icpFrames} frameIdx={frameIdx}
          poseKey="gtPose" color="#ff3333" pointSize={scanPointSize} opacity={pointOpacity} />
      )}

      {/* Target Scan overlay (blue) */}
      {showTargetScan && frameIdx > 0 && (
        <ScanOverlay vizData={vizData} icpFrames={icpFrames} frameIdx={frameIdx - 1}
          poseKey="gtPose" color="#3366ff" pointSize={scanPointSize} opacity={pointOpacity} />
      )}

      {/* Grid removed for cleaner view */}
      <OrbitControls target={center} enableDamping dampingFactor={0.1} rotateSpeed={0.6} zoomSpeed={0.8} />
      <GizmoHelper alignment="bottom-left" margin={[60, 60]}>
        <GizmoViewport labelColor="white" axisHeadScale={0.8} />
      </GizmoHelper>
    </Canvas>
  )
}
