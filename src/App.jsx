import React, { useState, useEffect, useRef, useCallback } from 'react'
import { loadVizData } from './lib/dataLoader'
import { initIcpState, computeNextFrameWithGtCorr, precomputeAllCorrespondences } from './lib/icp'
import Panel from './components/Panel'
import Viewer3D from './components/Viewer3D'
import ImageWindow from './components/ImageWindow'

export default function App() {
  // Raw LiDAR data
  const [vizData, setVizData] = useState(null)

  // Pre-computed GT correspondences
  const [corrData, setCorrData] = useState(null)

  // Frame-by-frame ICP state (mutable ref — doesn't trigger re-render)
  const icpStateRef = useRef(null)

  // Computed ICP frames (triggers re-render when updated)
  const [icpFrames, setIcpFrames] = useState(null)

  // Correspondence distance threshold (meters)
  const [corrDistThreshold, setCorrDistThreshold] = useState(0.2)

  // Display options
  const [showCorrespondences, setShowCorrespondences] = useState(true)

  // Frame navigation
  const [frameIdx, setFrameIdx] = useState(0)
  const [playing, setPlaying] = useState(false)
  const intervalRef = useRef(null)
  const [computing, setComputing] = useState(false)
  const [precomputing, setPrecomputing] = useState(false)

  // Display options
  const [showGtMap, setShowGtMap] = useState(true)
  const [showIcpMap, setShowIcpMap] = useState(true)
  const [showGtTraj, setShowGtTraj] = useState(true)
  const [showIcpTraj, setShowIcpTraj] = useState(true)
  const [pointSize, setPointSize] = useState(2.5)
  const [pointOpacity, setPointOpacity] = useState(0.85)
  const [showQueryScan, setShowQueryScan] = useState(true)
  const [showTargetScan, setShowTargetScan] = useState(true)
  const [scanPointSize, setScanPointSize] = useState(4.0)

  // Load data on mount
  useEffect(() => {
    loadVizData().then(setVizData)
  }, [])

  // Pre-compute GT correspondences when data loads
  useEffect(() => {
    if (vizData) {
      setPrecomputing(true)
      // Use setTimeout to avoid blocking UI during heavy computation
      setTimeout(() => {
        const corr = precomputeAllCorrespondences(vizData)
        setCorrData(corr)
        setPrecomputing(false)
      }, 50)
    }
  }, [vizData])

  // Initialize ICP state when correspondences are ready
  useEffect(() => {
    if (vizData && corrData) {
      const state = initIcpState(vizData)
      icpStateRef.current = state
      setIcpFrames([...state.frames])
      setFrameIdx(0)
    }
  }, [vizData, corrData])

  /**
   * Advance to next frame, computing SVD if needed.
   */
  const advanceFrame = useCallback(() => {
    const state = icpStateRef.current
    if (!state || !vizData || !corrData) return false

    const nextIdx = frameIdx + 1
    if (nextIdx >= vizData.length) return false

    // If we need to compute, do it now
    if (nextIdx >= state.frames.length) {
      const frame = computeNextFrameWithGtCorr(vizData, state, corrData, corrDistThreshold)
      if (!frame) return false
      setIcpFrames([...state.frames])
    }

    setFrameIdx(nextIdx)
    return true
  }, [frameIdx, vizData, corrData, corrDistThreshold])

  /**
   * Go to previous frame (already computed, no ICP needed)
   */
  const retreatFrame = useCallback(() => {
    setFrameIdx(prev => Math.max(prev - 1, 0))
  }, [])

  /**
   * Reset to frame 0 (keep computed frames)
   */
  const resetToStart = useCallback(() => {
    setFrameIdx(0)
    setPlaying(false)
  }, [])

  /**
   * Re-run with new threshold: reset state, recompute up to current frame
   */
  const rerunIcp = useCallback(() => {
    if (!vizData || !corrData) return
    setPlaying(false)
    setComputing(true)

    // Reset state
    const state = initIcpState(vizData)
    icpStateRef.current = state

    // Recompute up to current frameIdx
    const target = Math.min(frameIdx, vizData.length - 1)
    for (let i = 1; i <= target; i++) {
      computeNextFrameWithGtCorr(vizData, state, corrData, corrDistThreshold)
    }

    setIcpFrames([...state.frames])
    setFrameIdx(Math.min(target, state.frames.length - 1))
    setComputing(false)
  }, [vizData, corrData, corrDistThreshold, frameIdx])

  // Playback: advance one frame per tick, computing as needed
  useEffect(() => {
    if (playing && icpFrames && vizData && corrData) {
      intervalRef.current = setInterval(() => {
        setFrameIdx(prev => {
          const state = icpStateRef.current
          if (!state) return prev

          const next = prev + 1
          if (next >= vizData.length) {
            setPlaying(false)
            return prev
          }

          // Compute if needed
          if (next >= state.frames.length) {
            const frame = computeNextFrameWithGtCorr(vizData, state, corrData, corrDistThreshold)
            if (!frame) {
              setPlaying(false)
              return prev
            }
            setIcpFrames([...state.frames])
          }

          return next
        })
      }, 300)
    }
    return () => clearInterval(intervalRef.current)
  }, [playing, icpFrames, vizData, corrData, corrDistThreshold])

  // Keyboard controls
  useEffect(() => {
    const handler = (e) => {
      if (!icpFrames || !vizData || !corrData) return
      if (e.key === 'ArrowRight' || e.key === 'd') {
        e.preventDefault()
        const state = icpStateRef.current
        if (!state) return
        setFrameIdx(prev => {
          const next = prev + 1
          if (next >= vizData.length) return prev
          if (next >= state.frames.length) {
            const frame = computeNextFrameWithGtCorr(vizData, state, corrData, corrDistThreshold)
            if (!frame) return prev
            setIcpFrames([...state.frames])
          }
          return next
        })
      } else if (e.key === 'ArrowLeft' || e.key === 'a') {
        e.preventDefault()
        setFrameIdx(p => Math.max(p - 1, 0))
      } else if (e.key === ' ') {
        e.preventDefault()
        setPlaying(p => !p)
      }
    }
    window.addEventListener('keydown', handler)
    return () => window.removeEventListener('keydown', handler)
  }, [icpFrames, vizData, corrData, corrDistThreshold])

  // Loading state
  if (!vizData) return <div className="loading">Loading LiDAR data...</div>
  if (precomputing || !corrData) return <div className="loading">Pre-computing GT correspondences...</div>
  if (!icpFrames) return <div className="loading">Initializing...</div>

  const currentFrame = icpFrames[frameIdx]

  return (
    <div className="app">
      <Panel
        vizData={vizData}
        icpFrames={icpFrames}
        totalFrames={vizData.length}
        frameIdx={frameIdx}
        setFrameIdx={setFrameIdx}
        playing={playing}
        setPlaying={setPlaying}
        onAdvance={advanceFrame}
        onRetreat={retreatFrame}
        onReset={resetToStart}
        corrDistThreshold={corrDistThreshold}
        setCorrDistThreshold={setCorrDistThreshold}
        onRerunIcp={rerunIcp}
        computing={computing}
        currentFrame={currentFrame}
        corrData={corrData}
        showGtMap={showGtMap} setShowGtMap={setShowGtMap}
        showIcpMap={showIcpMap} setShowIcpMap={setShowIcpMap}
        showGtTraj={showGtTraj} setShowGtTraj={setShowGtTraj}
        showIcpTraj={showIcpTraj} setShowIcpTraj={setShowIcpTraj}
        showCorrespondences={showCorrespondences} setShowCorrespondences={setShowCorrespondences}
        showQueryScan={showQueryScan} setShowQueryScan={setShowQueryScan}
        showTargetScan={showTargetScan} setShowTargetScan={setShowTargetScan}
        scanPointSize={scanPointSize} setScanPointSize={setScanPointSize}
        pointSize={pointSize} setPointSize={setPointSize}
        pointOpacity={pointOpacity} setPointOpacity={setPointOpacity}
      />
      <div className="canvas-wrap">
        <Viewer3D
          vizData={vizData}
          icpFrames={icpFrames}
          frameIdx={frameIdx}
          showGtMap={showGtMap}
          showIcpMap={showIcpMap}
          showGtTraj={showGtTraj}
          showIcpTraj={showIcpTraj}
          showCorrespondences={showCorrespondences}
          corrData={corrData}
          corrDistThreshold={corrDistThreshold}
          showQueryScan={showQueryScan}
          showTargetScan={showTargetScan}
          scanPointSize={scanPointSize}
          pointSize={pointSize}
          pointOpacity={pointOpacity}
        />
        <ImageWindow frame={currentFrame} vizData={vizData} frameIdx={frameIdx} />
      </div>
    </div>
  )
}
