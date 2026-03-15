import React from 'react'
import { formatPose, poseDist, fmt3x3Row, fmtVec3 } from '../lib/poseUtils'

export default function Panel({
  vizData, icpFrames, totalFrames, frameIdx, setFrameIdx,
  playing, setPlaying,
  onAdvance, onRetreat, onReset,
  corrDistThreshold, setCorrDistThreshold,
  onRerunIcp, computing,
  currentFrame,
  corrData,
  showGtMap, setShowGtMap, showIcpMap, setShowIcpMap,
  showGtTraj, setShowGtTraj, showIcpTraj, setShowIcpTraj,
  showCorrespondences, setShowCorrespondences,
  showQueryScan, setShowQueryScan, showTargetScan, setShowTargetScan,
  scanPointSize, setScanPointSize,
  pointSize, setPointSize, pointOpacity, setPointOpacity,
}) {
  const drift = poseDist(currentFrame?.gtPose, currentFrame?.icpPose)
  const lastFrame = icpFrames[icpFrames.length - 1]
  const maxDrift = poseDist(lastFrame?.gtPose, lastFrame?.icpPose) || 10
  const driftPct = Math.min((drift / Math.max(maxDrift, 0.1)) * 100, 100)
  const driftLevel = drift < 1 ? 'low' : drift < 5 ? 'mid' : 'high'
  const detail = currentFrame?.icpDetail

  // Correspondence stats for current frame
  const corrStats = corrData && corrData[frameIdx]
    ? (() => {
        const c = corrData[frameIdx]
        let inliers = 0
        for (let i = 0; i < c.nSrc; i++) {
          if (c.nnDists[i] < corrDistThreshold) inliers++
        }
        return { total: c.nSrc, inliers }
      })()
    : null

  // Slider only covers computed frames
  const sliderMax = icpFrames.length - 1

  return (
    <div className="panel">
      <h1>ICP LiDAR Odometry</h1>
      <div className="subtitle">RT604 SLAM — GT Corr. + SVD Demo</div>

      {/* ── Frame Controls ───────────────────────── */}
      <div className="section-box">
        <div className="section-title">Frame Control</div>
        <div className="frame-info">
          <span className="frame-num">#{currentFrame.scanIndex}</span>
          <span className="frame-elapsed">{currentFrame.elapsed.toFixed(1)}s</span>
          <span className="frame-pts">{currentFrame.nPoints} pts</span>
          <span className="frame-pts" style={{ color: '#666' }}>
            ({icpFrames.length}/{totalFrames} computed)
          </span>
        </div>
        <div className="ctrl">
          <input type="range" min={0} max={sliderMax}
            value={frameIdx} onChange={e => setFrameIdx(Number(e.target.value))} />
        </div>
        <div className="btn-row">
          {/* Step back: |◀ */}
          <button onClick={onRetreat} title="Previous frame">&#9198;</button>
          {/* Play/Pause: ▶ / ⏸ — wider button to stand out */}
          <button
            className={playing ? 'playing' : ''}
            onClick={() => setPlaying(!playing)}
            title={playing ? 'Pause' : 'Auto-play'}
            style={{ flex: 1.5 }}
          >
            {playing ? '\u23F8 Pause' : '\u25B6 Play'}
          </button>
          {/* Step forward: ▶| */}
          <button onClick={onAdvance} title="Next frame (compute SVD)">&#9197;</button>
          {/* Reset: ↺ */}
          <button onClick={onReset} title="Reset to frame 0">&#8634;</button>
        </div>
      </div>

      {/* ── Visibility Toggles ───────────────────── */}
      <div className="section-box">
        <div className="section-title">Display Options</div>
        <div className="toggle-grid">
          <label><input type="checkbox" checked={showGtMap} onChange={e => setShowGtMap(e.target.checked)} />
            <span style={{color:'#58f'}}>GT Point Map</span></label>
          <label><input type="checkbox" checked={showIcpMap} onChange={e => setShowIcpMap(e.target.checked)} />
            <span style={{color:'#fa4'}}>ICP Point Map</span></label>
          <label><input type="checkbox" checked={showGtTraj} onChange={e => setShowGtTraj(e.target.checked)} />
            <span style={{color:'#58f'}}>GT Trajectory</span></label>
          <label><input type="checkbox" checked={showIcpTraj} onChange={e => setShowIcpTraj(e.target.checked)} />
            <span style={{color:'#fa4'}}>ICP Trajectory</span></label>
          <label><input type="checkbox" checked={showCorrespondences} onChange={e => setShowCorrespondences(e.target.checked)} />
            <span style={{color:'#ff5'}}>Correspondence Lines</span></label>
          <label><input type="checkbox" checked={showQueryScan} onChange={e => setShowQueryScan(e.target.checked)} />
            <span style={{color:'#f33'}}>Query Scan</span></label>
          <label><input type="checkbox" checked={showTargetScan} onChange={e => setShowTargetScan(e.target.checked)} />
            <span style={{color:'#36f'}}>Target Scan</span></label>
        </div>
        <div className="ctrl">
          <label>Scan Point Size: {scanPointSize.toFixed(1)}</label>
          <input type="range" min={1} max={15} step={0.5}
            value={scanPointSize} onChange={e => setScanPointSize(Number(e.target.value))} />
        </div>
        <div className="ctrl">
          <label>Map Point Size: {pointSize.toFixed(1)}</label>
          <input type="range" min={0.5} max={8} step={0.5}
            value={pointSize} onChange={e => setPointSize(Number(e.target.value))} />
        </div>
        <div className="ctrl">
          <label>Opacity: {pointOpacity.toFixed(2)}</label>
          <input type="range" min={0.1} max={1} step={0.05}
            value={pointOpacity} onChange={e => setPointOpacity(Number(e.target.value))} />
        </div>
      </div>

      {/* ── Correspondence / SVD Parameters ─────── */}
      <div className="section-box">
        <div className="section-title">Correspondence Parameters</div>
        <div className="ctrl">
          <label>Max Corr. Distance: {corrDistThreshold.toFixed(2)} m</label>
          <input type="range" min={0.02} max={2.0} step={0.01}
            value={corrDistThreshold} onChange={e => setCorrDistThreshold(Number(e.target.value))} />
        </div>
        {corrStats && (
          <div style={{ fontSize: '.75rem', color: '#aaa', marginTop: 4 }}>
            Inliers: <strong style={{ color: '#6f6' }}>{corrStats.inliers}</strong> / {corrStats.total} pts
            ({(corrStats.inliers / corrStats.total * 100).toFixed(1)}%)
          </div>
        )}
        <button className="rerun-btn" onClick={onRerunIcp} disabled={computing}>
          {computing ? 'Computing...' : 'Re-run SVD (with new threshold)'}
        </button>
      </div>

      {/* ── SVD Algorithm (Current Frame) ────────── */}
      {detail && (
        <details className="section-box" open>
          <summary className="section-title" style={{cursor:'pointer'}}>
            SVD Algorithm (Frame #{currentFrame.scanIndex})
          </summary>
          <div className="formula-display">
            <div className="formula-step">
              <span className="step-label">1. GT Correspondences</span>
              <span className="step-desc">
                q<sub>i</sub> = NN(T<sub>gt</sub> p<sub>i</sub>, T<sub>gt</sub> Q) — pre-computed
              </span>
              <span className="step-val">
                {detail.nCorrespondences} inliers / {detail.nSource} pts
                (threshold = {corrDistThreshold.toFixed(2)} m)
              </span>
            </div>

            <div className="formula-step">
              <span className="step-label">2. Centroids</span>
              <span className="step-desc">
                p&#772; = (1/N) &Sigma; p<sub>i</sub>,&nbsp; q&#772; = (1/N) &Sigma; q<sub>i</sub>
              </span>
              <span className="step-val src-c">p&#772; = {fmtVec3(detail.srcCentroid)}</span>
              <span className="step-val tgt-c">q&#772; = {fmtVec3(detail.tgtCentroid)}</span>
            </div>

            <div className="formula-step">
              <span className="step-label">3. Cross-covariance</span>
              <span className="step-desc">
                H = &Sigma; (p<sub>i</sub> - p&#772;)(q<sub>i</sub> - q&#772;)<sup>T</sup>
              </span>
              <pre className="matrix-val">
{`H = [${fmt3x3Row(detail.H[0])}]
    [${fmt3x3Row(detail.H[1])}]
    [${fmt3x3Row(detail.H[2])}]`}
              </pre>
            </div>

            <div className="formula-step">
              <span className="step-label">4. SVD</span>
              <span className="step-desc">H = U S V<sup>T</sup></span>
              <span className="step-val">
                S = diag({detail.S.map(s => s.toFixed(1)).join(', ')})
              </span>
            </div>

            <div className="formula-step">
              <span className="step-label">5. Rotation & Translation</span>
              <span className="step-desc">R = V U<sup>T</sup>, &nbsp; t = q&#772; - R p&#772;</span>
              <pre className="matrix-val">
{`R = [${fmt3x3Row(detail.R[0])}]
    [${fmt3x3Row(detail.R[1])}]
    [${fmt3x3Row(detail.R[2])}]`}
              </pre>
              <span className="step-val">t = {fmtVec3(detail.t)}</span>
            </div>

            <div className="formula-step">
              <span className="step-label">6. Result</span>
              <span className="step-val">
                Single-shot SVD (no iteration)
                &nbsp; | MSE = {detail.finalError.toFixed(6)}
              </span>
            </div>
          </div>
        </details>
      )}

      {/* ── Pose Comparison ──────────────────────── */}
      <div className="section-box">
        <div className="section-title">Pose Comparison</div>
        <div className="pose-info">
          <div className="pose-row">
            <span className="pose-label gt">GT</span>
            <span className="pose-value">{formatPose(currentFrame.gtPose)}</span>
          </div>
          <div className="pose-row">
            <span className="pose-label icp">SVD</span>
            <span className="pose-value">{formatPose(currentFrame.icpPose)}</span>
          </div>
        </div>
      </div>

      {/* ── Drift Meter ──────────────────────────── */}
      <div className="section-box">
        <div className="section-title">Accumulated Drift</div>
        <div className="drift-meter">
          <div className="drift-bar-bg">
            <div className={`drift-bar ${driftLevel}`} style={{ width: `${driftPct}%` }} />
          </div>
          <div className="drift-text">
            <span>{drift.toFixed(3)} m</span>
            <span>max: {maxDrift.toFixed(1)} m</span>
          </div>
        </div>
      </div>

      {/* ── Lesson ───────────────────────────────── */}
      <div className="section-box" style={{ borderColor: '#333' }}>
        <div className="section-title" style={{ color: '#999' }}>Lesson</div>
        <div style={{ fontSize: '.75rem', color: '#aaa', lineHeight: 1.5 }}>
          This demo uses <strong style={{ color: '#6f6' }}>GT correspondences</strong> (NN in world frame
          using ground-truth poses) with single-shot <strong style={{ color: '#fa4' }}>SVD</strong>.
          <br /><br />
          Try adjusting <strong>Max Corr. Distance</strong> — small values reject outliers for clean SVD,
          while large values let false correspondences in, corrupting the rotation estimate and increasing drift.
        </div>
      </div>

      <div className="keys-hint">
        &#8592; &#8594; : prev/next &nbsp; Space : play/pause
      </div>
    </div>
  )
}
