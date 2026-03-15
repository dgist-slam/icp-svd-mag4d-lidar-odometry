import React from 'react'

export default function ImageWindow({ frame, vizData, frameIdx }) {
  if (!vizData || !vizData[frameIdx]) return null
  const image = vizData[frameIdx].image

  return (
    <div className="img-window">
      <div className="img-label">
        Frame #{frame.scanIndex} — {frame.elapsed.toFixed(1)}s
      </div>
      <img src={`/${image}`} alt={`Frame ${frame.scanIndex}`} />
    </div>
  )
}
