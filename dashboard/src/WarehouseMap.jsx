import React, { useState } from 'react'
import { ROBOT_COLOR, ROBOT_SHORT } from './theme.js'

// Top-down live map. World frame: x east, y north  ->  SVG y is flipped.
// Every robot pose / path / goal / reservation drawn here comes from the ROS topics (ground truth from Gazebo,
// Nav2 global plan, traffic-manager state).
const fy = (y) => -y

export default function WarehouseMap({ layout, state }) {
  const [hover, setHover] = useState(null)
  if (!layout) return <div className="panel map-empty">loading layout…</div>
  const b = layout.bounds
  const pad = 0.45
  const vb = [b.x_min - pad, fy(b.y_max) - pad, b.x_max - b.x_min + 2 * pad, b.y_max - b.y_min + 2 * pad]
  const robots = state?.robots || []
  const zones = state?.traffic?.zones || []
  const stations = state?.traffic?.stations || []
  const items = state?.items || {}
  const zoneState = Object.fromEntries(zones.map(z => [z.id, z]))
  const stationState = Object.fromEntries(stations.map(z => [z.id, z]))
  const colorOf = (id) => ROBOT_COLOR[id] || '#898781'

  return (
    <div className="panel map-panel">
      <div className="panel-title">
        <span>Live warehouse map</span>
        <span className="muted small">positions: Gazebo ground truth · paths: Nav2 · reservations: traffic manager</span>
      </div>
      <svg viewBox={vb.join(' ')} className="map" role="img" aria-label="Top-down map of the warehouse with robots">
        <defs>
          <marker id="goal" markerWidth="6" markerHeight="6" refX="3" refY="3"><circle cx="3" cy="3" r="2" fill="none" stroke="#f0efec" strokeWidth="0.6" /></marker>
        </defs>
        {/* floor */}
        <rect x={b.x_min} y={fy(b.y_max)} width={b.x_max - b.x_min} height={b.y_max - b.y_min} fill="#222221" stroke="#383835" strokeWidth="0.06" />
        <text x={0} y={fy(5.2)} className="map-label" textAnchor="middle">north cross-aisle</text>
        <text x={0} y={fy(-3.85)} className="map-label" textAnchor="middle">south cross-aisle · robot bays</text>

        {/* aisle zones (reservations) */}
        {layout.zones.map(z => {
          const zs = zoneState[z.id] || {}
          const owner = zs.owner
          const col = owner ? colorOf(owner) : null
          return (
            <g key={z.id}>
              <rect x={z.x0} y={fy(z.y1)} width={z.x1 - z.x0} height={z.y1 - z.y0}
                fill={col || '#2c2c2a'} fillOpacity={owner ? 0.28 : 0.35}
                stroke={col || '#383835'} strokeWidth="0.05" strokeDasharray={owner ? '' : '0.15 0.12'} />
              <text x={(z.x0 + z.x1) / 2} y={fy((z.y0 + z.y1) / 2) + 0.05} className="map-zone" textAnchor="middle">{z.id}</text>
              {zs.queue && zs.queue.length > 0 && (
                <text x={(z.x0 + z.x1) / 2} y={fy((z.y0 + z.y1) / 2) + 0.5} className="map-queue" textAnchor="middle">
                  ⏳ {zs.queue.map(r => ROBOT_SHORT[r] || r).join(' ')}
                </text>
              )}
            </g>
          )
        })}

        {/* shelf blocks */}
        {layout.blocks.map(bl => (
          <g key={bl.id}>
            <rect x={bl.x0} y={fy(bl.y1)} width={bl.x1 - bl.x0} height={bl.y1 - bl.y0} fill="#3a3f48" stroke="#5b6270" strokeWidth="0.05" rx="0.04" />
            <text x={(bl.x0 + bl.x1) / 2} y={fy(0)} className="map-block" textAnchor="middle" transform={`rotate(-90 ${(bl.x0 + bl.x1) / 2} ${fy(0)})`}>{bl.id}</text>
          </g>
        ))}
        {layout.shelves.map(s => (
          <text key={s.id} x={s.face_x + (s.side === 'W' ? 0.17 : -0.17)} y={fy(-3.2)} className="map-shelf" textAnchor="middle">{s.id}</text>
        ))}

        {/* packing stations */}
        {layout.stations.map(s => {
          const st = stationState[s.id] || {}
          const col = st.owner ? colorOf(st.owner) : null
          return (
            <g key={s.id}>
              <rect x={s.x - s.width / 2} y={fy(s.y1)} width={s.width} height={s.y1 - s.y0}
                fill={col || '#1f4d3a'} fillOpacity={col ? 0.55 : 0.9} stroke={col || '#34a37a'} strokeWidth="0.06" rx="0.05" />
              <text x={s.x} y={fy((s.y0 + s.y1) / 2) + 0.07} className="map-station" textAnchor="middle">{s.id}</text>
              {st.queue && st.queue.length > 0 && (
                <text x={s.x} y={fy(s.y1) - 0.2} className="map-queue" textAnchor="middle">⏳ {st.queue.map(r => ROBOT_SHORT[r] || r).join(' ')}</text>
              )}
            </g>
          )
        })}

        {/* bays */}
        {layout.bays.map(bay => (
          <rect key={bay.robot} x={bay.x - 0.32} y={fy(bay.y) - 0.32} width={0.64} height={0.64} fill="none"
            stroke={colorOf(bay.robot)} strokeOpacity="0.55" strokeWidth="0.04" strokeDasharray="0.12 0.1" rx="0.05" />
        ))}

        {/* obstacles */}
        {layout.obstacles.map(o => o.shape === 'box'
          ? <rect key={o.name} x={o.x - o.sx / 2} y={fy(o.y) - o.sy / 2} width={o.sx} height={o.sy} fill="#6b4f2e" stroke="#9a7a4e" strokeWidth="0.04" />
          : <circle key={o.name} cx={o.x} cy={fy(o.y)} r={o.sx} fill="#8a3a22" stroke="#c2603e" strokeWidth="0.04" />)}

        {/* items (live positions from the simulator, so carried items move with the gripper) */}
        {layout.items.map(it => {
          const p = items[it.id]
          const x = p ? p[0] : it.x, y = p ? p[1] : it.y
          const [r, g, bl] = it.rgba
          const fill = `rgb(${Math.round(r * 255)},${Math.round(g * 255)},${Math.round(bl * 255)})`
          return it.shape === 'box'
            ? <rect key={it.id} x={x - 0.09} y={fy(y) - 0.09} width="0.18" height="0.18" fill={fill} stroke="#0d0d0d" strokeWidth="0.025" />
            : <circle key={it.id} cx={x} cy={fy(y)} r="0.1" fill={fill} stroke="#0d0d0d" strokeWidth="0.025" />
        })}

        {/* paths + goals */}
        {robots.map(r => {
          const col = colorOf(r.id)
          const pts = []
          for (let i = 0; i + 1 < (r.path || []).length; i += 2) pts.push(`${r.path[i]},${fy(r.path[i + 1])}`)
          return (
            <g key={'p' + r.id}>
              {pts.length > 1 && <polyline points={pts.join(' ')} fill="none" stroke={col} strokeWidth="0.06" strokeDasharray="0.18 0.12" strokeOpacity="0.9" />}
              {r.goal && (
                <g>
                  <circle cx={r.goal[0]} cy={fy(r.goal[1])} r="0.2" fill="none" stroke={col} strokeWidth="0.05" />
                  <path d={`M${r.goal[0] - 0.14} ${fy(r.goal[1]) - 0.14} L${r.goal[0] + 0.14} ${fy(r.goal[1]) + 0.14} M${r.goal[0] + 0.14} ${fy(r.goal[1]) - 0.14} L${r.goal[0] - 0.14} ${fy(r.goal[1]) + 0.14}`} stroke={col} strokeWidth="0.05" />
                </g>
              )}
            </g>
          )
        })}

        {/* robots */}
        {robots.map(r => {
          const col = colorOf(r.id)
          const deg = -(r.yaw * 180) / Math.PI
          const waiting = r.state?.startsWith('WAITING')
          const bad = r.state === 'FAULTED' || !r.healthy
          return (
            <g key={r.id} onMouseEnter={() => setHover(r.id)} onMouseLeave={() => setHover(null)} style={{ cursor: 'default' }}>
              <g transform={`translate(${r.x} ${fy(r.y)}) rotate(${deg})`}>
                <rect x={-0.25} y={-0.22} width={0.5} height={0.44} rx="0.07" fill={col} stroke="#0d0d0d" strokeWidth="0.05" opacity={r.online === false ? 0.35 : 1} />
                <path d="M0.12 -0.12 L0.38 0 L0.12 0.12 Z" fill="#f0efec" stroke="#0d0d0d" strokeWidth="0.03" />
                <circle cx={0.02} cy={0} r={0.07} fill="#0d0d0d" opacity="0.7" />
                {r.gripper_status === 'HOLDING' && <rect x={0.24} y={-0.07} width={0.14} height={0.14} fill="#f0efec" stroke="#0d0d0d" strokeWidth="0.02" />}
              </g>
              <text x={r.x} y={fy(r.y) - 0.38} className="map-robot" textAnchor="middle">{ROBOT_SHORT[r.id] || r.id}</text>
              {(waiting || bad) && (
                <text x={r.x} y={fy(r.y) + 0.65} className={bad ? 'map-flag crit' : 'map-flag warn'} textAnchor="middle">{bad ? '✖ FAULT' : '⏳ WAITING'}</text>
              )}
              {hover === r.id && (
                <g transform={`translate(${Math.min(r.x + 0.5, 2.6)} ${fy(r.y) - 1.0})`}>
                  <rect width="2.5" height="0.95" rx="0.08" fill="#0d0d0d" stroke="#383835" strokeWidth="0.03" />
                  <text x="0.12" y="0.28" className="map-tip">{r.id} · {r.state}</text>
                  <text x="0.12" y="0.52" className="map-tip dim">{r.order_id || 'no order'} · v {r.speed.toFixed(2)} m/s</text>
                  <text x="0.12" y="0.76" className="map-tip dim">({r.x.toFixed(2)}, {r.y.toFixed(2)})</text>
                </g>
              )}
            </g>
          )
        })}
      </svg>
      <div className="map-legend">
        {robots.map(r => <span key={r.id}><i style={{ background: colorOf(r.id) }} />{r.id}</span>)}
        <span><i className="dash" />Nav2 path</span>
        <span><i className="ring" />goal</span>
        <span><i style={{ background: '#2c2c2a', border: '1px solid #898781' }} />free zone</span>
        <span><i style={{ background: '#d95926', opacity: 0.5 }} />reserved zone (robot colour)</span>
        <span>⏳ queued for access</span>
      </div>
    </div>
  )
}
