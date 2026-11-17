import React from 'react'
import { ROBOT_COLOR, ROBOT_SHORT, STATE_STYLE, STATUS, fmtTime } from './theme.js'

export function Badge({ icon, tone, children }) {
  return <span className={`badge tone-${tone}`}><span className="badge-ic" aria-hidden>{icon}</span>{children}</span>
}

// ------------------------------------------------------------------ KPI row
export function KpiRow({ state }) {
  const m = state?.metrics || {}
  const robots = state?.robots || []
  const active = robots.filter(r => r.online !== false && r.healthy && r.state !== 'FAULTED').length
  const sys = systemStatus(state)
  const tiles = [
    ['Total orders', m.orders_total ?? 0, ''],
    ['Completed', m.completed ?? 0, 'good'],
    ['Pending', m.pending ?? 0, ''],
    ['Processing', m.processing ?? 0, ''],
    ['Late orders', m.late ?? 0, (m.late || 0) > 0 ? 'serious' : ''],
    ['Orders / min', (m.orders_per_minute ?? 0).toFixed(2), ''],
    ['Active robots', `${active} / ${robots.length}`, ''],
    ['Collisions', m.collisions ?? 0, (m.collisions || 0) > 0 ? 'critical' : 'good'],
  ]
  return (
    <div className="kpis">
      {tiles.map(([label, value, tone]) => (
        <div className="kpi" key={label}>
          <div className="kpi-label">{label}</div>
          <div className={`kpi-value ${tone ? 'tone-' + tone : ''}`}>{value}</div>
        </div>
      ))}
      <div className="kpi kpi-sys">
        <div className="kpi-label">System status</div>
        <div className={`kpi-value sys tone-${sys.tone}`}><span aria-hidden>{sys.icon}</span> {sys.label}</div>
        {sys.sub && <div className="kpi-sub">{sys.sub}</div>}
      </div>
    </div>
  )
}

export function systemStatus(state) {
  if (!state) return { label: 'CONNECTING', icon: '○', tone: 'muted', sub: 'waiting for the ROS bridge' }
  const sim = state.sim || {}
  const su = state.startup || {}
  if (sim.estop) return { label: 'E-STOP', icon: '✖', tone: 'critical', sub: 'all bases locked' }
  if (sim.state === 'RESETTING') return { label: 'RESETTING', icon: '⟳', tone: 'serious', sub: '' }
  if (sim.paused) return { label: 'PAUSED', icon: '❚❚', tone: 'warning', sub: 'simulation clock stopped' }
  if (!su.ready && sim.state !== 'RUNNING') return { label: 'STARTING', icon: '⟳', tone: 'warning', sub: (su.missing || []).slice(0, 2).join(', ') }
  if (sim.state === 'RUNNING') return { label: 'RUNNING', icon: '●', tone: 'good', sub: `RTF ${sim.rtf ?? 0}` }
  return { label: sim.state || 'IDLE', icon: '●', tone: 'muted', sub: '' }
}

// ------------------------------------------------------------------ robot cards
export function RobotCards({ state }) {
  const robots = state?.robots || []
  return (
    <div className="robots">
      {robots.map(r => <RobotCard key={r.id} r={r} />)}
    </div>
  )
}

function RobotCard({ r }) {
  const [icon, tone] = STATE_STYLE[r.state] || ['●', 'muted']
  const col = ROBOT_COLOR[r.id] || '#898781'
  const batt = Math.max(0, Math.min(100, r.battery || 0))
  const row = (k, v) => <div className="kv"><span>{k}</span><b>{v}</b></div>
  return (
    <div className="panel robot-card" style={{ borderTopColor: col }}>
      <div className="robot-head">
        <span className="robot-dot" style={{ background: col }} />
        <b>{r.id}</b><span className="muted"> · {ROBOT_SHORT[r.id]}</span>
        <span className="spacer" />
        <Badge icon={icon} tone={tone}>{r.state}</Badge>
      </div>
      <div className="robot-grid">
        {row('Task', r.order_id ? `${r.order_id} · ${r.item_id || ''}` : '—')}
        {row('Destination', r.destination || '—')}
        {row('Position', `${(r.x ?? 0).toFixed(2)}, ${(r.y ?? 0).toFixed(2)} m`)}
        {row('Navigation', r.nav_status)}
        {row('Arm', r.arm_status)}
        {row('Gripper', r.gripper_status)}
        {row('Distance', `${(r.distance ?? 0).toFixed(1)} m`)}
        {row('Queue', `${r.queue_length}${r.queued_orders?.length ? ' (' + r.queued_orders.join(', ') + ')' : ''}`)}
        {row('Done / failed', `${r.tasks_completed} / ${r.tasks_failed}`)}
        {row('Holds', r.held_resources?.length ? r.held_resources.join(', ') : '—')}
      </div>
      {r.waiting_for && <div className="waiting">⏳ waiting for {r.waiting_for}</div>}
      <div className="batt" title={`battery estimate ${batt.toFixed(0)} %`}>
        <span className="muted small">Battery (estimate)</span>
        <div className="batt-bar"><div style={{ width: `${batt}%`, background: batt < 20 ? STATUS.critical : col }} /></div>
        <span className="small">{batt.toFixed(0)}%</span>
      </div>
    </div>
  )
}

// ------------------------------------------------------------------ orders
const ORDER_TONE = { PENDING: ['◌', 'muted'], ASSIGNED: ['◆', 'info'], IN_PROGRESS: ['➜', 'info'], COMPLETED: ['✔', 'good'], FAILED: ['✖', 'critical'] }

export function OrdersTable({ state }) {
  const simT = state?.sim?.sim_time || 0
  const rows = [...(state?.orders || [])].sort((a, b) => {
    const act = (o) => (['PENDING', 'ASSIGNED', 'IN_PROGRESS'].includes(o.status) ? 0 : 1)
    return act(a) - act(b) || b.arrival - a.arrival
  }).slice(0, 40)
  return (
    <div className="panel">
      <div className="panel-title"><span>Orders</span><span className="muted small">{rows.length} shown · live from the order manager</span></div>
      <div className="table-wrap">
        <table>
          <thead><tr><th>Order</th><th>Object</th><th>Shelf</th><th>Station</th><th>Robot</th><th>Deadline</th><th>Status</th><th>Completed in</th></tr></thead>
          <tbody>
            {rows.map(o => {
              const [ic, tone] = ORDER_TONE[o.status] || ['●', 'muted']
              const open = ['PENDING', 'ASSIGNED', 'IN_PROGRESS'].includes(o.status)
              const left = o.deadline - simT
              const late = open ? left < 0 : o.completed > o.deadline
              return (
                <tr key={o.id}>
                  <td className="mono">{o.id}</td><td>{o.item}</td><td>{o.shelf}</td><td>{o.station}</td>
                  <td>{o.robot ? <span><i className="dot" style={{ background: ROBOT_COLOR[o.robot] }} />{o.robot}</span> : '—'}</td>
                  <td className={late ? 'tone-serious' : ''}>{open ? (late ? `⚠ late ${fmtTime(-left)}` : `in ${fmtTime(left)}`) : fmtTime(o.deadline - o.arrival) + ' window'}</td>
                  <td><Badge icon={ic} tone={tone}>{o.status}{o.attempts > 0 ? ` (try ${o.attempts + 1})` : ''}</Badge></td>
                  <td className="mono">{o.status === 'COMPLETED' ? `${fmtTime(o.completed - o.arrival)}${late ? ' ⚠' : ''}` : '–'}</td>
                </tr>
              )
            })}
            {rows.length === 0 && <tr><td colSpan="8" className="muted center">no orders yet</td></tr>}
          </tbody>
        </table>
      </div>
    </div>
  )
}

// ------------------------------------------------------------------ traffic
export function TrafficPanel({ state }) {
  const t = state?.traffic || { zones: [], stations: [] }
  const events = (state?.events || []).filter(e => ['reservation', 'wait', 'release', 'deadlock_prevented', 'recovery', 'nav_failure'].includes(e.category)).slice(-40).reverse()
  const all = [...t.zones, ...t.stations]
  const waiting = all.filter(z => z.queue?.length)
  return (
    <div className="panel">
      <div className="panel-title"><span>Traffic monitoring</span>
        <span className="muted small">{t.deadlock_preventions} deadlock preventions · {t.recoveries} recoveries</span></div>
      <div className="res-grid">
        {all.map(z => (
          <div key={z.id} className={`res ${z.owner ? 'owned' : ''}`} style={z.owner ? { borderColor: ROBOT_COLOR[z.owner] } : null}>
            <div className="res-id">{z.id}</div>
            <div className="res-owner">{z.owner ? <span><i className="dot" style={{ background: ROBOT_COLOR[z.owner] }} />{ROBOT_SHORT[z.owner] || z.owner}</span> : <span className="muted">free</span>}</div>
            {z.queue?.length > 0 && <div className="res-q">⏳ {z.queue.map(q => ROBOT_SHORT[q] || q).join(', ')}</div>}
          </div>
        ))}
      </div>
      <div className="sub-title">Robots waiting for access: {waiting.length ? waiting.map(z => `${z.queue.map(q => ROBOT_SHORT[q] || q).join(',')} → ${z.id}`).join(' · ') : 'none'}</div>
      <div className="log small">
        {events.length === 0 && <div className="muted">no traffic events yet</div>}
        {events.map(e => <EventLine key={e.seq} e={e} />)}
      </div>
    </div>
  )
}

const SEV_ICON = { info: 'ℹ', warn: '⚠', error: '✖' }
export function EventLine({ e }) {
  return (
    <div className={`ev sev-${e.severity}`}>
      <span className="ev-t mono">{fmtTime(e.t)}</span>
      <span className="ev-ic" aria-hidden>{SEV_ICON[e.severity] || 'ℹ'}</span>
      <span className="ev-cat">{e.category.replace('_', ' ')}</span>
      <span className="ev-d">{e.detail}</span>
    </div>
  )
}

export function EventLog({ state }) {
  const events = [...(state?.events || [])].reverse().slice(0, 60)
  return (
    <div className="panel">
      <div className="panel-title"><span>Event log</span><span className="muted small">orders · collisions · recoveries</span></div>
      <div className="log">{events.map(e => <EventLine key={e.seq} e={e} />)}</div>
    </div>
  )
}
