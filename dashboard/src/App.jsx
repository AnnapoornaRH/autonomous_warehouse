import React, { useState } from 'react'
import { useWarehouse, post, useToast } from './useWarehouse.js'
import WarehouseMap from './WarehouseMap.jsx'
import Charts from './Charts.jsx'
import OrderConsole from './OrderConsole.jsx'
import { KpiRow, RobotCards, OrdersTable, TrafficPanel, EventLog, systemStatus, Badge } from './Panels.jsx'
import { fmtTime } from './theme.js'

export default function App() {
  const { state, history, layout, connected } = useWarehouse()
  const [toast, show] = useToast()
  const [busy, setBusy] = useState('')
  const [count, setCount] = useState(5)
  const [rate, setRate] = useState(2)
  const sys = systemStatus(state)
  const sim = state?.sim || {}

  const run = async (label, fn) => {
    setBusy(label)
    try { const r = await fn(); show(r.message || 'done', 'ok') } catch (e) { show(`${label}: ${e.message}`, 'err') }
    setBusy('')
  }
  const control = (command, label) => () => run(label, () => post('/api/control', { command }))

  return (
    <div className="app">
      <header className="top">
        <div className="brand">
          <div className="logo"><i style={{ background: '#3987e5' }} /><i style={{ background: '#d95926' }} /><i style={{ background: '#199e70' }} /></div>
          <div><h1>Autonomous Warehouse</h1><div className="muted small">ROS 2 · Gazebo · Nav2 · fleet control center</div></div>
        </div>
        <div className="top-stats">
          <Badge icon={connected ? '●' : '○'} tone={connected ? 'good' : 'critical'}>{connected ? 'live' : 'disconnected'}</Badge>
          <Badge icon={sys.icon} tone={sys.tone}>{sys.label}</Badge>
          <span className="stat"><span className="muted">sim time</span> <b className="mono">{fmtTime(sim.sim_time)}</b></span>
          <span className="stat"><span className="muted">RTF</span> <b className="mono">{(sim.rtf ?? 0).toFixed(2)}</b></span>
        </div>
      </header>

      <section className="controls panel">
        <div className="btn-group">
          <button className="btn good" disabled={!!busy} onClick={control('start', 'start')}>▶ Start</button>
          <button className="btn" disabled={!!busy || sim.paused} onClick={control('pause', 'pause')}>❚❚ Pause</button>
          <button className="btn" disabled={!!busy || !sim.paused} onClick={control('resume', 'resume')}>▶ Resume</button>
          <button className="btn warn" disabled={!!busy} onClick={() => { if (confirm('Reset the simulation? Orders and metrics are cleared (CSV of this run is saved first).')) control('reset', 'reset')() }}>⟳ Reset</button>
          {sim.estop
            ? <button className="btn estop released" onClick={control('release_estop', 'release e-stop')}>⏏ Release E-STOP</button>
            : <button className="btn estop" onClick={control('estop', 'emergency stop')}>✖ EMERGENCY STOP</button>}
        </div>
        <div className="btn-group">
          <label className="field">orders <input type="number" min="1" max="20" value={count} onChange={e => setCount(+e.target.value)} /></label>
          <button className="btn" disabled={!!busy} onClick={() => run('generate', () => post('/api/orders/generate', { count }))}>＋ Generate test orders</button>
        </div>
        <div className="btn-group">
          <label className="field">arrival rate <input type="range" min="0" max="8" step="0.5" value={rate} onChange={e => setRate(+e.target.value)} /> <b className="mono">{rate.toFixed(1)}/min</b></label>
          <button className="btn" disabled={!!busy} onClick={() => run('set rate', () => post('/api/orders/rate', { orders_per_minute: rate }))}>Apply</button>
        </div>
        <div className="btn-group">
          <button className="btn" onClick={() => run('export', () => post('/api/export'))}>⇩ Export CSV</button>
          <a className="btn" href="/api/metrics.csv" download>metrics.csv</a>
          <a className="btn" href="/api/orders.csv" download>orders.csv</a>
        </div>
      </section>

      {!state?.startup?.ready && state && sim.state !== 'RUNNING' && (
        <div className="banner">System is starting – waiting for: {(state.startup?.missing || []).join(', ') || '…'}</div>
      )}

      <OrderConsole state={state} layout={layout} connected={connected} onNotice={show} />

      <KpiRow state={state} />

      <main className="grid">
        <div className="col-map"><WarehouseMap layout={layout} state={state} /></div>
        <div className="col-side">
          <RobotCards state={state} />
        </div>
      </main>

      <section className="two">
        <OrdersTable state={state} />
        <TrafficPanel state={state} />
      </section>

      <h2 className="section">Performance analytics</h2>
      <Charts history={history} state={state} />

      <section className="one"><EventLog state={state} /></section>

      {toast && <div className={`toast ${toast.kind}`}>{toast.text}</div>}
    </div>
  )
}
