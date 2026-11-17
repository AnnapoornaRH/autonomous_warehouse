import React, { useEffect, useMemo, useState } from 'react'
import { post } from './useWarehouse.js'

// Product ordering uses the same item IDs as Gazebo and the ROS 2 order manager.
// Each catalog entry is one physical object, so it cannot be ordered twice at once.
export default function OrderConsole({ state, layout, connected, onNotice }) {
  const [selected, setSelected] = useState([])
  const [station, setStation] = useState('PK1')
  const [fleet, setFleet] = useState('all')
  const [busy, setBusy] = useState(false)
  const [feedback, setFeedback] = useState('')

  const items = layout?.items || []
  const stations = layout?.stations || []
  const robots = state?.robots || []
  const active = useMemo(() => new Set((state?.orders || [])
    .filter(o => ['PENDING', 'ASSIGNED', 'IN_PROGRESS'].includes(o.status))
    .map(o => o.item_id)), [state?.orders])
  useEffect(() => {
    if (!stations.some(s => s.id === station) && stations.length) setStation(stations[0].id)
  }, [stations, station])
  useEffect(() => {
    if (state?.fleet_selection) setFleet(state.fleet_selection)
  }, [state?.fleet_selection])

  function toggle(id) {
    setFeedback('')
    setSelected(s => s.includes(id) ? s.filter(x => x !== id) : [...s, id])
  }
  async function chooseFleet(value) {
    setBusy(true)
    try {
      const r = await post('/api/fleet/selection', { robot_id: value })
      setFleet(value)
      setFeedback(r.message)
    } catch (e) { setFeedback('Fleet: ' + e.message) }
    finally { setBusy(false) }
  }
  async function placeOrder() {
    setBusy(true)
    try {
      const r = await post('/api/orders/submit', { item_ids: selected, station_id: station })
      setFeedback(`${r.message}. IDs: ${r.order_ids.join(', ')}`)
      setSelected([])
      onNotice?.(r.message, 'ok')
    } catch (e) { setFeedback('Order not placed: ' + e.message); onNotice?.(e.message, 'err') }
    finally { setBusy(false) }
  }
  const ready = connected && state?.sim?.state === 'RUNNING' && !state?.sim?.estop
  return <section className="panel order-console">
    <div className="panel-title"><span>📦 Product ordering & dispatch</span><span className="small muted">Real warehouse inventory · {selected.length} selected</span></div>
    <p className="muted small">Choose physical items, a packing station, and the robots allowed to accept <b>new</b> orders. Nav2 handles travel; your existing arm agents handle picking and placing. Existing assignments are not interrupted.</p>
    <div className="order-layout">
      <div>
        <div className="sub-title">Shelves and available items</div>
        <div className="catalog-grid">
          {items.map(item => {
            const unavailable = active.has(item.id)
            const chosen = selected.includes(item.id)
            return <button type="button" key={item.id} className={`catalog-item ${chosen ? 'chosen' : ''}`}
              disabled={busy || unavailable} aria-pressed={chosen} onClick={() => toggle(item.id)}>
              <span className="catalog-top"><b>{item.name}</b><span className="catalog-shelf">{item.shelf}</span></span>
              <span className="muted small">{item.id} · Aisle {item.shelf === 'S1' ? 'A0' : (layout?.shelves || []).find(s => s.id === item.shelf)?.aisle || '—'}</span>
              <span className={`small ${unavailable ? 'not-available' : chosen ? 'selected-item' : ''}`}>{unavailable ? 'Reserved / in progress' : chosen ? '✓ Selected' : 'Available · Click to add'}</span>
            </button>
          })}
        </div>
      </div>
      <div className="checkout">
        <div className="sub-title">Packing destination</div>
        <select aria-label="Packing station" value={station} onChange={e => setStation(e.target.value)}>
          {stations.map(s => <option key={s.id} value={s.id}>{s.id} · Packing station</option>)}
        </select>
        <div className="sub-title">Dispatch mode</div>
        <select aria-label="Robot dispatch mode" value={fleet} onChange={e => chooseFleet(e.target.value)} disabled={busy}>
          <option value="all">Fleet optimization · All available robots</option>
          {robots.map(r => <option key={r.id} value={r.id}>{r.id} only</option>)}
        </select>
        <p className="small muted">Fleet selection applies to new allocations. Run <code>./demo.sh --fleet</code> to enable all three robots.</p>
        <div className="sub-title">Selected items ({selected.length})</div>
        <div className="selected-list">
          {selected.length ? selected.map(id => {
            const item = items.find(i => i.id === id)
            return <div key={id} className="picked-line"><span>{item?.name || id} <span className="muted">({item?.shelf})</span></span><button className="link" onClick={() => toggle(id)}>Remove</button></div>
          }) : <span className="muted small">Select products from the named shelves.</span>}
        </div>
        <button className="btn good order-place" disabled={!ready || busy || !selected.length || selected.some(x => active.has(x))}
          onClick={placeOrder}>{busy ? 'Please wait…' : `Place ${selected.length} item${selected.length === 1 ? '' : 's'} → ${station}`}</button>
        {!ready && <p className="small muted">Start the simulation and connect the dashboard to place real ROS orders.</p>}
        {feedback && <div className="order-feedback" role="status">{feedback}</div>}
        <p className="small muted">One unit per physical item. The order manager rejects duplicates and unavailable items. The existing deadline-aware fleet scheduler selects robots and estimates work time; routes update through Nav2.</p>
      </div>
    </div>
  </section>
}
