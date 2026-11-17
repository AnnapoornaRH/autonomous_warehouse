import { useEffect, useRef, useState, useCallback } from 'react'

// Live data hook: one WebSocket to the ROS bridge (/ws) + the static layout from /api/layout.
// Nothing here is simulated: if the backend is down the UI shows "disconnected" and keeps the last real data.
export function useWarehouse() {
  const [state, setState] = useState(null)
  const [history, setHistory] = useState([])
  const [layout, setLayout] = useState(null)
  const [connected, setConnected] = useState(false)
  const wsRef = useRef(null)
  const retry = useRef(0)

  useEffect(() => {
    let stop = false
    fetch('/api/layout').then(r => r.json()).then(setLayout).catch(() => {})
    fetch('/api/history').then(r => r.json()).then(setHistory).catch(() => {})

    const connect = () => {
      if (stop) return
      const proto = location.protocol === 'https:' ? 'wss' : 'ws'
      const ws = new WebSocket(`${proto}://${location.host}/ws`)
      wsRef.current = ws
      ws.onopen = () => { retry.current = 0; setConnected(true) }
      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data)
          if (msg.type === 'state') setState(msg)
          else if (msg.type === 'history') setHistory(msg.history)
        } catch (e) { /* ignore malformed frame */ }
      }
      ws.onclose = () => {
        setConnected(false)
        if (!stop) setTimeout(connect, Math.min(5000, 500 * 2 ** retry.current++))
      }
      ws.onerror = () => ws.close()
    }
    connect()
    return () => { stop = true; wsRef.current && wsRef.current.close() }
  }, [])

  return { state, history, layout, connected }
}

export async function post(path, body) {
  const res = await fetch(path, {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}),
  })
  const data = await res.json().catch(() => ({}))
  if (!res.ok) throw new Error(data.detail || res.statusText)
  return data
}

export function useToast() {
  const [toast, setToast] = useState(null)
  const show = useCallback((text, kind = 'info') => {
    setToast({ text, kind, id: Date.now() })
    setTimeout(() => setToast(t => (t && Date.now() - t.id >= 2900 ? null : t)), 3000)
  }, [])
  return [toast, show]
}
