import React, { useMemo, useState } from 'react'
import {
  Area, AreaChart, Bar, BarChart, CartesianGrid, Legend, Line, LineChart, ResponsiveContainer, Tooltip, XAxis, YAxis, Cell,
} from 'recharts'
import { INK, ROBOT_COLOR, STATUS, fmtTime } from './theme.js'

// Performance analytics.  Every point is a sample of the real MetricsSnapshot stream (history kept by the bridge).
// Rules applied: one y axis per chart, 2px lines, rounded bar ends, hairline grid, legend whenever >= 2 series,
// tooltips on every mark, a table view for every chart.
const FLEET = '#e8e6e1'      // neutral ink for fleet-wide aggregates (robot colours are reserved for robots)

const axisProps = { stroke: INK.axis, tick: { fill: INK.muted, fontSize: 11 }, tickLine: false }

function Tip({ active, payload, label, fmt, xfmt }) {
  if (!active || !payload?.length) return null
  return (
    <div className="tip">
      <div className="tip-h">{xfmt ? xfmt(label) : label}</div>
      {payload.map(p => (
        <div key={p.dataKey} className="tip-r"><i style={{ background: p.color || p.fill || p.stroke }} />
          <span>{p.name}</span><b>{fmt ? fmt(p.value) : p.value}</b></div>
      ))}
    </div>
  )
}

function Card({ title, sub, children, table }) {
  const [asTable, setAsTable] = useState(false)
  return (
    <div className="panel chart-card">
      <div className="panel-title"><span>{title}</span>
        <button className="link" onClick={() => setAsTable(v => !v)}>{asTable ? 'chart' : 'table'}</button></div>
      {sub && <div className="muted small sub">{sub}</div>}
      <div className="chart-body">{asTable ? <div className="table-wrap small">{table}</div> : children}</div>
    </div>
  )
}

const SimpleTable = ({ head, rows }) => (
  <table><thead><tr>{head.map(h => <th key={h}>{h}</th>)}</tr></thead>
    <tbody>{rows.map((r, i) => <tr key={i}>{r.map((c, j) => <td key={j} className="mono">{c}</td>)}</tr>)}</tbody></table>
)

export default function Charts({ history, state }) {
  const data = useMemo(() => history.map(h => ({ ...h, t: h.sim_time })), [history])
  const last = data.slice(-14).reverse()
  const m = state?.metrics || {}
  const ids = m.robot_ids || []
  const robotRows = ids.map((id, i) => ({
    id, util: Math.round((m.utilization?.[i] || 0) * 100), tasks: m.tasks_completed?.[i] || 0,
    dist: +(m.distance?.[i] || 0).toFixed(1), idle: Math.round(m.idle_time?.[i] || 0), avg: Math.round(m.avg_task_time?.[i] || 0),
  }))
  const xTick = { ...axisProps, dataKey: 't', tickFormatter: fmtTime, minTickGap: 40 }

  return (
    <div className="charts">
      <Card title="Orders completed over time" sub="cumulative, fleet total"
        table={<SimpleTable head={['sim time', 'completed', 'orders/min']} rows={last.map(h => [fmtTime(h.t), h.completed, h.orders_per_minute.toFixed(2)])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <AreaChart data={data} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...xTick} /><YAxis {...axisProps} allowDecimals={false} />
            <Tooltip content={<Tip xfmt={fmtTime} />} cursor={{ stroke: INK.muted }} />
            <Area type="monotone" dataKey="completed" name="Completed orders" stroke={FLEET} strokeWidth={2} fill={FLEET} fillOpacity={0.12} dot={false} />
          </AreaChart>
        </ResponsiveContainer>
      </Card>

      <Card title="Robot utilization" sub="% of time not idle"
        table={<SimpleTable head={['robot', 'utilization %', 'idle s']} rows={robotRows.map(r => [r.id, r.util, r.idle])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <BarChart data={robotRows} margin={{ top: 8, right: 12, left: -12, bottom: 0 }} barCategoryGap="30%">
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...axisProps} dataKey="id" /><YAxis {...axisProps} domain={[0, 100]} unit="%" />
            <Tooltip content={<Tip fmt={v => `${v} %`} />} cursor={{ fill: 'rgba(255,255,255,0.04)' }} />
            <Bar dataKey="util" name="Utilization" radius={[4, 4, 0, 0]}>
              {robotRows.map(r => <Cell key={r.id} fill={ROBOT_COLOR[r.id]} stroke="#1a1a19" strokeWidth={2} />)}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </Card>

      <Card title="Average order completion time" sub="seconds from arrival to verified delivery"
        table={<SimpleTable head={['sim time', 'avg completion s']} rows={last.map(h => [fmtTime(h.t), Math.round(h.avg_completion_time)])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <LineChart data={data} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...xTick} /><YAxis {...axisProps} unit="s" />
            <Tooltip content={<Tip fmt={v => `${Math.round(v)} s`} xfmt={fmtTime} />} cursor={{ stroke: INK.muted }} />
            <Line type="monotone" dataKey="avg_completion_time" name="Avg completion" stroke={FLEET} strokeWidth={2} dot={false} />
          </LineChart>
        </ResponsiveContainer>
      </Card>

      <Card title="Late orders" sub="completed after the deadline, or still open past it"
        table={<SimpleTable head={['sim time', 'late', 'pending', 'processing']} rows={last.map(h => [fmtTime(h.t), h.late, h.pending, h.processing])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <LineChart data={data} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...xTick} /><YAxis {...axisProps} allowDecimals={false} />
            <Tooltip content={<Tip xfmt={fmtTime} />} cursor={{ stroke: INK.muted }} />
            <Line type="stepAfter" dataKey="late" name="Late orders" stroke={FLEET} strokeWidth={2} dot={false} />
          </LineChart>
        </ResponsiveContainer>
      </Card>

      <Card title="Robot workload comparison" sub="orders delivered and distance driven per robot"
        table={<SimpleTable head={['robot', 'orders', 'distance m', 'avg task s']} rows={robotRows.map(r => [r.id, r.tasks, r.dist, r.avg])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <BarChart data={robotRows} margin={{ top: 8, right: 12, left: -12, bottom: 0 }} barCategoryGap="30%">
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...axisProps} dataKey="id" /><YAxis {...axisProps} allowDecimals={false} />
            <Tooltip content={<Tip fmt={v => v} />} cursor={{ fill: 'rgba(255,255,255,0.04)' }} />
            <Bar dataKey="tasks" name="Orders delivered" radius={[4, 4, 0, 0]}>
              {robotRows.map(r => <Cell key={r.id} fill={ROBOT_COLOR[r.id]} stroke="#1a1a19" strokeWidth={2} />)}
            </Bar>
          </BarChart>
        </ResponsiveContainer>
      </Card>

      <Card title="Collisions" sub="collisions and near misses are counted separately"
        table={<SimpleTable head={['sim time', 'collisions', 'near misses']} rows={last.map(h => [fmtTime(h.t), h.collisions, h.near_misses])} />}>
        <ResponsiveContainer width="100%" height={190}>
          <LineChart data={data} margin={{ top: 8, right: 12, left: -12, bottom: 0 }}>
            <CartesianGrid stroke={INK.grid} vertical={false} />
            <XAxis {...xTick} /><YAxis {...axisProps} allowDecimals={false} />
            <Tooltip content={<Tip xfmt={fmtTime} />} cursor={{ stroke: INK.muted }} />
            <Legend iconType="plainline" wrapperStyle={{ fontSize: 12, color: INK.secondary }} />
            <Line type="stepAfter" dataKey="collisions" name="✖ Collisions" stroke={STATUS.critical} strokeWidth={2} dot={false} />
            <Line type="stepAfter" dataKey="near_misses" name="⚠ Near misses" stroke={STATUS.warning} strokeWidth={2} dot={false} strokeDasharray="5 4" />
          </LineChart>
        </ResponsiveContainer>
      </Card>
    </div>
  )
}
