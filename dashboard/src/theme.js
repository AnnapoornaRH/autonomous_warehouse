// Colour tokens (dark surface).  Robot colours are categorical slots 1-3 of the validated palette
// (blue / orange / aqua: all-pairs CVD ΔE >= 9.4, normal-vision ΔE >= 20.9, >= 3:1 on #1a1a19) and are the SAME
// colours as the robot bodies in Gazebo.  Status colours are the fixed status palette and are always paired
// with an icon and a text label.
export const ROBOT_COLOR = { robot1: '#3987e5', robot2: '#d95926', robot3: '#199e70' }
export const ROBOT_SHORT = { robot1: 'R1', robot2: 'R2', robot3: 'R3' }
export const STATUS = { good: '#0ca30c', warning: '#fab219', serious: '#ec835a', critical: '#d03b3b' }
export const INK = { primary: '#f0efec', secondary: '#c3c2b7', muted: '#898781', grid: '#2c2c2a', axis: '#383835' }

// state -> [icon, label tone]
export const STATE_STYLE = {
  IDLE: ['●', 'muted'],
  ASSIGNED: ['◆', 'info'],
  NAVIGATING_TO_SHELF: ['➜', 'info'],
  WAITING_FOR_AISLE: ['⏳', 'warning'],
  POSITIONING_FOR_PICKUP: ['◎', 'info'],
  PICKING: ['✋', 'info'],
  NAVIGATING_TO_PACKING: ['➜', 'info'],
  WAITING_FOR_PACKING: ['⏳', 'warning'],
  DROPPING: ['⬇', 'info'],
  COMPLETED: ['✔', 'good'],
  RECOVERING: ['⟳', 'serious'],
  FAULTED: ['✖', 'critical'],
  OFFLINE: ['○', 'muted'],
}

export const fmtTime = (s) => {
  if (s === undefined || s === null || isNaN(s)) return '–'
  const sign = s < 0 ? '-' : ''
  s = Math.abs(Math.round(s))
  return `${sign}${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`
}
