import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Production: `npm run build` -> dist/, served by the ROS dashboard backend on http://localhost:3000
// Development: `npm run dev` (port 3001) proxies API + WebSocket to the running backend on :3000
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://localhost:3000',
      '/ws': { target: 'ws://localhost:3000', ws: true },
    },
  },
  build: { outDir: 'dist', chunkSizeWarningLimit: 1200 },
})
