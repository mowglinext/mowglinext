import {defineConfig} from 'vite'
import react from '@vitejs/plugin-react'

const API_TARGET = process.env.MOWGLI_API_TARGET || 'http://localhost:4006';
const targetUrl = new URL(API_TARGET);
const targetOrigin = `${targetUrl.protocol}//${targetUrl.host}`;

// https://vitejs.dev/config/
export default defineConfig({
    plugins: [
        react(),
    ],
    server: {
        host: '0.0.0.0',
        proxy: {
            '/api': {
                target: API_TARGET,
                ws: true,
                changeOrigin: true,
                // Robot's Go backend gates WebSocket upgrades on the Origin
                // header. Rewrite it so cross-origin dev (frontend on :5173,
                // backend on the robot) passes the check.
                headers: {
                    Origin: targetOrigin,
                },
                configure: (proxy) => {
                    proxy.on('proxyReqWs', (proxyReq: { setHeader?: (k: string, v: string) => void; }) => {
                        proxyReq.setHeader?.('Origin', targetOrigin);
                        proxyReq.setHeader?.('Host', targetUrl.host);
                    });
                },
            }
        }
    }
})
