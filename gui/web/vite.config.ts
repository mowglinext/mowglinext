import {defineConfig} from 'vite'
import react from '@vitejs/plugin-react'
import {randomUUID} from 'node:crypto'

const API_TARGET = process.env.MOWGLI_API_TARGET || 'http://localhost:4006';
const targetUrl = new URL(API_TARGET);
const targetOrigin = `${targetUrl.protocol}//${targetUrl.host}`;

// The web build has its own identity: a web-only patch need not rebuild Go.
// Embed the same ID in the client and its served manifest, including rebuilds
// from the same source revision with different assets/configuration.
const webBuild = {id: randomUUID(), revision: process.env.VITE_BUILD_REVISION || '',
    version: process.env.VITE_BUILD_VERSION || '', built_at: process.env.VITE_BUILD_TIME || ''};

// https://vitejs.dev/config/
export default defineConfig({
    define: {'import.meta.env.VITE_WEB_BUILD_ID': JSON.stringify(webBuild.id)},
    plugins: [
        react(),
        {
            name: 'served-web-build',
            generateBundle() {
                this.emitFile({type: 'asset', fileName: 'web-build.json', source: JSON.stringify(webBuild)});
            },
            configureServer(server) {
                server.middlewares.use('/web-build.json', (_request, response) => {
                    response.setHeader('Content-Type', 'application/json');
                    response.setHeader('Cache-Control', 'no-store');
                    response.end(JSON.stringify(webBuild));
                });
            },
        },
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
