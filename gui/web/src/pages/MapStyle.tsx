// Draw styles for mapbox-gl-draw
// Polygon coloring is driven by user_feature_type property (set via userProperties: true).
// Line/point coloring prefers user_color when available (paths, heading lines, mower, dock).
//
// Feature type color palette — MowgliNext brand tokens.
// This is a plain constant module (no React context), so brand hex values are
// inlined here as literals with traceability comments back to the palette.
const green  = '#7CFFB2';                // brand: lime — workarea / active
const white  = '#ECFFF4';                // brand: ink — navigation
const black  = '#02110D';                // brand: deep — obstacle
const orange = '#F3A85C';                // brand: amber — active / draft / default accent
const gray   = 'rgba(236,255,244,0.42)'; // brand: ink @ 0.42 — inactive line fallback
const mint   = '#45D688';                // brand: mint — secondary / fill green (reserved)

// Mapbox expression that resolves a polygon STROKE color from user_feature_type.
// Workarea strokes read in the lime hero accent.
const type_color: unknown[] = [
    'case',
    ['==', ['get', 'user_feature_type'], 'workarea'],   green,
    ['==', ['get', 'user_feature_type'], 'navigation'], white,
    ['==', ['get', 'user_feature_type'], 'obstacle'],   black,
    orange, // draft / active / unknown
];

// Polygon FILL color from user_feature_type. Workarea fills use the softer
// mint (secondary green) so the lime stroke stays the dominant edge cue.
const type_fill_color: unknown[] = [
    'case',
    ['==', ['get', 'user_feature_type'], 'workarea'],   mint,
    ['==', ['get', 'user_feature_type'], 'navigation'], white,
    ['==', ['get', 'user_feature_type'], 'obstacle'],   black,
    orange, // draft / active / unknown
];

export const MapStyle = [
    // ── Polygon fills ──────────────────────────────────────────────────

    // Active polygon fill – orange at higher opacity
    {
        id: 'gl-draw-polygon-fill-active',
        type: 'fill',
        filter: ['all', ['==', 'active', 'true'], ['==', '$type', 'Polygon']],
        paint: {
            'fill-color': orange,
            'fill-outline-color': orange,
            'fill-opacity': 0.45,
        },
    },

    // Inactive polygon fill – colored by feature_type
    {
        id: 'gl-draw-polygon-fill-inactive',
        type: 'fill',
        filter: ['all', ['==', 'active', 'false'], ['==', '$type', 'Polygon']],
        paint: {
            'fill-color': type_fill_color,
            'fill-outline-color': type_color,
            'fill-opacity': 0.25,
        },
    },

    // ── Polygon strokes ────────────────────────────────────────────────

    // Active polygon stroke – solid orange
    {
        id: 'gl-draw-polygon-stroke-active',
        type: 'line',
        filter: ['all', ['==', 'active', 'true'], ['==', '$type', 'Polygon']],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': orange,
            'line-width': 2.5,
        },
    },

    // Inactive polygon stroke – colored by feature_type, dashed
    {
        id: 'gl-draw-polygon-stroke-inactive',
        type: 'line',
        filter: ['all', ['==', 'active', 'false'], ['==', '$type', 'Polygon']],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': type_color,
            'line-dasharray': [3, 1.5],
            'line-width': 2,
        },
    },

    // ── Lines ──────────────────────────────────────────────────────────

    // Inactive line with user_color AND user_width (path features with custom width)
    {
        id: 'gl-draw-line-color-width',
        type: 'line',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'LineString'],
            ['==', 'meta', 'feature'],
            ['has', 'user_color'],
            ['has', 'user_width'],
        ],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': ['get', 'user_color'],
            'line-width': ['get', 'user_width'],
        },
    },

    // Inactive line with user_color only
    {
        id: 'gl-draw-line-color',
        type: 'line',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'LineString'],
            ['==', 'meta', 'feature'],
            ['has', 'user_color'],
            ['!has', 'user_width'],
        ],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': ['get', 'user_color'],
            'line-width': 2,
        },
    },

    // Inactive line without user_color – gray fallback
    {
        id: 'gl-draw-line-inactive',
        type: 'line',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'LineString'],
            ['==', 'meta', 'feature'],
            ['!has', 'user_color'],
        ],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': gray,
            'line-dasharray': [3, 2],
            'line-width': 2,
        },
    },

    // Active line – orange dashed
    {
        id: 'gl-draw-line-active',
        type: 'line',
        filter: ['all',
            ['==', '$type', 'LineString'],
            ['==', 'active', 'true'],
        ],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': orange,
            'line-dasharray': [0.5, 2],
            'line-width': 2.5,
        },
    },

    // ── Midpoints ──────────────────────────────────────────────────────
    {
        id: 'gl-draw-polygon-midpoint',
        type: 'circle',
        filter: ['all',
            ['==', '$type', 'Point'],
            ['==', 'meta', 'midpoint'],
        ],
        paint: {
            'circle-radius': 4,
            'circle-color': orange,
        },
    },

    // ── Vertices ───────────────────────────────────────────────────────

    // Vertex outer ring (white halo)
    {
        id: 'gl-draw-vertex-stroke',
        type: 'circle',
        filter: ['all',
            ['==', 'meta', 'vertex'],
            ['==', '$type', 'Point'],
            ['!=', 'mode', 'static'],
        ],
        paint: {
            'circle-radius': 6,
            'circle-color': white,
        },
    },

    // Vertex inner fill
    {
        id: 'gl-draw-vertex',
        type: 'circle',
        filter: ['all',
            ['==', 'meta', 'vertex'],
            ['==', '$type', 'Point'],
            ['!=', 'mode', 'static'],
        ],
        paint: {
            'circle-radius': 3.5,
            'circle-color': orange,
        },
    },

    // ── Point features ─────────────────────────────────────────────────

    // Active point – outer halo
    {
        id: 'gl-draw-point-stroke-active',
        type: 'circle',
        filter: ['all',
            ['==', '$type', 'Point'],
            ['==', 'active', 'true'],
            ['!=', 'meta', 'midpoint'],
        ],
        paint: {
            'circle-radius': 9,
            'circle-color': white,
        },
    },

    // Active point – inner fill
    {
        id: 'gl-draw-point-active',
        type: 'circle',
        filter: ['all',
            ['==', '$type', 'Point'],
            ['==', 'active', 'true'],
            ['!=', 'meta', 'midpoint'],
        ],
        paint: {
            'circle-radius': 6,
            'circle-color': orange,
        },
    },

    // Inactive point with user_color – outer white ring
    {
        id: 'gl-draw-point-stroke-color',
        type: 'circle',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'Point'],
            ['==', 'meta', 'feature'],
            ['has', 'user_color'],
        ],
        paint: {
            'circle-radius': 8,
            'circle-color': white,
            'circle-opacity': 0.9,
        },
    },

    // Inactive point with user_color – colored inner fill
    {
        id: 'gl-draw-point-color',
        type: 'circle',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'Point'],
            ['==', 'meta', 'feature'],
            ['has', 'user_color'],
        ],
        paint: {
            'circle-radius': 5,
            'circle-color': ['get', 'user_color'],
        },
    },

    // Inactive point without user_color – outer ring
    {
        id: 'gl-draw-point-stroke-inactive',
        type: 'circle',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'Point'],
            ['==', 'meta', 'feature'],
            ['!has', 'user_color'],
        ],
        paint: {
            'circle-radius': 8,
            'circle-color': white,
            'circle-opacity': 0.9,
        },
    },

    // Inactive point without user_color – gray inner fill
    {
        id: 'gl-draw-point-inactive',
        type: 'circle',
        filter: ['all',
            ['==', 'active', 'false'],
            ['==', '$type', 'Point'],
            ['==', 'meta', 'feature'],
            ['!has', 'user_color'],
        ],
        paint: {
            'circle-radius': 5,
            'circle-color': gray,
        },
    },

    // ── Static mode ────────────────────────────────────────────────────
    {
        id: 'gl-draw-polygon-fill-static',
        type: 'fill',
        filter: ['all', ['==', 'mode', 'static'], ['==', '$type', 'Polygon']],
        paint: {
            'fill-color': gray,
            'fill-outline-color': gray,
            'fill-opacity': 0.1,
        },
    },
    {
        id: 'gl-draw-polygon-stroke-static',
        type: 'line',
        filter: ['all', ['==', 'mode', 'static'], ['==', '$type', 'Polygon']],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': gray,
            'line-width': 2,
        },
    },
    {
        id: 'gl-draw-line-static',
        type: 'line',
        filter: ['all', ['==', 'mode', 'static'], ['==', '$type', 'LineString']],
        layout: { 'line-cap': 'round', 'line-join': 'round' },
        paint: {
            'line-color': gray,
            'line-width': 2,
        },
    },
    {
        id: 'gl-draw-point-static',
        type: 'circle',
        filter: ['all', ['==', 'mode', 'static'], ['==', '$type', 'Point']],
        paint: {
            'circle-radius': 5,
            'circle-color': gray,
        },
    },
];
