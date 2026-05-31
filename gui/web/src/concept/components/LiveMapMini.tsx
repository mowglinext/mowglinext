import {motion} from "framer-motion";

/**
 * Dashboard-scale live garden view. Quiet polygon, soft fill, single
 * focal robot dot with a Gaussian-blur halo + sonar pulse, contour
 * lines as ambient texture. No labels inside the canvas -- the data
 * lives in the surrounding glass card.
 */

interface LiveMapMiniProps {
  /** Polygon in normalised units (0..1). Designed for the demo path. */
  polygon?: {x: number; y: number}[];
  /** Robot position 0..1. */
  robot?:   {x: number; y: number; heading: number};
  /** Coverage fraction 0..1 -- drawn as a shaded overlay band. */
  coverage?: number;
  height?: number;
}

const DEFAULT_POLY = [
  {x: 0.18, y: 0.20},
  {x: 0.62, y: 0.10},
  {x: 0.86, y: 0.32},
  {x: 0.82, y: 0.68},
  {x: 0.55, y: 0.84},
  {x: 0.22, y: 0.74},
  {x: 0.12, y: 0.46},
];

export function LiveMapMini({
  polygon = DEFAULT_POLY,
  robot   = {x: 0.62, y: 0.46, heading: 30},
  coverage = 0.42,
  height = 200,
}: LiveMapMiniProps) {
  const w = 600;
  const h = height * (w / 600);
  // Build polygon path
  const toX = (x: number) => x * w;
  const toY = (y: number) => y * h;
  const polyPath = polygon
    .map((p, i) => `${i === 0 ? "M" : "L"} ${toX(p.x).toFixed(1)} ${toY(p.y).toFixed(1)}`)
    .join(" ") + " Z";

  return (
    <div style={{position: "relative", width: "100%", height, overflow: "hidden"}}>
      <svg
        viewBox={`0 0 ${w} ${h}`}
        width="100%"
        height={height}
        preserveAspectRatio="xMidYMid slice"
        style={{display: "block"}}
      >
        <defs>
          <radialGradient id="lawnFill" cx="50%" cy="50%" r="55%">
            <stop offset="0%" stopColor="rgba(124, 255, 178, 0.22)"/>
            <stop offset="100%" stopColor="rgba(124, 255, 178, 0.06)"/>
          </radialGradient>
          <linearGradient id="lawnEdge" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0%" stopColor="rgba(124, 255, 178, 1)"/>
            <stop offset="100%" stopColor="rgba(69, 214, 232, 0.6)"/>
          </linearGradient>
          <filter id="robotGlow">
            <feGaussianBlur stdDeviation="6"/>
            <feComposite in2="SourceGraphic" operator="over"/>
          </filter>
        </defs>

        {/* contour atmosphere */}
        <g fill="none" stroke="rgba(124,255,178,0.07)" strokeWidth={0.7}>
          <path d={`M -20 ${h * 0.4} Q ${w * 0.3} ${h * 0.3}, ${w * 0.55} ${h * 0.42} T ${w + 20} ${h * 0.36}`}/>
          <path d={`M -20 ${h * 0.55} Q ${w * 0.35} ${h * 0.46}, ${w * 0.6} ${h * 0.6} T ${w + 20} ${h * 0.54}`}/>
          <path d={`M -20 ${h * 0.7} Q ${w * 0.4} ${h * 0.62}, ${w * 0.65} ${h * 0.76} T ${w + 20} ${h * 0.7}`}/>
        </g>

        {/* polygon -- soft fill + clean edge */}
        <motion.path
          d={polyPath}
          fill="url(#lawnFill)"
          stroke="url(#lawnEdge)"
          strokeWidth={1.4}
          strokeLinejoin="round"
          initial={{pathLength: 0, opacity: 0}}
          animate={{pathLength: 1, opacity: 1}}
          transition={{duration: 1.2, ease: [0.2, 0.7, 0.2, 1], delay: 0.2}}
        />

        {/* coverage swath -- diagonal lines clipped to polygon */}
        <defs>
          <pattern id="cov" patternUnits="userSpaceOnUse" width={8} height={8} patternTransform="rotate(35)">
            <line x1={0} y1={0} x2={0} y2={8} stroke="rgba(124,255,178,0.18)" strokeWidth={1.4}/>
          </pattern>
          <clipPath id="polyClip"><path d={polyPath}/></clipPath>
        </defs>
        <g clipPath="url(#polyClip)">
          <motion.rect
            x={0} y={0} width={w * coverage} height={h}
            fill="url(#cov)"
            initial={{opacity: 0}} animate={{opacity: 1}}
            transition={{delay: 0.7, duration: 0.6}}
          />
        </g>

        {/* robot dot */}
        <g transform={`translate(${toX(robot.x)} ${toY(robot.y)})`}>
          {/* outer glow */}
          <circle r={14} fill="rgba(124,255,178,0.55)" filter="url(#robotGlow)"/>
          {/* heading wedge */}
          <g transform={`rotate(${-robot.heading - 90})`}>
            <path d="M 0 -16 L -4 -4 L 4 -4 Z" fill="var(--lime)" opacity={0.9}/>
          </g>
          {/* core */}
          <circle r={5.5} fill="var(--bg-deep)" stroke="var(--lime)" strokeWidth={1.6}/>
          <circle r={3.2} fill="var(--lime)"/>
          {/* sonar pulse */}
          <motion.circle
            r={5.5}
            fill="none"
            stroke="rgba(124,255,178,0.65)"
            strokeWidth={1}
            initial={{r: 5.5, opacity: 0.55}}
            animate={{r: [5.5, 28], opacity: [0.55, 0]}}
            transition={{duration: 1.8, ease: "easeOut", repeat: Infinity}}
          />
        </g>

        {/* dock marker */}
        <g transform={`translate(${toX(0.16)} ${toY(0.22)})`}>
          <rect x={-6} y={-3} width={12} height={6} rx={2}
                fill="rgba(243, 168, 92, 0.14)"
                stroke="var(--amber)" strokeWidth={1}/>
          <circle cx={0} cy={0} r={1.2} fill="var(--amber)"/>
        </g>
      </svg>
    </div>
  );
}
