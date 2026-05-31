/** Tiny SVG noise overlay to take the slick edge off pure dark canvases. */
export function NoiseTexture() {
  return (
    <svg aria-hidden style={{
      position: "absolute", inset: 0, pointerEvents: "none",
      opacity: 0.04, mixBlendMode: "overlay", zIndex: 0,
    }} width="100%" height="100%">
      <filter id="concept-noise">
        <feTurbulence type="fractalNoise" baseFrequency="0.85" numOctaves="2"/>
        <feColorMatrix type="saturate" values="0"/>
      </filter>
      <rect width="100%" height="100%" filter="url(#concept-noise)"/>
    </svg>
  );
}
