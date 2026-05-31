import {useEffect, useState} from "react";

/**
 * Viewport size buckets used by the concept screens. Returns a tuple of
 * booleans so call-sites stay readable: `const {isMobile, isWide} = ...`.
 *
 *   mobile    < 768
 *   tablet    768  – 1023
 *   desktop   1024 – 1439
 *   wide      ≥ 1440
 */

export interface Viewport {
  width: number;
  isMobile: boolean;
  isTablet: boolean;
  isDesktop: boolean;
  isWide: boolean;
  /** Convenience: anything ≥ 768. */
  isAtLeastTablet: boolean;
}

export function useViewport(): Viewport {
  const get = (): Viewport => {
    const w = typeof window === "undefined" ? 1280 : window.innerWidth;
    return {
      width: w,
      isMobile:  w < 768,
      isTablet:  w >= 768 && w < 1024,
      isDesktop: w >= 1024 && w < 1440,
      isWide:    w >= 1440,
      isAtLeastTablet: w >= 768,
    };
  };

  const [vp, setVp] = useState<Viewport>(get);

  useEffect(() => {
    const onResize = () => setVp(get());
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);

  return vp;
}
