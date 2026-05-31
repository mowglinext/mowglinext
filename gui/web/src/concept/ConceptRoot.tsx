import {type ComponentType, useState} from "react";
import {AnimatePresence, motion} from "framer-motion";

import "./concept.css";

import {DashboardHome} from "./screens/DashboardHome";
import {GardenMap} from "./screens/GardenMap";
import {QuickControls} from "./screens/QuickControls";
import {ScheduleZones} from "./screens/ScheduleZones";
import {Statistics} from "./screens/Statistics";

import {BottomNav, type Screen} from "./components/BottomNav";
import {SideRail} from "./components/SideRail";
import {useViewport} from "./useViewport";

/**
 * Concept shell. Mounts a `data-concept` scope so the tokens + CSS only
 * apply inside this subtree. Adapts the chrome to viewport size:
 *   < 768px  -> fixed bottom-nav, full-bleed content
 *   ≥ 768px  -> fixed left side-rail (88px), content offset
 */

const SCREENS: Record<Screen, ComponentType> = {
  home:     DashboardHome,
  map:      GardenMap,
  controls: QuickControls,
  schedule: ScheduleZones,
  stats:    Statistics,
};

export function ConceptRoot() {
  const [screen, setScreen] = useState<Screen>("home");
  const vp = useViewport();
  const Screen = SCREENS[screen];

  return (
    <div data-concept style={{
      minHeight: "100dvh",
      background: "var(--bg-deep)",
      color: "var(--ink)",
    }}>
      {vp.isAtLeastTablet && <SideRail active={screen} onChange={setScreen}/>}

      <main style={{
        paddingLeft: vp.isAtLeastTablet ? 88 : 0,
        paddingBottom: vp.isAtLeastTablet ? 0 : 0,
        minHeight: "100dvh",
      }}>
        <AnimatePresence mode="wait">
          <motion.div
            key={screen}
            initial={{opacity: 0, y: 12}}
            animate={{opacity: 1, y: 0}}
            exit={{opacity: 0, y: -8}}
            transition={{duration: 0.35, ease: [0.2, 0.7, 0.2, 1]}}
          >
            <Screen/>
          </motion.div>
        </AnimatePresence>
      </main>

      {!vp.isAtLeastTablet && <BottomNav active={screen} onChange={setScreen}/>}
    </div>
  );
}

export default ConceptRoot;
