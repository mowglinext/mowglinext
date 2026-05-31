import {useState} from "react";
import {AnimatePresence, motion} from "framer-motion";

import "./concept.css";

import {DashboardHome} from "./screens/DashboardHome";
import {BottomNav, type Screen} from "./components/BottomNav";

/**
 * Concept shell. Mounts a `data-concept` scope so the tokens + CSS only
 * apply inside this subtree (the main MowgliNext app remains untouched).
 *
 * For now the bottom-nav cycles between the Dashboard and "soon"
 * placeholders for the other four screens -- the dashboard is what the
 * spec asks to land first with exceptional finish.
 */

export function ConceptRoot() {
  const [screen, setScreen] = useState<Screen>("home");

  return (
    <div data-concept style={{
      minHeight: "100dvh",
      background: "var(--bg-deep)",
      color: "var(--ink)",
    }}>
      <AnimatePresence mode="wait">
        <motion.div
          key={screen}
          initial={{opacity: 0, y: 12}}
          animate={{opacity: 1, y: 0}}
          exit={{opacity: 0, y: -8}}
          transition={{duration: 0.35, ease: [0.2, 0.7, 0.2, 1]}}
        >
          {screen === "home" && <DashboardHome/>}
          {screen !== "home" && <SoonScreen name={screen}/>}
        </motion.div>
      </AnimatePresence>
      <BottomNav active={screen} onChange={setScreen}/>
    </div>
  );
}

function SoonScreen({name}: {name: Screen}) {
  const title = {
    map:      "Carte interactive",
    controls: "Contrôles rapides",
    schedule: "Planning & zones",
    stats:    "Statistiques",
  }[name as Exclude<Screen, "home">];

  return (
    <div style={{
      minHeight: "100dvh",
      display: "flex", flexDirection: "column",
      alignItems: "center", justifyContent: "center",
      padding: "0 24px 120px", textAlign: "center",
    }}>
      <div style={{
        width: 96, height: 96, borderRadius: 28,
        background: "var(--grad-primary)",
        display: "flex", alignItems: "center", justifyContent: "center",
        fontSize: 38, color: "var(--bg-deep)",
        boxShadow: "0 18px 60px -10px rgba(124,255,178,0.4)",
        marginBottom: 22,
      }}>✿</div>
      <div className="display" style={{
        fontSize: 30, fontWeight: 700, letterSpacing: "-0.02em",
        marginBottom: 8,
      }}>
        {title}
      </div>
      <div style={{
        fontSize: 14, color: "var(--ink-2)", maxWidth: 320, lineHeight: 1.55,
      }}>
        Cet écran arrive dans la prochaine passe. Le Dashboard est la pièce
        montrée pour valider le langage visuel.
      </div>
    </div>
  );
}

export default ConceptRoot;
