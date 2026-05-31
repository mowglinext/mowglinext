import type {Easing, Transition, Variants} from "framer-motion";

/**
 * Motion presets for the concept screens.
 *
 * Two springs (soft for layout, snap for taps), one shared cubic-bezier
 * for tweens, and a couple of choreographed entrance variants. Keep all
 * easings in one file so the system stays coherent across screens.
 */

export const easeOut: Easing = [0.2, 0.7, 0.2, 1];

export const springSoft: Transition = {
  type: "spring", stiffness: 180, damping: 24, mass: 0.9,
};

export const springSnap: Transition = {
  type: "spring", stiffness: 380, damping: 32,
};

/** Standard rise-and-fade for cards / tiles. */
export const riseFade: Variants = {
  hidden: {opacity: 0, y: 16},
  show:   {opacity: 1, y: 0, transition: {duration: 0.55, ease: easeOut}},
};

/** Subtle scale entrance for hero glyphs. */
export const popIn: Variants = {
  hidden: {opacity: 0, scale: 0.94},
  show:   {opacity: 1, scale: 1,    transition: {duration: 0.6, ease: easeOut}},
};

/** Parent variant -- staggers children. */
export const staggerParent = (step = 0.06, initial = 0.08): Variants => ({
  hidden: {},
  show:   {transition: {staggerChildren: step, delayChildren: initial}},
});

/** Press feedback: scale down with snap. */
export const pressFeedback = {
  whileTap:   {scale: 0.96, transition: springSnap},
  whileHover: {scale: 1.015, transition: springSnap},
};
