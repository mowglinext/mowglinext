/*
 * MowgliNext build manual — manual shell.
 *
 * Chapters live in content/NN-<chapter>.js (one file each, loaded in order by
 * index.html and print.html); every one pushes onto MOWGLI_MANUAL.chapters.
 * Text fields are {en, fr} objects and bodies are the small markdown subset
 * documented in README.md.
 *
 * Profile groups drive the "Your build" selector: a step with
 *   when: { model: ["yf500b"] }
 * is shown only when that option is active. Option ids are stable — do not
 * rename them, viewers keep their choice in localStorage.
 */
window.MOWGLI_MANUAL = {
  id: "mowglinext",
  title: { en: "MowgliNext build manual", fr: "Guide de montage MowgliNext" },
  printNote: {
    en: "Every step of every chapter, including the ones the interactive version hides for other hardware profiles. Live version: https://mowgli.garden/build/",
    fr: "Toutes les étapes de tous les chapitres, y compris celles que la version interactive masque pour d'autres profils matériels. Version interactive : https://mowgli.garden/build/",
  },
  profile: {
    model: {
      label: { en: "Mower", fr: "Tondeuse" },
      options: [
        { id: "yf500", label: { en: "YardForce Classic 500", fr: "YardForce Classic 500" } },
        { id: "yf500b", label: { en: "YardForce 500B", fr: "YardForce 500B" } },
        { id: "other", label: { en: "SA650 / 900 ECO / other", fr: "SA650 / 900 ECO / autre" } },
      ],
    },
    gps: {
      label: { en: "GNSS connection", fr: "Connexion GNSS" },
      options: [
        { id: "usb", label: { en: "USB", fr: "USB" } },
        { id: "uart", label: { en: "UART (Pi header)", fr: "UART (GPIO du Pi)" } },
      ],
    },
    lidar: {
      label: { en: "LiDAR", fr: "LiDAR" },
      options: [
        { id: "no", label: { en: "No LiDAR", fr: "Sans LiDAR" } },
        { id: "yes", label: { en: "LD19 / LD06 LiDAR", fr: "LiDAR LD19 / LD06" } },
      ],
    },
  },
  chapters: [],
};
