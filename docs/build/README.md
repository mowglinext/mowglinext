# Build manual (`mowgli.garden/build/`)

Interactive, bilingual (EN/FR) step-by-step manual for turning a YardForce
500 / 500B into a MowgliNext mower. Modelled on the ArmoredTurtle assembly
manuals: a media pane, one step at a time, prev/next, a chapter sidebar, and
a hardware profile that hides steps which do not apply.

It adapts the community **Mowgli Docs** guide by Juditech3D
(<https://mowglifrenchtouch.github.io/mowgli-docs/>, GPLv3) to the MowgliNext
stack: no controller, no CubeProgrammer, firmware flashed from the GUI,
config written by the installer and the onboarding wizard.

Served as-is by GitHub Pages together with the landing page — **no build
step**, plain HTML/CSS/JS.

## Files

| File | Role |
|---|---|
| `index.html` | interactive shell (sidebar, media pane, step pane, footer nav) |
| `print.html` | every chapter and step on one page, for printing / PDF |
| `manual.js` | engine: URL state, profile filter, localStorage progress, the markdown subset renderer, i18n strings |
| `manual.css` | styling — shares the palette of `../style.css` |
| `manual-data.js` | manual title, print note, **profile groups** |
| `content/NN-<chapter>.js` | one chapter each, loaded in order by both HTML files |
| `img/*.svg` | diagrams authored for this manual (overview, mainboard, J18, J9, Pi header, power, axes) |
| `img/*.jpg` | photos reused from mowgli-docs (GPLv3, credited in the captions) |

GUI screenshots are referenced from `../gui-walkthrough/screenshots/` and
`../screenshots/` rather than copied.

## URL contract

`?chapter=<id>&step=<n>&lang=en|fr` — `step` is 1-based **within the
filtered chapter**, so a deep link is only stable for the same profile.
Chapter ids are stable: `intro parts open compute imu gnss lidar firmware
install onboarding calibration first-mow troubleshooting credits`.

Landing-page deep links: `build/?chapter=parts`, `build/?chapter=firmware`.

## Editing content

Each chapter file pushes one object:

```js
MOWGLI_MANUAL.chapters.push({
  id: "gnss",                 // stable, used in URLs
  icon: "🛰️",
  title: { en: "...", fr: "..." },
  media: { type: "img", src: "img/gnss-uart.svg", alt: {en, fr} },   // chapter default
  steps: [
    {
      id: "gnss-antenna",     // stable, keys the viewer's "done" state
      title: { en, fr },
      body:  { en: `markdown`, fr: `markdown` },
      media: { type: "img" | "video", src, alt: {en, fr}, caption?: {en, fr}, credit?: {en, fr} },
      parts: [ { qty: 1, name: {en, fr} } ],
      when:  { gps: ["uart"] }   // optional: profile filter, see manual-data.js
    },
  ],
});
```

Keep both languages in the same object. A step without `media` shows the
chapter's `media`, or the chapter icon if there is none.

### Markdown subset (`manual.js` → `markdown()`)

Paragraphs, `### heading`, `- bullet`, `1. numbered`, `- [ ] task`,
fenced code blocks, pipe tables, `> quote`, callouts
`> [!NOTE|TIP|WARNING|DANGER] Title`, and inline `**bold**`, `*italic*`,
`` `code` ``, `[text](url)`. Nothing else is rendered; raw HTML is escaped.

### Profile filter

Groups and option ids live in `manual-data.js`. Add an option there, then
use it in `when:`. Never rename an id: viewers keep their choice in
localStorage under `mowgli-manual`.

## Checking locally

```bash
cd docs && python3 -m http.server 8000
# http://localhost:8000/build/            interactive
# http://localhost:8000/build/print.html  printable
```

`node --check content/*.js manual.js manual-data.js` catches syntax slips
before pushing.
