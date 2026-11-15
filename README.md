# SLAM School

A 24-week reading companion to the *SLAM Handbook: From Localization and Mapping to Spatial Intelligence*
(Carlone, Kim, Barfoot, Cremers, Dellaert, eds., Cambridge University Press; edition compiled April 21, 2026).
One hour a day, five days a week: read a few pages of the book, then do the lesson here. Every sixth day is a review.

The app explains the book in its own words and does not reproduce its text or figures. You need the book.

## Where

| What | Where |
|---|---|
| Live site | https://benmsk.github.io/slam-school/ (GitHub Pages, branch `main`, root) |
| Korean | https://benmsk.github.io/slam-school/ko.html. The **KR / EN** button in the header switches language and keeps your place. Once Korean is chosen, the English address opens the Korean page until you switch back. |
| Repository | https://github.com/BenMSK/slam-school |
| The app | `index.html` (open it in a browser; it is one self-contained file, about 3.9 MB) |
| The Korean page (generated, never edit by hand) | `ko.html`, built by `tools/i18n.py` from `index.html` + `i18n/ko/` |
| Korean translation | `i18n/ko/`: `STYLE.md` (the rules), `TASK.md` (how one week was translated), `<chunk>.en.txt` / `<chunk>.ko.txt`, `js.<week>.txt` |
| Lesson sources | `src/wNN/d1.html` … `d6.html` and `src/wNN/*.js`, one folder per week |
| Shell (CSS, header, overview, core script) | `src/shell.html` |
| Week list (titles, book sections, pages, planned widgets) | `src/weeks.json` |
| Contract every lesson follows | `src/CONTRACT.md` |
| Build and checks | `tools/build.js`, `tools/i18n.py`, `tools/htmlcheck.js`, `tools/review.py`, `tools/makeicon.js` |
| KaTeX 0.16.22 (inlined into `index.html` by the build) | `vendor/katex/` |
| Book text and page images used while writing (not published, git-ignored) | `research/book/` |

`slamhandbook.pdf`, `research/` and `build/` are in `.gitignore`: the book is copyrighted and must never be pushed.
Progress, theme and language are saved only in the browser.

## What is inside index.html

- **144 days**: 24 weeks × (5 lessons + 1 review). Day `k` of week `W` has the permanent id `d{(W-1)*6+k}`
  (Week 2 Day 1 is `#d7`). Never renumber.
- **119 widgets** (125 placements), each a small real computation: Gauss-Newton and Levenberg-Marquardt on a pose
  graph, variable elimination with fill-in, Exp/Log on SE(2), spline and Gaussian-process fits, RANSAC and graduated
  non-convexity, a rank-relaxed rotation synchronization with its optimality certificate, ICP, an FMCW radar chain,
  IMU preintegration, an observability lab, volume rendering, Gaussian belief propagation, and so on.
- **751 quiz questions**, **96 pen-and-paper problems** with worked solutions, **106 diagrams**, about **11,800 formulas**,
  and about **1,180 page chips** pointing into the book.
- Week nav (W1…W24, grouped by Part) and a day nav for the open week. Progress is in `localStorage` under `slam-school`.
- Widgets and math are set up the first time a day is opened, so the page starts fast despite its size.

| Weeks | Book | Topic |
|---|---|---|
| 1 | Prelude I | What SLAM is; Gaussians, least squares, factorizations |
| 2–3 | Ch 1 | Factor graphs, Gauss-Newton/LM, sparsity, elimination, iSAM |
| 4–5 | Ch 2 | Lie groups; splines and Gaussian processes |
| 6 | Ch 3 | Outliers: RANSAC, PCM, M-estimators, GNC |
| 7 | Ch 4 | Differentiable optimization |
| 8 | Ch 5 | Dense maps |
| 9–10 | Ch 6 | Shor's relaxation, SE-Sync, Cramér-Rao bound; Part I review |
| 11–12 | Prelude II, Ch 7 | Visual SLAM |
| 13 | Ch 8 | LiDAR SLAM |
| 14 | Ch 9 | Radar SLAM |
| 15 | Ch 10 | Event cameras |
| 16–17 | Ch 11 | IMU preintegration, observability, VIO |
| 18 | Ch 12 | Leg odometry; Part II review |
| 19 | Prelude III, Ch 13 | Deep learning in SLAM |
| 20 | Ch 14 | NeRF and Gaussian splatting |
| 21 | Ch 15 | Dynamic and deformable SLAM |
| 22 | Ch 16 | Metric-semantic SLAM |
| 23 | Ch 17 | Open-world spatial AI |
| 24 | Ch 18, Epilogue | Computational structure; whole-course review |

## Content conventions

`src/CONTRACT.md` is the full specification. In short:

- A lesson is: header (title, outcome, reading), "Read first" box, big idea, 3–5 numbered sections, at least one
  "Filling the gap" derivation the book skips, visuals, a 4-question quiz, recap, sources.
- A review day is: the week in one paragraph, a formula sheet, an 8-question quiz, 4 problems with solutions, a readiness checklist.
- Notation follows the book. Page chips (`<span class="src">§1.3.2 · p. 30</span>`) mark what comes from the book.
  Material that is not in the book names a standard textbook in the Sources list. There are no external links.
- Math is KaTeX: `\( \)` inline and `\[ \]` display. Never `<` or `>` inside math (`\lt`, `\gt`).
- Widget names are `wNN_name`; each week's JS is wrapped in its own block by the build.

## Edit and rebuild

`index.html` is generated. Edit the files in `src/`, then:

```
node tools/build.js --week 7      # builds build/w07.html and checks that week (fast)
node tools/build.js               # builds index.html and checks everything (about 3 minutes)
python tools/review.py d37        # audits the SVG labels of one day and screenshots its figures and widgets
```

`build.js` fails on: a malformed lesson fragment, a formula KaTeX cannot compile, widget JS that does not parse,
unbalanced tags, a quiz answer index out of range, a duplicate id, a widget without a function, and (in headless
Edge) any runtime error after mounting every day and moving every slider and pressing every button.

To change the look, edit `src/shell.html`. To bump the version, change `v1.1 (2026-11-15)` in the header and footer
there, and add a line below.

## Korean mode

`ko.html` is the same page with the text swapped, made the same way as in LLM School and Robotics School:
`tools/i18n.py` cuts `index.html` into text units (a heading, a paragraph, a table cell, a quiz option, an SVG label,
an aria-label), looks each one up in `i18n/ko/`, and writes the result. CSS, diagrams, formulas and widget logic are
shared, so a fix in `src/` reaches both pages on the next build.

- `i18n/ko/STYLE.md` is the rule book. Prose is Korean; every SLAM, math and ML term of art and every proper noun stays
  in English as the book writes it ("SLAM", "localization", "factor graph", "Jacobian", "loop closure", "pose"), with a
  Korean gloss in parentheses once where a lesson first defines a common-noun term (`<strong>odometry</strong>(주행 거리 측정)`).
- Text: `<chunk>.en.txt` is a numbered snapshot of the English units of one chunk (`ui`, `common`, `d1`…`d144`);
  `<chunk>.ko.txt` has the Korean under the same numbers (`=` means "keep the English").
- Widgets: `js.wNN.txt` holds pairs of an exact English source snippet and its Korean replacement, applied only inside
  that week's widget code; `js.ui.txt` (`@@ *`) translates the shared app script (day labels, buttons, week titles).
- Formulas are never translated: the check fails if any `\( \)` or `\[ \]` differs from the English, except words
  inside `\text{}`.

After changing `src/`:

```
node tools/build.js                      # rebuild index.html
python tools/i18n.py extract             # re-cut the English; new or changed units are marked TODO in the .ko.txt
                                         # translate them and delete the word TODO
python tools/i18n.py check               # every chunk 0 findings, apart from the deliberate number findings below
python tools/i18n.py build               # writes ko.html, then htmlcheck and a headless-Edge smoke test of every widget
```

A widget whose strings changed needs its pair updated in `i18n/ko/js.wNN.txt` (`python tools/i18n.py js wNN` prints
the week's script). The version string sits inside two `ui` units, so each version bump shows up as two TODO units.

## Change log

| Date | Version | Git tag | What |
|---|---|---|---|
| 2026-11-15 | v1.1 | `v1.1` | Korean mode: `ko.html`, generated by `tools/i18n.py` from `index.html` and `i18n/ko/` (14,508 text units and the strings of all 119 widgets, translated by rule: prose in Korean, SLAM terms and names in English). KR / EN button in the header that keeps the place and remembers the choice. English fixes found during translation: a garbled formula in Week 1 Day 3, a wrong count in Week 6 Day 3 and Week 9 Day 1, "yesterday" in Week 19 Day 4, a bedroom in an office in Week 23 Day 5, a noise-level caption in Week 22 Day 1. |
| 2026-11-15 | v1.0 | `v1.0` | First build: all 24 weeks in English. |

## Known gaps and things to verify

**How it was written.** Each week was written by a separate agent from the book's text and page images, against
`src/CONTRACT.md`. Every week passes the automated checks above. No human has read the 144 lessons end to end, and
the math has not been independently reviewed. Treat a surprising formula as possibly wrong and check it against the book.

**Places where a lesson departs from the printed book.** The authoring agents judged these to be misprints or
ambiguities in the book and say so in the lesson. They are the agents' readings and are not confirmed:

| Week | Where in the book | What the lesson does |
|---|---|---|
| 2 | eq. (1.39) | reads `L(δ)` as the linearized cost ‖Aδ − b‖² |
| 4 | Ad(T) size "(2d)×(2d)"; eq. (2.9) | notes the size fits only d = 3; drops a stray "=" |
| 5 | (2.56)–(2.62), Fig. 2.4 labels; (2.75) | notes an off-by-one index; presents (2.75) as a close-poses approximation |
| 7 | eq. (4.10); normal equations on p. 102 | carries a factor −η from (4.9); uses an explicit minus sign |
| 8 | eq. (5.9) | uses the zero-mean form of (5.10)–(5.11) |
| 9 | eq. (6.16); A in (6.17b) | reads R* as R; takes A as the transpose of Box 6.1's |
| 10 | range cross term on p. 168 | uses "−" where the page prints "+" |
| 12 | eq. (7.6) | describes a rigid-body motion where SO(3) is printed |
| 13 | eq. (8.1) vs Fig. 8.3; a sentence on p. 249 | keeps the target fixed; follows the next paragraph |
| 14 | eqs. (9.9), (9.11) | uses v = λf/2 derived from (9.4); states (9.11) as printed |
| 16 | eq. (11.7) last row; text at (11.3) | reads ω in the body frame |
| 17 | Table 11.1 "constant acceleration" | states it is the body-frame acceleration that must be constant |
| 18 | eq. (12.10); (12.38) | teaches the chained order fk(q) fk(q′)⁻¹; describes (12.38) as a per-sample sum |
| 19 | eqs. (13.3), (13.22) | uses I₂ on the right of (13.3); keeps the printed sign of (13.22) and flags it |
| 20 | definition of δᵢ on p. 403 | uses δᵢ = tᵢ₊₁ − tᵢ |
| 21 | eq. (15.3) | reads it as a Euclidean difference |
| 22 | eq. (16.15) | shows it as printed and notes the wording implies a different order |
| 23 | a bullet on p. 502 | follows the description (f^P) where f^L is printed |

**Other gaps.**

- The book's reference list (pp. 551–644) was not extracted, so citations to outside papers and textbooks are named
  from the agents' memory and were not checked. Some chapter numbers in textbook citations may be off.
- Cross-links between weeks (463 of them) all point to existing days with matching week and day labels, but an agent
  linking to another week assumed the day split in `src/weeks.json`. Where that week moved a topic to a different
  day, the link lands on a neighbouring lesson.
- Several weeks moved day boundaries from the plan to match the book's real section breaks, and Week 24's
  whole-course review was written before Weeks 21–23 existed, from `weeks.json`.
- Lessons are longer than planned (mostly 16–25 KB against a guide of 11–16 KB), so some days may take more than an hour.
- Phone widths below about 500 px were not tested: headless Edge does not go narrower. Check on a real phone.
- Widget screenshots were checked at their default settings. Behaviour after interaction was checked by the smoke
  test (no errors) and, for some widgets, by re-running the numbers in Node, not by eye.

**Korean page.**

- The Korean was machine-translated by rule (`i18n/ko/STYLE.md`), one week per agent, and checked mechanically: tags,
  formulas and numbers identical to the English, every unit present, the script parses, and every widget mounts and
  survives every control in headless Edge. Each translator also screenshot its week's figures and widgets for label
  overflow. No Korean-speaking person has proofread it end to end.
- `python tools/i18n.py check` prints 6 findings, all deliberate: a number word turned into Korean units
  ("12 million" → 1200만, "1.7 million" → 170만, "4.5 million" → 450만, "9 million" → 900만) in d13, d31, d50 and d53.
- Twenty-four translators worked in parallel from one term list. Small choices still differ between weeks (for example
  whether "orientation", "heading" or "update" stays English, and 각속도 versus "angular velocity"). "gravity" stays in
  English throughout, where Robotics School writes 중력.
- A few widget labels stay English on purpose because the code uses them as lookup keys (the Harris widget's
  Flat / Edge / Corner in Week 11, the scene-graph node names in Week 22, the feature-axis names in Week 23), and the
  lessons quote them the same way.
