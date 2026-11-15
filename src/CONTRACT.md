# SLAM School: contract for writing one week

SLAM School is a single-page web app that walks a reader through the *SLAM Handbook* (Carlone, Kim, Barfoot,
Cremers, Dellaert, eds., Cambridge University Press; edition compiled April 21, 2026) in 24 weeks. Each week has five
lesson days and one review day. The reader spends about an hour a day: roughly 25 minutes reading 3–6 pages of the
book, then the lesson here. The lesson's job is to make those pages understood: say the idea in plain words, fill in
the derivation steps and background math the book skips, give something to play with, and test it.

The reader is a smart adult with undergraduate linear algebra, probability, and calculus, and no SLAM background.
They have never met you or this file. Write for them.

You write ONE week. Everything you produce goes in `src/wNN/` (NN = two-digit week number). Do not edit any other
file (`src/shell.html`, `tools/`, other weeks). If you need a helper that does not exist, define it inside your own JS file.

## 1. What to produce

| File | Content |
|---|---|
| `src/wNN/d1.html` … `d5.html` | one lesson each: exactly one `<section class="day" …>` |
| `src/wNN/d6.html` | the review day |
| `src/wNN/viz.js` | every widget of the week (`VIZ.wNN_name = el => {…}`) |

Day ids are global: day k of week W has id `d{(W-1)*6+k}`. Week 2 is d7…d12, week 13 is d73…d78.
The first line of each file must be exactly `<section class="day" data-day="ID" id="dID" hidden>` and the last `</section>`.

Size guide: a lesson day is 11–16 KB of HTML, the review day 7–10 KB, the week's JS at most about 30 KB.
Depth beats length. Do not pad.

Your week's row in `src/weeks.json` gives the title, the book sections and pages, a suggested split into days, the
extra math the app must add, and the widgets to build. "MAIN" widgets are required. The day split and the page
numbers in the row are estimates: check the real section boundaries in the book text and adjust.

## 2. The book

- `research/book/pages.txt`: the whole book as text, one block per page, each headed `=== book p.N ===`.
  Find your pages with Grep (`^=== book p\.(19|34) ===`, output_mode content, -n) and Read that line range.
  Equations are garbled in the text. Prose, section titles, and figure captions are fine.
- `research/book/png/pNNN.png`: an image of each book page (p019.png = book page 19). Read the image of any page
  whose equations or figures you need to see exactly. Look at 6–12 pages, the ones that carry the math.
  `notation1.png` and `notation2.png` are the book's notation tables.

Read all of your week's pages before writing. Rules that follow from this being a companion to a copyrighted book:

- Explain in your own words. Do not copy sentences, and do not redraw the book's figures; draw your own diagram of the idea.
  Stating an equation, a definition, or a term is fine.
- Use the book's notation and symbol names for that chapter (for example the book perturbs poses on the right,
  \(\mathbf{T} = \mathbf{T}^0 \operatorname{Exp}(\boldsymbol{\xi})\), and orders \(\boldsymbol{\xi}\) as translation then rotation). The reader goes back and forth.
- Say where things are: put a page chip after each claim that comes from the book, `<span class="src">§1.3.2 · p. 30</span>`
  (a `span`, not a link), and refer to the book's equation and figure numbers in text ("equation (1.12)", "Figure 1.4").
- Content that is NOT in the book (the "extra math", your own derivations, background) must be standard textbook
  material you are sure of. Name the reference it follows in the Sources list by author, title, and year, with no URL
  (for example: Barfoot, *State Estimation for Robotics*, 2017; Solà, Deray, Atchuthan, *A micro Lie theory for
  state estimation in robotics*, 2018; Boyd and Vandenberghe, *Convex Optimization*, 2004; Hartley and Zisserman,
  *Multiple View Geometry*, 2004; Nocedal and Wright, *Numerical Optimization*, 2006; Thrun, Burgard, Fox,
  *Probabilistic Robotics*, 2005). Never write a URL. Never invent a citation, a number, or a quote.
- If the book and your memory disagree, the book wins; if you are not sure of a fact, leave it out.

## 3. Lesson anatomy (days 1–5)

```html
<section class="day" data-day="ID" id="dID" hidden>
<header class="day-h">
  <span class="eyebrow">Week W · Day k</span>
  <h2>Title, plain text</h2>
  <p class="lead">After today you can … (one sentence, plain text, an outcome; it is reused in the syllabus).</p>
  <p class="meta">About 60 min · Read §1.3, pp. 29–32 (4 pages) · Needs Week 2 Day 2</p>
</header>

<div class="box read"><h4>Read first: §1.3, pp. 29–32</h4>
<p>While you read, look for:</p><ul><li>…2–4 concrete things to find, with page numbers…</li></ul></div>

<div class="box key"><h3>Today's big idea</h3><p>2–3 sentences.</p></div>

<h3>1. …</h3>      3–5 numbered sections, one idea each. <strong>Bold</strong> each term where it is first defined.
                   Worked examples with real numbers. A visual in most sections (see below).
<div class="box note"><h4>Filling the gap: …</h4> a derivation step or background fact the book assumes. The main value of the app.</div>
<div class="box kid"><h4>In plain words</h4><p>…</p></div>          the idea with no jargon
<div class="box tip"><h4>Intuition</h4><p>…</p></div>                a picture or analogy
<div class="box dont"><h4>Common mistakes</h4><ul><li><strong>"Wrong belief."</strong> Why it is wrong.</li></ul></div>
<div class="box note"><h4>Going deeper (optional): …</h4>…</div>

<div class="quiz"><h3>Check yourself</h3>
  <div class="q" data-a="2"><p class="qq">Q1. …?</p><div class="opts"><button type="button" class="opt">…</button><button type="button" class="opt">…</button><button type="button" class="opt">…</button><button type="button" class="opt">…</button></div><p class="exp" hidden>Why the right answer is right.</p></div>
  … 4 questions (5 at most) …
</div>

<div class="box ok"><h4>Recap</h4><ol><li>…3–5 items…</li></ol></div>

<div class="refs"><h4>Sources</h4><ol><li><em>SLAM Handbook</em>, §…, pp. …</li><li>Author, <em>Title</em>, year (for the extra math).</li></ol></div>
<p class="next"></p>
</section>
```

Required in every lesson: the header, `.box.read`, `.box.key`, at least one "Filling the gap" note, a `.quiz`,
`.box.ok`, `.refs`, and the empty `<p class="next"></p>` (the app fills it). Use the other boxes where they help,
not all of them every day. `data-a` is the 0-based index of the right option; vary its position; four options,
plausible distractors; quiz questions test understanding and small calculations, not recall of wording.

Visuals: every lesson day has at least one, and most sections benefit from one. Options:
- a widget: `<p>Try it: what to do.</p><div class="viz" data-viz="wNN_name"></div><p class="cap">Notice … (what the reader should see).</p>`
- a diagram: `<figure class="fig"><svg viewBox="0 0 600 H" role="img" aria-label="full description">…</svg><figcaption>…</figcaption></figure>`
- a table: `<div class="tbl"><table><tr><th>…</th></tr><tr><td>…</td></tr></table></div>` (first row must be `th`; it becomes cards on phones)
- code or pseudocode: `<pre class="code"><code>…</code></pre>`

Cross-link other days with `<a href="#d14">Week 3 · Day 2</a>`. Link only to days of earlier weeks or your own week.

## 4. Review day anatomy (day 6)

```html
<section class="day" data-day="ID" id="dID" hidden>
<header class="day-h">
  <span class="eyebrow">Week W · Review</span>
  <h2>Week W review: short title</h2>
  <p class="lead">After today you can … (the week's skills, one sentence).</p>
  <p class="meta">About 60 min · No new pages</p>
</header>
<div class="box key"><h3>The week in one paragraph</h3><p>…</p></div>
<h3>1. Formula sheet</h3>        the 6–10 formulas of the week in a .tbl table: What it is | Formula | Where (book § and page, and the day)
<h3>2. Mixed quiz</h3>
<div class="quiz"><h3>Check yourself</h3> … 8 questions across the week … </div>
<h3>3. Pen-and-paper problems</h3>
<ol class="probs">
  <li><p>Problem statement with concrete numbers.</p>
      <details class="sol"><summary>Solution</summary><div><p>Worked solution, step by step.</p></div></details></li>
  … 4 problems, from routine to one that stretches …
</ol>
<div class="box ok"><h4>You are ready for next week if you can</h4><ul><li>…4–6 items…</li></ul></div>
<div class="refs"><h4>Sources</h4><ol><li>…</li></ol></div>
<p class="next"></p>
</section>
```
A review day may reuse one of the week's widgets (same `data-viz` name) if it helps. If your `weeks.json` row gives
D6 extra duties (a Part review), add them as further numbered sections.

## 5. Writing rules

- Plain words, short paragraphs, active voice. No hype, no filler, no "in this lesson we will". Define every term
  and symbol at first use. Prefer a worked example with numbers to a general statement.
- Derivations: show every step a careful undergraduate would need. This is where the book is terse and the app earns its keep.
- Be exact. Check every formula, sign, index, and number twice. Check each quiz answer by working it out.
- HTML must be well-formed: close every `<p>`, `<li>`, `<td>`, `<tr>`, `<div>`; quote attribute values; write `&amp;`
  for a literal ampersand and `&lt;` for a literal less-than in text. No `<script>`, `<style>`, `<link>`, `<iframe>`, no `id`
  attributes except inside your own SVGs (prefix them `wNN`), no inline event handlers, no images.
- The header (`h2`, `.lead`, `.meta`) is plain text: no math, no tags.

## 6. Math (KaTeX)

- Inline `\( … \)`, display `\[ … \]`. Never `$`. Math is rendered by KaTeX 0.16, so use only what KaTeX supports:
  `aligned`, `bmatrix`, `pmatrix`, `cases`, `array`, `\frac`, `\sum`, `\int`, `\mathbf`, `\boldsymbol`, `\mathcal`,
  `\mathbb`, `\operatorname`, `\underbrace`, `\text{…}`, `\tag{1.3}`, `\quad`. No `\newcommand`, `\label`, `\eqref`, `\begin{align}`, `\begin{equation}`.
- Never type `<` or `>` inside math: write `\lt`, `\gt`, `\le`, `\ge`, `\langle`, `\rangle`. No HTML tags inside math.
- In matrices and aligned blocks put spaces around `&` (` & `). Do not write `\\[4pt]`; use plain `\\`.
- Macros available: `\R` (ℝ), `\E`, `\N` (calligraphic N for a Gaussian), `\SO`, `\SE`, `\so`, `\se`, `\Exp`, `\Log`,
  `\Ad`, `\tr`, `\diag`, `\rank`, `\cov`, `\T` (transpose superscript: `\mathbf{A}\T`), `\norm{x}`. `\argmin` and `\argmax` are built in.
- Vectors bold lower case `\mathbf{x}`, matrices bold upper case `\mathbf{A}`, bold Greek with `\boldsymbol{\xi}`.
- No TeX inside `<svg>`, widget labels, `h2`, `.lead`, `.meta`. In SVG use Unicode (θ, Σ, ᵀ, subscripts via `<tspan baseline-shift="sub" font-size="10">`).
- Wide display equations scroll sideways on phones; still, break long ones with `aligned`.

## 7. SVG diagrams

- `viewBox="0 0 600 H"`, `role="img"`, and an `aria-label` that fully describes the picture.
- Colours only through CSS variables so both themes work: `var(--ink)`, `var(--muted)`, `var(--line)`, `var(--line2)`,
  `var(--surface)`, `var(--soft)`, `var(--acc)`, `var(--acc-soft)`, and five accent pairs `var(--c1)`…`var(--c5)` with
  `var(--c1-soft)`…`var(--c5-soft)` (c1 green, c2 amber, c3 pink, c4 violet, c5 blue). Never a hex colour, never `black`/`white`.
- Every `<text>` sets `fill` and `font-size` (11–14). Every label must fit inside its box at that font size:
  budget about 0.6 × font-size per character. Arrow labels sit beside the arrow, never on a box or a line.
- Self-close shapes (`<rect …/>`, `<line …/>`, `<path …/>`, `<circle …/>`); close `<text>` and `<tspan>`.
- Arrowheads: define one `<marker id="wNNar…">` in a `<defs>` per SVG; marker ids must be unique across the whole
  app, so prefix with your week and vary by figure (`w07ar1`, `w07ar2`).

## 8. Widgets (`src/wNN/viz.js`)

Your file is pasted into the app's script inside its own `{ … }` block, in strict mode. Top-level `const`/`function`
names are private to your week. Register each widget as `VIZ.wNN_name = el => { … }`; it is called once, when its day
is first opened, with the empty `div.viz` element. The names in `data-viz` must start with `wNN_`.

In scope:

```js
h(tag, attrs, ...children)          // HTML element. attrs: class, html (innerHTML), on<event> listeners, any attribute
sv(tag, attrs, ...children)         // SVG element
txt(x, y, string, attrs)            // SVG <text>, font-size 13 by default; class "mut" = muted colour
range(label, min, max, step, value, onInput, fmt)   // → {el, input}; a labelled slider. fmt formats the shown value
select(label, [[value, text], …], onChange)         // → {el, input}
button(label, onClick, cls)         // a .btn; cls "primary" or "ghost" optional. Toggle state with classList.toggle("on")
stat(key, value, sub)               // a stat tile; put several in h("div",{class:"stats"})
lineChart({w,h,aria,x:{min,max,log,ticks:[[v,label]],label},y:{min,max,ticks,label},series:[{name,color,dots,points:[{x,y,label,fmt}]}],marker:{x,y}})  // → element
stepper(el, [{t:"title", c:"html" | ()=>Element}], title, desc)   // a Back/Next step-through
tex(string, display)                // → span with KaTeX-rendered math, for formulas inside a widget
rng(seed)                           // → {u(): uniform [0,1), n(): standard normal}. ALWAYS seed randomness
M.zeros(r,c) M.eye(n) M.T(A) M.mul(A,B) M.mv(A,x) M.add(A,B) M.sub(A,B) M.scale(A,s)
M.solve(A,b)                        // dense linear solve, null if singular
M.inv(A)  M.chol(A)                 // inverse (null if singular); lower Cholesky factor (null if not positive definite)
M.eigSym(A)                         // symmetric eigen-decomposition → {val: ascending, vec: eigenvectors as columns}
covEllipse(S, cx, cy, k, scale, attrs)   // SVG ellipse of a 2×2 covariance at k sigma, screen coords (y down)
drag(svg, element, (x, y) => …)     // make an SVG element draggable (mouse and touch); x, y in viewBox units
cssv("--ink")                       // current value of a colour token, for <canvas> drawing
COL                                 // ["var(--c1)", … "var(--c5)"]
```

Layout convention (see `src/w01/d1.js`):
```js
el.append(h("div",{class:"vt"},"Title"), h("p",{class:"vd"},"One line: what this computes."),
  h("div",{class:"ctl"}, slider1.el, slider2.el), h("div",{class:"ctl"}, button(…), button(…)),
  svg, h("div",{class:"legend"}, …), h("div",{class:"stats"}, stat(…), …));
```
Other ready-made classes: `.panel` (soft box), `.out` (monospace output line), `.warn`, `.bars`/`.bar`, `.tok`, and plain `table`.

Rules:
- A widget is a small real computation of the thing the lesson teaches (an actual Gauss-Newton step, an actual
  Cholesky factorization), not an animation of a picture. The numbers it shows must be right.
- What the caption ("Notice …") and the prose claim must be what the widget does at its default settings and seeds.
  Work the default case through and check the claim.
- SVG `viewBox` 600 wide; `role="img"` and `aria-label`; colours via CSS variables only; text via `txt`.
  `<canvas>` is allowed for raster things (grids, images): size it in JS, read colours with `cssv`, and redraw on
  `document.addEventListener("themechange", draw)`.
- Works with touch and at 360 px width: controls wrap, nothing depends on hover, targets are at least 32 px.
- Robust: must not throw at any slider extreme or after any sequence of button presses. The build's smoke test sets
  every slider to its max and min and clicks every button. Animations run on `requestAnimationFrame`, stop by
  themselves at the end, and have a Play/Pause or Step button. Keep each redraw under about 30 ms.
- No network, no localStorage, no globals, no external libraries, no `document.getElementById` (query inside `el`).

## 9. Check your work (required)

From the project root (`C:\Users\msk93\Documents\Apps\SLAM`):

1. `node tools/build.js --week N` builds `build/wNN.html` from your files and checks them: fragment shape, every
   formula compiled by KaTeX, JS syntax, tag balance, quiz answer indices, then loads the page in headless Edge,
   mounts every widget, pokes every control, and reports runtime errors. Fix everything until it prints `errors: 0`.
2. `python tools/review.py dID --html build/wNN.html noshot` for each of your six days: audits SVG labels (off the
   canvas, overlapping, crossed by a line). Fix the findings.
3. `python tools/review.py dID --html build/wNN.html` (without `noshot`) for each day that has a widget: it prints PNG
   paths, one per figure and widget. Read the PNG of every widget and of at least two diagrams, and fix what looks
   wrong (overlaps, clipped labels, empty plots, numbers that contradict the caption).
4. Re-run step 1 after the last edit.

Do not stop before all seven files exist and step 1 prints `errors: 0`. If you run short, finish every file at
reduced length rather than leaving one missing.

## 10. Final report (your last message, under 200 words)

The day titles with the pages each day covers, the widget names, the result of the last `build.js` run, and
anything you were unsure about or left out (a formula you could not confirm against the page image, a claim not in
the book). No other commentary.
