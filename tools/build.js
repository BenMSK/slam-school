// Splice src/ into one self-contained page, check it, and smoke-test it in headless Edge.
//   node tools/build.js            → index.html (every week that exists in src/)
//   node tools/build.js --week 3   → build/w03.html (one week, for checking while writing it)
//   add --nosmoke to skip the headless-browser run
// Checks: fragment shape, every formula compiles in KaTeX, widget JS parses, tools/htmlcheck.js, then (smoke) every
// day is mounted with ?check, every widget control is poked, and runtime errors are reported.
const fs = require("fs"), path = require("path"), os = require("os"), cp = require("child_process");
const ROOT = path.dirname(__dirname), SRC = path.join(ROOT, "src"), KX = path.join(ROOT, "vendor", "katex");
const katex = require(path.join(KX, "katex.min.js"));
const args = process.argv.slice(2), only = args.includes("--week") ? +args[args.indexOf("--week") + 1] : 0;
const weeks = JSON.parse(fs.readFileSync(path.join(SRC, "weeks.json"), "utf8"));
const rel = f => path.relative(ROOT, f).replace(/\\/g, "/");

// KaTeX macros available in every lesson (and to tex() in widgets)
const MACROS = {
  "\\R": "\\mathbb{R}", "\\E": "\\mathbb{E}", "\\N": "\\mathcal{N}",
  "\\SO": "\\mathrm{SO}", "\\SE": "\\mathrm{SE}", "\\so": "\\mathfrak{so}", "\\se": "\\mathfrak{se}",
  "\\Exp": "\\operatorname{Exp}", "\\Log": "\\operatorname{Log}", "\\Ad": "\\operatorname{Ad}",
  "\\tr": "\\operatorname{tr}", "\\diag": "\\operatorname{diag}", "\\rank": "\\operatorname{rank}", "\\cov": "\\operatorname{cov}",
  "\\T": "^{\\mathsf{T}}", "\\norm": "\\left\\lVert #1 \\right\\rVert",
};

let errors = 0, warnings = 0;
const err = (...a) => { errors++; console.log("ERROR", ...a); };
const warn = (...a) => { warnings++; console.log("warn ", ...a); };

function checkMath(s, file) {
  const body = s.replace(/<svg[\s\S]*?<\/svg>/g, "").replace(/<(code|pre)\b[\s\S]*?<\/\1>/g, "");
  const re = /\\\(([\s\S]*?)\\\)|\\\[([\s\S]*?)\\\]/g; let m, n = 0;
  while ((m = re.exec(body))) {
    n++; const disp = m[2] != null; let t = disp ? m[2] : m[1];
    if (/[<>]/.test(t)) { err(file, "math contains < or > (write \\lt \\gt \\le \\ge, and no HTML tags inside math):", t.slice(0, 90)); continue; }
    t = t.replace(/&amp;/g, "&").replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&nbsp;/g, " ");
    try { katex.renderToString(t, { displayMode: disp, macros: Object.assign({}, MACROS), throwOnError: true, strict: "ignore" }); }
    catch (e) { err(file, "KaTeX:", e.message.replace(/\s+/g, " ").slice(0, 170), "| in:", t.replace(/\s+/g, " ").slice(0, 90)); }
  }
  if (/(^|[^\\$])\$[^$\n<]{1,60}\$/.test(body)) warn(file, "looks like $...$ math; use \\( \\) and \\[ \\]");
  for (const sv of s.match(/<svg[\s\S]*?<\/svg>/g) || []) if (/\\\(|\\\[/.test(sv)) err(file, "TeX inside an <svg>; use plain Unicode text there");
  return n;
}

let days = "", viz = "", nDays = 0, nMath = 0, nViz = 0, missing = [];
for (const W of weeks) {
  if (only && W.n !== only) continue;
  const dir = path.join(SRC, "w" + String(W.n).padStart(2, "0"));
  if (!fs.existsSync(dir)) { missing.push("week " + W.n); continue; }
  for (let k = 1; k <= 6; k++) {
    const f = path.join(dir, "d" + k + ".html"), id = (W.n - 1) * 6 + k;
    if (!fs.existsSync(f)) { missing.push(rel(f)); continue; }
    const s = fs.readFileSync(f, "utf8").replace(/^\uFEFF/, "").trim(), name = rel(f);
    const open = '<section class="day" data-day="' + id + '" id="d' + id + '" hidden>';
    if (!s.startsWith(open)) err(name, "must start with exactly:", open);
    if (!s.endsWith("</section>")) err(name, "must end with </section>");
    if ((s.match(/<section\b/g) || []).length !== 1) err(name, "must contain exactly one <section>");
    if (/<script\b|<style\b|<link\b|<iframe\b/i.test(s)) err(name, "no <script>, <style>, <link>, or <iframe> in a lesson fragment");
    if (!/<header class="day-h">/.test(s)) err(name, "missing header.day-h");
    if (!/class="eyebrow">Week (\d+) · (Day \d|Review)</.test(s) || RegExp.$1 != W.n) err(name, "eyebrow must read 'Week " + W.n + " · Day k' or 'Week " + W.n + " · Review'");
    if (!/<p class="lead">/.test(s)) err(name, "missing p.lead");
    if (!/class="quiz"/.test(s)) err(name, "missing .quiz");
    if (!/class="box ok"/.test(s)) err(name, "missing recap .box.ok");
    if (!/<p class="next"><\/p>/.test(s)) err(name, 'missing empty <p class="next"></p>');
    if (k < 6 && !/class="box read"/.test(s)) err(name, "missing the 'Read first' box (.box.read)");
    const head = (s.match(/<header class="day-h">[\s\S]*?<\/header>/) || [""])[0];
    if (/\\\(|\\\[/.test(head)) err(name, "no TeX in the lesson header (h2, lead, and meta are reused as plain text)");
    if (/href="https?:/.test(s)) warn(name, "external link found; only link pages you actually opened");
    for (const m of s.matchAll(/data-viz="([^"]+)"/g)) { nViz++; if (!m[1].startsWith("w" + String(W.n).padStart(2, "0") + "_")) err(name, "widget name must start with w" + String(W.n).padStart(2, "0") + "_ :", m[1]); }
    nMath += checkMath(s, name); nDays++;
    days += "\n<!-- week " + W.n + " · day " + k + " -->\n" + s + "\n";
  }
  for (const f of fs.readdirSync(dir).filter(x => x.endsWith(".js")).sort()) {
    const code = fs.readFileSync(path.join(dir, f), "utf8").replace(/^\uFEFF/, "");
    try { new Function('"use strict";' + code); } catch (e) { err(rel(path.join(dir, f)), "JavaScript does not parse:", e.message); }
    if (/<\/script/i.test(code)) err(rel(path.join(dir, f)), "must not contain </script");
    viz += "\n// ---- " + rel(path.join(dir, f)) + " ----\n{\n" + code + "\n}\n";
  }
}

// ---- assemble ----
const put = (s, mark, val) => { if (!s.includes(mark)) throw new Error("placeholder missing in shell: " + mark); return s.split(mark).join(val); };
const kcss = fs.readFileSync(path.join(KX, "katex.min.css"), "utf8")
  .replace(/url\(fonts\/([\w-]+)\.woff2\) format\("woff2"\),url\([^)]*\) format\("woff"\),url\([^)]*\) format\("truetype"\)/g,
    (m, n) => "url(data:font/woff2;base64," + fs.readFileSync(path.join(KX, "fonts", n + ".woff2")).toString("base64") + ') format("woff2")');
if (/url\(fonts\//.test(kcss)) err("KaTeX CSS still references font files");
const kjs = fs.readFileSync(path.join(KX, "katex.min.js"), "utf8") + "\n" + fs.readFileSync(path.join(KX, "auto-render.min.js"), "utf8");
if (/<\/script/i.test(kjs)) err("KaTeX JS contains </script");
let html = fs.readFileSync(path.join(SRC, "shell.html"), "utf8");
html = put(html, "/*KATEX_CSS*/", kcss);
html = put(html, "/*KATEX_JS*/", kjs);
html = put(html, "<!--DAYS-->", days);
html = put(html, "/*WEEKS*/[]", JSON.stringify(weeks.map(w => ({ t: w.title, book: w.book, pp: w.pp[0] + "–" + w.pp[1], part: w.part }))));
html = put(html, "/*MACROS*/{}", JSON.stringify(MACROS));
html = put(html, "/*VIZ*/", viz);
const out = only ? path.join(ROOT, "build", "w" + String(only).padStart(2, "0") + ".html") : path.join(ROOT, "index.html");
fs.mkdirSync(path.dirname(out), { recursive: true }); fs.writeFileSync(out, html);
console.log("built", rel(out), "·", (html.length / 1024).toFixed(0) + " KB ·", nDays, "days ·", nViz, "widgets ·", nMath, "formulas");
if (missing.length) console.log((only ? "MISSING " : "not written yet: ") + (missing.length > 12 ? missing.length + " files/weeks" : missing.join(", ")));
if (only && missing.length) errors += missing.length;

// ---- htmlcheck ----
const hc = cp.spawnSync(process.execPath, [path.join(__dirname, "htmlcheck.js"), out], { encoding: "utf8" });
const hcOut = (hc.stdout || "").trim().split("\n"), hcErr = +(/errors: (\d+)/.exec(hcOut[hcOut.length - 1]) || [0, 1])[1];
if (hcErr) { console.log("htmlcheck (line numbers refer to " + rel(out) + "):"); hcOut.slice(0, 40).forEach(l => console.log("  " + l)); errors += hcErr; }

// ---- smoke test in headless Edge ----
const EDGE = ["C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe", "C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe"].find(p => fs.existsSync(p));
if (!args.includes("--nosmoke") && EDGE && nDays) {
  const prof = path.join(os.tmpdir(), "slamschool-edge-" + (only || "all") + "-" + process.pid);
  const r = cp.spawnSync(EDGE, ["--headless=new", "--disable-gpu", "--no-first-run", "--user-data-dir=" + prof, "--virtual-time-budget=20000", "--window-size=900,1200", "--dump-dom",
    "file:///" + out.replace(/\\/g, "/") + "?check"], { encoding: "utf8", maxBuffer: 1 << 28, timeout: 240000 });
  try { fs.rmSync(prof, { recursive: true, force: true }); } catch (e) {}
  const m = /data-check="([^"]*)"/.exec(r.stdout || "");
  if (!m) err("smoke test did not report (the page script may have crashed at boot)");
  else {
    const c = JSON.parse(m[1].replace(/&quot;/g, '"').replace(/&lt;/g, "<").replace(/&gt;/g, ">").replace(/&amp;/g, "&"));
    console.log("smoke:", c.days, "days,", c.widgets, "widgets mounted");
    c.errors.forEach(e => err("runtime:", e)); c.mathErrors.forEach(e => err("math at runtime:", e)); c.emptyWidgets.forEach(e => err("widget rendered nothing:", e));
  }
}
console.log("errors:", errors, "· warnings:", warnings);
process.exit(errors ? 1 : 0);
