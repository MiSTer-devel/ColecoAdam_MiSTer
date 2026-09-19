#!/usr/bin/env python3
"""make_report.py OUTROOT REPORTDIR — build the cartridge validation page.

Reads OUTROOT/summary.json (summarize.py), plus optional OUTROOT/notes.json
({dir: {"verdict": "core"|"explained", "note": text}}) and OUTROOT/findings.json.
"""
import html
import json
import os
import shutil
import sys

SHOTS = [120, 400, 740, 1000, 1300]
SHOT_NOTE = {120: "boot", 400: "title", 740: "option screen", 1000: "after input", 1300: "gameplay"}
ORDER = ["FAIL", "ISSUE", "REVIEW", "EXPLAINED", "DRIFT", "PASS"]
LABEL = {"FAIL": "Fail", "ISSUE": "Core issue", "REVIEW": "Review", "EXPLAINED": "Explained", "DRIFT": "Drift", "PASS": "Pass"}
TOTAL_LABEL = {"PASS": "Pass", "DRIFT": "Timing drift", "EXPLAINED": "Explained", "ISSUE": "Core issue", "REVIEW": "Unexplained", "FAIL": "Fail"}


def esc(s):
    return html.escape(str(s))


def pct(v):
    return "–" if v is None else "%.1f" % (float(v) * 100)


def bar(v):
    if v is None:
        return '<span class="bar"><i style="width:0"></i></span>'
    f = float(v)
    level = "hi" if f >= 0.98 else "mid" if f >= 0.75 else "lo"
    return '<span class="bar %s"><i style="width:%.1f%%"></i></span>' % (level, f * 100)


def load_json(path, default):
    return json.load(open(path)) if os.path.isfile(path) else default


def main():
    root, out = sys.argv[1], sys.argv[2]
    rows = json.load(open(os.path.join(root, "summary.json")))
    notes = load_json(os.path.join(root, "notes.json"), {})
    findings = load_json(os.path.join(root, "findings.json"), [])
    os.makedirs(os.path.join(out, "strips"), exist_ok=True)

    for r in rows:
        n = notes.get(r["dir"])
        if isinstance(n, str):
            n = {"verdict": "", "note": n}
        r["note"] = n["note"] if n else ""
        r["shown"] = r["status"]
        if r["status"] == "REVIEW" and n and n.get("verdict"):
            r["shown"] = "ISSUE" if n["verdict"] == "core" else "EXPLAINED"

    rows.sort(key=lambda r: (ORDER.index(r["shown"]), r["cart"].lower()))
    counts = {k: sum(1 for r in rows if r["shown"] == k) for k in ORDER}

    items = []
    for i, r in enumerate(rows):
        strip = ""
        if r["strip"]:
            strip = "strips/%03d.png" % i
            shutil.copyfile(os.path.join(root, r["strip"]), os.path.join(out, strip))
        title = os.path.splitext(r["cart"])[0]
        cells = []
        for s in SHOTS:
            sh = r["shots"].get(str(s)) or r["shots"].get(s) or {}
            m, fg = sh.get("match"), sh.get("fgmatch")
            cells.append('<td class="shot" title="frame %d, %s: %s%% of pixels, %s%% of foreground">%s<span class="num">%s</span></td>'
                         % (s, SHOT_NOTE[s], pct(m), pct(fg), bar(fg), pct(fg)))
        detail = ""
        if r["note"]:
            detail += '<p class="note">%s</p>' % esc(r["note"])
        if r["reasons"]:
            detail += '<ul class="reasons">%s</ul>' % "".join("<li>%s</li>" % esc(x) for x in r["reasons"])
        if strip:
            labels = "".join('<span>F%d <small>%s</small></span>' % (s, SHOT_NOTE[s]) for s in SHOTS)
            detail += ('<div class="strip"><div class="labels">%s</div>'
                       '<div class="pair"><span class="side">Core</span><span class="side">ColEm</span>'
                       '<img loading="lazy" src="%s" alt="%s: core frames above, closest ColEm frames below" width="1280" height="384"></div></div>'
                       % (labels, strip, esc(title)))
        items.append(
            '<tbody class="cart" data-status="%s" data-name="%s">'
            '<tr class="row" tabindex="0" aria-expanded="false">'
            '<td class="name"><span class="pill %s">%s</span><span class="title">%s</span></td>'
            '<td class="mono">%s</td><td class="mono">%s</td>%s</tr>'
            '<tr class="more" hidden><td colspan="8">%s</td></tr></tbody>'
            % (r["shown"].lower(), esc(title.lower()), r["shown"].lower(), LABEL[r["shown"]], esc(title),
               esc(r["header"].upper()), "%d K" % round(r["size"] / 1024), "".join(cells), detail))

    totals = ['<div class="total"><b>%d</b><span>Cartridges</span></div>' % len(rows)]
    for k in ("PASS", "DRIFT", "EXPLAINED", "ISSUE", "REVIEW", "FAIL"):
        if k in ("REVIEW", "FAIL") and not counts[k]:
            continue
        totals.append('<div class="total %s"><b>%d</b><span>%s</span></div>' % (k.lower(), counts[k], TOTAL_LABEL[k]))

    finding_items = "".join(
        '<li class="finding %s"><h3>%s</h3><p class="carts">%s</p><p>%s</p></li>'
        % ("core" if f["kind"] == "core" else "explained", esc(f["title"]), esc(f["carts"]), esc(f["detail"]))
        for f in findings)

    page = TEMPLATE.replace("{{TOTALS}}", "\n      ".join(totals))
    page = page.replace("{{FINDINGS}}", finding_items)
    page = page.replace("{{ROWS}}", "\n".join(items))
    with open(os.path.join(out, "index.html"), "w") as f:
        f.write(page)
    print("wrote %s: %d carts, %s" % (os.path.join(out, "index.html"), len(rows),
                                      ", ".join("%s=%d" % (k, counts[k]) for k in ORDER if counts[k])))


TEMPLATE = r"""<title>ColecoVision Cartridge Check</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Semi+Condensed:wght@500;600;700&family=IBM+Plex+Mono:wght@400;500&family=Source+Sans+3:wght@400;600&display=swap">
<style>
:root {
  --ground: #f1f2f7;
  --surface: #ffffff;
  --sunken: #f6f7fb;
  --ink: #161827;
  --muted: #5d6078;
  --line: #dcdfea;
  --accent: #4f47d6;
  --pass: #1b8a3e;
  --review: #9a7f0c;
  --fail: #c93a37;
  --explained: #56607a;
  --pass-soft: #e3f3e7;
  --review-soft: #f6efcf;
  --fail-soft: #fae3e2;
  --explained-soft: #e7eaf2;
  --accent-soft: #e8e7fb;
  --bar-track: #e6e8f0;
  --display: "Barlow Semi Condensed", "Arial Narrow", system-ui, sans-serif;
  --body: "Source Sans 3", system-ui, -apple-system, "Segoe UI", sans-serif;
  --mono: "IBM Plex Mono", ui-monospace, "SF Mono", Menlo, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --ground: #0f1018;
    --surface: #171927;
    --sunken: #13141f;
    --ink: #e7e8f3;
    --muted: #9699b2;
    --line: #2a2d42;
    --accent: #948eff;
    --pass: #4cc774;
    --review: #d9bd4a;
    --fail: #ff7a76;
    --explained: #a3abc4;
    --pass-soft: #16301f;
    --review-soft: #332b10;
    --fail-soft: #3a1a1a;
    --explained-soft: #242838;
    --accent-soft: #25234a;
    --bar-track: #262939;
  }
}
:root[data-theme="dark"] {
  --ground: #0f1018;
  --surface: #171927;
  --sunken: #13141f;
  --ink: #e7e8f3;
  --muted: #9699b2;
  --line: #2a2d42;
  --accent: #948eff;
  --pass: #4cc774;
  --review: #d9bd4a;
  --fail: #ff7a76;
  --explained: #a3abc4;
  --pass-soft: #16301f;
  --review-soft: #332b10;
  --fail-soft: #3a1a1a;
  --explained-soft: #242838;
  --accent-soft: #25234a;
  --bar-track: #262939;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--ground);
  color: var(--ink);
  font: 16px/1.5 var(--body);
  padding-inline: 20px;
  padding-block: 32px 64px;
}
.wrap { max-width: 1120px; margin: 0 auto; display: grid; gap: 32px; }
header { display: grid; gap: 14px; }
h1 { font: 700 clamp(30px, 5vw, 44px)/1.05 var(--display); letter-spacing: 0.01em; margin: 0; text-wrap: balance; }
h2 { font: 600 22px/1.2 var(--display); letter-spacing: 0.02em; margin: 0; }
.tms { display: grid; grid-template-columns: repeat(15, 1fr); height: 6px; max-width: 360px; border-radius: 1px; overflow: hidden; }
.tms i:nth-child(1) { background: #000000; } .tms i:nth-child(2) { background: #21c842; }
.tms i:nth-child(3) { background: #5edc78; } .tms i:nth-child(4) { background: #5455ed; }
.tms i:nth-child(5) { background: #7d76fc; } .tms i:nth-child(6) { background: #d4524d; }
.tms i:nth-child(7) { background: #42ebf5; } .tms i:nth-child(8) { background: #fc5554; }
.tms i:nth-child(9) { background: #ff7978; } .tms i:nth-child(10) { background: #d4c154; }
.tms i:nth-child(11) { background: #e6ce80; } .tms i:nth-child(12) { background: #21b03b; }
.tms i:nth-child(13) { background: #c95bba; } .tms i:nth-child(14) { background: #cccccc; }
.tms i:nth-child(15) { background: #ffffff; box-shadow: inset 0 0 0 1px var(--line); }
.lede { margin: 0; max-width: 70ch; color: var(--muted); }
.lede b { color: var(--ink); font-weight: 600; }
.totals { display: flex; flex-wrap: wrap; gap: 10px; }
.total { display: grid; gap: 2px; min-width: 112px; padding: 12px 16px; background: var(--surface); border: 1px solid var(--line); border-radius: 6px; }
.total b { font: 600 30px/1 var(--display); font-variant-numeric: tabular-nums; }
.total span { font-size: 13px; color: var(--muted); letter-spacing: 0.04em; text-transform: uppercase; }
.total.pass b { color: var(--pass); } .total.drift b { color: var(--accent); }
.total.explained b { color: var(--explained); } .total.issue b, .total.fail b { color: var(--fail); }
.total.review b { color: var(--review); }
.findings { display: grid; gap: 12px; }
.findings ul { list-style: none; margin: 0; padding: 0; display: grid; gap: 10px; }
.finding { background: var(--surface); border: 1px solid var(--line); border-left: 4px solid var(--fail); border-radius: 6px; padding: 14px 18px; display: grid; gap: 4px; }
.finding.explained { border-left-color: var(--explained); }
.finding h3 { margin: 0; font: 600 18px/1.25 var(--display); letter-spacing: 0.01em; }
.finding p { margin: 0; max-width: 78ch; }
.finding .carts { font: 13px/1.4 var(--mono); color: var(--muted); }
.controls { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; justify-content: space-between; }
.filters { display: flex; flex-wrap: wrap; gap: 6px; }
.filters button { font: 600 14px/1 var(--body); color: var(--muted); background: transparent; border: 1px solid var(--line); border-radius: 999px; padding: 8px 14px; cursor: pointer; }
.filters button[aria-pressed="true"] { color: var(--surface); background: var(--ink); border-color: var(--ink); }
input[type="search"] { font: 15px/1 var(--body); color: var(--ink); background: var(--surface); border: 1px solid var(--line); border-radius: 6px; padding: 9px 12px; width: min(280px, 100%); }
button:focus-visible, input:focus-visible, .row:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.table-wrap { overflow-x: auto; background: var(--surface); border: 1px solid var(--line); border-radius: 8px; }
table { width: 100%; border-collapse: collapse; min-width: 800px; }
th { text-align: left; font: 600 12px/1.2 var(--body); letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted); padding: 12px 12px 10px; border-bottom: 1px solid var(--line); vertical-align: bottom; }
th small { display: block; font-weight: 400; letter-spacing: 0; text-transform: none; font-size: 12px; }
td { padding: 9px 12px; border-top: 1px solid var(--line); vertical-align: middle; }
tbody.cart:first-of-type tr.row td { border-top: 0; }
.row { cursor: pointer; }
.row:hover td { background: color-mix(in srgb, var(--accent) 5%, transparent); }
.name { display: flex; align-items: center; gap: 10px; min-width: 280px; }
.title { font-weight: 600; }
.pill { flex: none; width: 86px; text-align: center; font: 600 12px/1 var(--body); letter-spacing: 0.03em; text-transform: uppercase; padding: 5px 0; border-radius: 4px; }
.pill.pass { color: var(--pass); background: var(--pass-soft); }
.pill.drift { color: var(--accent); background: var(--accent-soft); }
.pill.explained { color: var(--explained); background: var(--explained-soft); }
.pill.issue, .pill.fail { color: var(--fail); background: var(--fail-soft); }
.pill.review { color: var(--review); background: var(--review-soft); }
.mono { font-family: var(--mono); font-size: 13px; color: var(--muted); font-variant-numeric: tabular-nums; white-space: nowrap; }
.shot { white-space: nowrap; width: 92px; }
.bar { display: inline-block; width: 44px; height: 6px; background: var(--bar-track); border-radius: 3px; overflow: hidden; vertical-align: middle; margin-right: 6px; }
.bar i { display: block; height: 100%; background: var(--pass); }
.bar.mid i { background: var(--review); } .bar.lo i { background: var(--fail); }
.num { font: 12px var(--mono); color: var(--muted); font-variant-numeric: tabular-nums; }
.more td { background: var(--sunken); padding: 14px 12px 18px; }
.more .note { margin: 0 0 10px; max-width: 80ch; font-weight: 600; }
.reasons { margin: 0 0 12px; padding-left: 18px; color: var(--muted); font-size: 14px; }
.strip { display: grid; gap: 6px; max-width: 1040px; }
.labels { display: grid; grid-template-columns: repeat(5, 1fr); padding-left: 56px; font: 12px var(--mono); color: var(--muted); }
.labels small { font-family: var(--body); }
.pair { display: grid; grid-template-columns: 50px 1fr; grid-template-rows: 1fr 1fr; column-gap: 6px; }
.pair .side { font: 600 12px/1 var(--body); color: var(--muted); letter-spacing: 0.04em; text-transform: uppercase; align-self: center; }
.pair .side:nth-child(2) { grid-row: 2; }
.pair img { grid-column: 2; grid-row: 1 / span 2; width: 100%; height: auto; max-width: 100%; image-rendering: pixelated; border-radius: 3px; }
.empty { padding: 24px; color: var(--muted); margin: 0; }
footer { color: var(--muted); font-size: 14px; max-width: 78ch; display: grid; gap: 8px; }
footer p { margin: 0; }
@media (max-width: 640px) { .labels small { display: none; } }
</style>

<div class="wrap">
  <header>
    <h1>ColecoVision Cartridge Check</h1>
    <div class="tms" aria-hidden="true"><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i><i></i></div>
    <p class="lede">Every cartridge in <b>roms colecovision</b>, run on the ColecoAdam MiSTer core in Verilator (console mode) and on ColEm 5.6 with the same BIOS and the same scripted input. Each score is the share of the 256×192 display whose TMS9918 colour matches ColEm, ignoring the background colour. No cartridge crashed, went blank or stuck on the no-cartridge screen.</p>
    <div class="totals">
      {{TOTALS}}
    </div>
  </header>

  <section class="findings" aria-labelledby="findings-title">
    <h2 id="findings-title">What differs from ColEm</h2>
    <ul>{{FINDINGS}}</ul>
  </section>

  <section class="findings" aria-labelledby="carts-title">
    <h2 id="carts-title">Cartridges</h2>
    <div class="controls">
      <div class="filters" role="group" aria-label="Filter by result">
        <button type="button" id="f-attention" data-filter="attention" aria-pressed="true">Core issues</button>
        <button type="button" id="f-explained" data-filter="explained" aria-pressed="false">Explained</button>
        <button type="button" id="f-pass" data-filter="pass" aria-pressed="false">Pass</button>
        <button type="button" id="f-all" data-filter="all" aria-pressed="false">All</button>
      </div>
      <input type="search" id="search" placeholder="Find a cartridge" aria-label="Find a cartridge">
    </div>
    <div class="table-wrap">
      <table>
        <thead>
          <tr>
            <th>Cartridge</th><th>Header</th><th>Size</th>
            <th>F120<small>boot</small></th><th>F400<small>title</small></th><th>F740<small>option screen</small></th>
            <th>F1000<small>after input</small></th><th>F1300<small>gameplay</small></th>
          </tr>
        </thead>
        {{ROWS}}
      </table>
      <p class="empty" id="empty" hidden>No cartridges match this filter.</p>
    </div>
  </section>

  <footer>
    <p>Input script on controller 1: keypad 1 at frame 760, fire 1 at 900, stick right from 1050 for 40 frames, fire 1 at 1150. Frames 120–740 come before any input and should match ColEm almost exactly; later frames can drift because the two emulators time the CPU and sample input differently. Drift means the same screens appear in ColEm within 60 frames.</p>
    <p>Header 55AA skips the Coleco title screen; AA55 shows it for about eleven seconds. Select a row to see the core’s frames above ColEm’s closest match.</p>
    <p>The run used the simulator with clk_sys at the 10.7 MHz rate and --no-timing. Its frames were byte-identical to the stock build for every cartridge compared: Donkey Kong, Frogger, Jungle Hunt, Cosmo Fighter II, Super Cobra and Search for the Stolen Crown Jewels I, plus Adam boot with and without a disk.</p>
  </footer>
</div>

<script>
(function () {
  var carts = Array.prototype.slice.call(document.querySelectorAll("tbody.cart"));
  var buttons = Array.prototype.slice.call(document.querySelectorAll(".filters button"));
  var search = document.getElementById("search");
  var empty = document.getElementById("empty");
  var groups = {
    attention: ["issue", "review", "fail"],
    explained: ["explained"],
    pass: ["pass", "drift"]
  };
  var filter = "attention";

  function apply() {
    var q = search.value.trim().toLowerCase();
    var shown = 0;
    carts.forEach(function (c) {
      var ok = filter === "all" || groups[filter].indexOf(c.dataset.status) !== -1;
      if (q) ok = c.dataset.name.indexOf(q) !== -1;
      c.hidden = !ok;
      if (ok) shown++;
    });
    empty.hidden = shown !== 0;
  }

  function select(name) {
    filter = name;
    buttons.forEach(function (x) { x.setAttribute("aria-pressed", x.dataset.filter === name ? "true" : "false"); });
    apply();
  }

  buttons.forEach(function (b) {
    b.addEventListener("click", function () { select(b.dataset.filter); });
  });
  search.addEventListener("input", apply);

  carts.forEach(function (c) {
    var row = c.querySelector(".row");
    var more = c.querySelector(".more");
    function toggle() {
      more.hidden = !more.hidden;
      row.setAttribute("aria-expanded", more.hidden ? "false" : "true");
    }
    row.addEventListener("click", toggle);
    row.addEventListener("keydown", function (e) {
      if (e.key === "Enter" || e.key === " ") { e.preventDefault(); toggle(); }
    });
  });

  if (!carts.some(function (c) { return groups.attention.indexOf(c.dataset.status) !== -1; })) select("all");
  else apply();
})();
</script>
"""

if __name__ == "__main__":
    main()
