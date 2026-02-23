#!/usr/bin/env bash
# generate_size_report.sh — Parse object sizes and emit a sci-fi themed HTML dashboard
# Usage: ./scripts/generate_size_report.sh <build-dir> [output-dir]

set -euo pipefail

BUILD_DIR="${1:?Usage: $0 <build-dir> [output-dir]}"
OUTPUT_DIR="${2:-.}"
OUTFILE="${OUTPUT_DIR}/index.html"

DRIVERS_LIB="${BUILD_DIR}/drivers/libsensorfusion_drivers.a"
MIDDLEWARE_LIB="${BUILD_DIR}/middleware/libsensorfusion_middleware.a"

for f in "$DRIVERS_LIB" "$MIDDLEWARE_LIB"; do
    [ -f "$f" ] || { echo "ERROR: $f not found" >&2; exit 1; }
done

COMMIT_SHA="${GITHUB_SHA:-$(git rev-parse --short HEAD 2>/dev/null || echo 'local')}"
TIMESTAMP="$(date -u '+%Y-%m-%d %H:%M:%S UTC')"

# ── Parse size output into arrays ──────────────────────────────────────────────
parse_size() {
    local lib="$1"
    size "$lib" | tail -n +2 | while IFS=$'\t' read -r text data bss dec hex filename; do
        # Extract component name: "Foo.cpp.o (ex ...)" → "Foo"
        local name
        name=$(echo "$filename" | sed 's/\.cpp\.o.*//')
        name=$(echo "$name" | xargs)  # trim whitespace
        text=$(echo "$text" | xargs)
        data=$(echo "$data" | xargs)
        bss=$(echo "$bss" | xargs)
        echo "${name}|${text}|${data}|${bss}"
    done
}

DRIVER_DATA=$(parse_size "$DRIVERS_LIB")
MIDDLEWARE_DATA=$(parse_size "$MIDDLEWARE_LIB")

# ── Compute totals ────────────────────────────────────────────────────────────
sum_column() {
    local data="$1" col="$2"
    echo "$data" | awk -F'|' -v c="$col" '{s+=$c} END {print s+0}'
}

DRV_TEXT=$(sum_column "$DRIVER_DATA" 2)
DRV_DATA=$(sum_column "$DRIVER_DATA" 3)
DRV_BSS=$(sum_column "$DRIVER_DATA" 4)
MW_TEXT=$(sum_column "$MIDDLEWARE_DATA" 2)
MW_DATA=$(sum_column "$MIDDLEWARE_DATA" 3)
MW_BSS=$(sum_column "$MIDDLEWARE_DATA" 4)
TOTAL_TEXT=$((DRV_TEXT + MW_TEXT))
TOTAL_DATA=$((DRV_DATA + MW_DATA))
TOTAL_BSS=$((DRV_BSS + MW_BSS))

# Find max .text across all components for bar scaling
MAX_TEXT=$(echo -e "${DRIVER_DATA}\n${MIDDLEWARE_DATA}" | awk -F'|' 'BEGIN{m=0} {if($2+0>m) m=$2+0} END{print m}')
[ "$MAX_TEXT" -eq 0 ] && MAX_TEXT=1

# ── Helper: emit table rows + bar chart rows ──────────────────────────────────
emit_table_rows() {
    local data="$1" accent="$2"
    echo "$data" | while IFS='|' read -r name text data_val bss; do
        local pct=$(( (text * 100) / MAX_TEXT ))
        [ "$pct" -lt 1 ] && pct=1
        cat <<ROWEOF
            <tr>
              <td class="comp-name">${name}</td>
              <td class="num">${text}</td>
              <td class="num">${data_val}</td>
              <td class="num">${bss}</td>
              <td class="bar-cell">
                <div class="bar" style="width:${pct}%;background:${accent};"></div>
                <span class="bar-label">${text} B</span>
              </td>
            </tr>
ROWEOF
    done
}

# ── Generate HTML ─────────────────────────────────────────────────────────────
mkdir -p "$OUTPUT_DIR"

cat > "$OUTFILE" <<'HTMLHEAD'
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SensorFusion // Code Size Report</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@300;400;700&display=swap');

  :root {
    --bg: #0a0a1a;
    --bg2: #0f0f2a;
    --bg3: #141433;
    --cyan: #00f0ff;
    --green: #00ff88;
    --magenta: #ff00aa;
    --dim: #334;
    --text: #c8c8e0;
    --text-bright: #eeeeff;
  }

  * { margin: 0; padding: 0; box-sizing: border-box; }

  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'JetBrains Mono', 'Fira Code', 'Cascadia Code', monospace;
    min-height: 100vh;
    overflow-x: hidden;
  }

  /* Scanline overlay */
  body::after {
    content: '';
    position: fixed;
    top: 0; left: 0; right: 0; bottom: 0;
    pointer-events: none;
    background: repeating-linear-gradient(
      0deg,
      transparent,
      transparent 2px,
      rgba(0,0,0,0.08) 2px,
      rgba(0,0,0,0.08) 4px
    );
    z-index: 9999;
  }

  .container {
    max-width: 1100px;
    margin: 0 auto;
    padding: 2rem 1.5rem;
  }

  /* ── Header ──────────────────────────────────────────── */
  .header {
    text-align: center;
    margin-bottom: 2.5rem;
    position: relative;
  }

  .header h1 {
    font-size: 1.6rem;
    font-weight: 700;
    letter-spacing: 0.25em;
    text-transform: uppercase;
    color: var(--cyan);
    text-shadow: 0 0 20px rgba(0,240,255,0.5), 0 0 60px rgba(0,240,255,0.2);
    animation: flicker 4s infinite alternate;
  }

  .header .subtitle {
    font-size: 0.75rem;
    color: var(--dim);
    letter-spacing: 0.3em;
    margin-top: 0.5rem;
    text-transform: uppercase;
  }

  @keyframes flicker {
    0%, 95%, 100% { opacity: 1; }
    96% { opacity: 0.8; }
    97% { opacity: 1; }
    98% { opacity: 0.7; }
    99% { opacity: 1; }
  }

  /* ── Meta info ───────────────────────────────────────── */
  .meta {
    display: flex;
    justify-content: center;
    gap: 2rem;
    margin-bottom: 2rem;
    font-size: 0.7rem;
    color: var(--dim);
    letter-spacing: 0.1em;
  }

  .meta span::before {
    content: '// ';
    color: var(--cyan);
    opacity: 0.5;
  }

  /* ── Summary cards ───────────────────────────────────── */
  .summary {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 1rem;
    margin-bottom: 2.5rem;
  }

  .card {
    background: var(--bg2);
    border: 1px solid var(--dim);
    border-radius: 4px;
    padding: 1.2rem;
    text-align: center;
    position: relative;
    overflow: hidden;
  }

  .card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
  }

  .card:nth-child(1)::before { background: var(--cyan); box-shadow: 0 0 15px var(--cyan); }
  .card:nth-child(2)::before { background: var(--green); box-shadow: 0 0 15px var(--green); }
  .card:nth-child(3)::before { background: var(--magenta); box-shadow: 0 0 15px var(--magenta); }

  .card .label {
    font-size: 0.6rem;
    letter-spacing: 0.2em;
    text-transform: uppercase;
    color: var(--dim);
    margin-bottom: 0.5rem;
  }

  .card .value {
    font-size: 1.8rem;
    font-weight: 700;
    color: var(--text-bright);
  }

  .card:nth-child(1) .value { color: var(--cyan); text-shadow: 0 0 10px rgba(0,240,255,0.4); }
  .card:nth-child(2) .value { color: var(--green); text-shadow: 0 0 10px rgba(0,255,136,0.4); }
  .card:nth-child(3) .value { color: var(--magenta); text-shadow: 0 0 10px rgba(255,0,170,0.4); }

  .card .unit {
    font-size: 0.7rem;
    color: var(--dim);
    margin-left: 0.2rem;
  }

  /* ── Section ─────────────────────────────────────────── */
  .section {
    margin-bottom: 2.5rem;
  }

  .section-title {
    font-size: 0.85rem;
    font-weight: 700;
    letter-spacing: 0.2em;
    text-transform: uppercase;
    margin-bottom: 1rem;
    padding-bottom: 0.5rem;
    border-bottom: 1px solid var(--dim);
  }

  .section:nth-of-type(odd) .section-title {
    color: var(--cyan);
    border-color: rgba(0,240,255,0.3);
  }

  .section:nth-of-type(even) .section-title {
    color: var(--green);
    border-color: rgba(0,255,136,0.3);
  }

  .section-total {
    font-size: 0.7rem;
    color: var(--dim);
    float: right;
    font-weight: 400;
    letter-spacing: 0.1em;
  }

  /* ── Table ───────────────────────────────────────────── */
  table {
    width: 100%;
    border-collapse: collapse;
    font-size: 0.78rem;
  }

  thead th {
    font-size: 0.6rem;
    letter-spacing: 0.15em;
    text-transform: uppercase;
    color: var(--dim);
    text-align: left;
    padding: 0.5rem 0.8rem;
    border-bottom: 1px solid var(--dim);
    font-weight: 400;
  }

  thead th.num, td.num { text-align: right; }

  tbody tr {
    border-bottom: 1px solid rgba(50,50,80,0.3);
    transition: background 0.15s;
  }

  tbody tr:hover {
    background: rgba(0,240,255,0.03);
  }

  td {
    padding: 0.55rem 0.8rem;
  }

  .comp-name {
    color: var(--text-bright);
    font-weight: 400;
  }

  .num {
    font-variant-numeric: tabular-nums;
    color: var(--text);
  }

  /* ── Bar chart ───────────────────────────────────────── */
  .bar-cell {
    position: relative;
    width: 40%;
  }

  .bar {
    height: 16px;
    border-radius: 2px;
    opacity: 0.7;
    transition: opacity 0.15s;
    min-width: 2px;
  }

  tr:hover .bar { opacity: 1; }

  .bar-label {
    position: absolute;
    right: 0.8rem;
    top: 50%;
    transform: translateY(-50%);
    font-size: 0.6rem;
    color: var(--dim);
  }

  /* Totals row */
  tr.total-row {
    border-top: 1px solid var(--dim);
    font-weight: 700;
  }

  tr.total-row td {
    padding-top: 0.8rem;
    color: var(--text-bright);
  }

  /* ── Footer ──────────────────────────────────────────── */
  .footer {
    text-align: center;
    font-size: 0.6rem;
    color: var(--dim);
    letter-spacing: 0.15em;
    padding-top: 2rem;
    border-top: 1px solid rgba(50,50,80,0.3);
  }

  .footer a {
    color: var(--cyan);
    text-decoration: none;
    opacity: 0.7;
    transition: opacity 0.15s;
  }

  .footer a:hover { opacity: 1; }

  /* ── Responsive ──────────────────────────────────────── */
  @media (max-width: 700px) {
    .summary { grid-template-columns: 1fr; }
    .bar-cell { display: none; }
    .header h1 { font-size: 1.1rem; }
  }
</style>
</head>
<body>
<div class="container">
  <div class="header">
    <h1>SensorFusion // Code Size Report</h1>
    <div class="subtitle">Embedded Sensor Driver &amp; Middleware Library</div>
  </div>
HTMLHEAD

# Meta info
cat >> "$OUTFILE" <<METAEOF
  <div class="meta">
    <span>commit ${COMMIT_SHA}</span>
    <span>${TIMESTAMP}</span>
    <span>release build</span>
  </div>
METAEOF

# Summary cards
cat >> "$OUTFILE" <<SUMEOF
  <div class="summary">
    <div class="card">
      <div class="label">Drivers .text</div>
      <div class="value">${DRV_TEXT}<span class="unit">B</span></div>
    </div>
    <div class="card">
      <div class="label">Middleware .text</div>
      <div class="value">${MW_TEXT}<span class="unit">B</span></div>
    </div>
    <div class="card">
      <div class="label">Total .text</div>
      <div class="value">${TOTAL_TEXT}<span class="unit">B</span></div>
    </div>
  </div>
SUMEOF

# ── Drivers table ──
cat >> "$OUTFILE" <<'DTHEAD'
  <div class="section">
    <div class="section-title">
DTHEAD

cat >> "$OUTFILE" <<DTTITLE
      Drivers <span class="section-total">.text ${DRV_TEXT} B &nbsp;|&nbsp; .data ${DRV_DATA} B &nbsp;|&nbsp; .bss ${DRV_BSS} B</span>
    </div>
DTTITLE

cat >> "$OUTFILE" <<'DTABLE'
    <table>
      <thead>
        <tr>
          <th>Component</th>
          <th class="num">.text</th>
          <th class="num">.data</th>
          <th class="num">.bss</th>
          <th>.text footprint</th>
        </tr>
      </thead>
      <tbody>
DTABLE

emit_table_rows "$DRIVER_DATA" "var(--cyan)" >> "$OUTFILE"

cat >> "$OUTFILE" <<'DTFOOT'
      </tbody>
    </table>
  </div>
DTFOOT

# ── Middleware table ──
cat >> "$OUTFILE" <<'MTHEAD'
  <div class="section">
    <div class="section-title">
MTHEAD

cat >> "$OUTFILE" <<MTTITLE
      Middleware <span class="section-total">.text ${MW_TEXT} B &nbsp;|&nbsp; .data ${MW_DATA} B &nbsp;|&nbsp; .bss ${MW_BSS} B</span>
    </div>
MTTITLE

cat >> "$OUTFILE" <<'MTABLE'
    <table>
      <thead>
        <tr>
          <th>Component</th>
          <th class="num">.text</th>
          <th class="num">.data</th>
          <th class="num">.bss</th>
          <th>.text footprint</th>
        </tr>
      </thead>
      <tbody>
MTABLE

emit_table_rows "$MIDDLEWARE_DATA" "var(--green)" >> "$OUTFILE"

cat >> "$OUTFILE" <<'MTFOOT'
      </tbody>
    </table>
  </div>
MTFOOT

# ── Grand total + Footer ──
cat >> "$OUTFILE" <<GFOOT
  <div class="section">
    <div class="section-title" style="color:var(--magenta);border-color:rgba(255,0,170,0.3);">
      Grand Total <span class="section-total">.text ${TOTAL_TEXT} B &nbsp;|&nbsp; .data ${TOTAL_DATA} B &nbsp;|&nbsp; .bss ${TOTAL_BSS} B</span>
    </div>
  </div>

  <div class="footer">
    <p>SensorFusion &mdash; Portable Embedded Sensor Drivers &amp; Middleware</p>
    <p style="margin-top:0.5rem;"><a href="https://github.com/Mrunmoy/SensorFusion">github.com/Mrunmoy/SensorFusion</a></p>
  </div>
</div>
</body>
</html>
GFOOT

echo "Generated ${OUTFILE} ($(wc -c < "$OUTFILE") bytes)"
