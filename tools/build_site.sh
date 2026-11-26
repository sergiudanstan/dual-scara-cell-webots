#!/usr/bin/env bash
# Build site/: a Webots HTML5 animation of one packing cycle, for GitHub Pages.
#   tools/build_site.sh            (T0=0 T1=90 simulated seconds)
# Publish: git subtree push --prefix site origin gh-pages
set -eu
ROOT=$(cd "$(dirname "$0")/.." && pwd)
WEBOTS=${WEBOTS:-/Applications/Webots.app/Contents/MacOS/webots}
T0=${T0:-2}
T1=${T1:-90}
SHOT_T=${SHOT_T:-18}
WALK_T=${WALK_T:-55}
tmp="$ROOT/worlds/.site.wbt"
sed 's/basicTimeStep 16/basicTimeStep 40/' "$ROOT/worlds/dual_scara_cell.wbt" > "$tmp"
cat >> "$tmp" <<WBT
Robot { name "animation recorder" controller "animation_recorder" supervisor TRUE
        controllerArgs [ "$ROOT/site/dual_scara.html" "$T0" "$T1" ] }
Robot { name "demo cues" controller "demo_cues" supervisor TRUE
        controllerArgs [ "$ROOT/site/dual_scara.jpg" "$SHOT_T" "$WALK_T" ] }
WBT
rm -f "$ROOT"/site/dual_scara.{html,css,json,w3d,jpg}
mkdir -p "$ROOT/site"
# macOS 27 makes Webots R2025a report every TCP port as busy. The interposer corrects that probe.
FIX="$ROOT/tools/webots_connect_fix.dylib"
if [ ! -f "$FIX" ]; then
  clang -dynamiclib -o "$FIX" "$ROOT/tools/webots_connect_fix.c"
fi
perl -e 'alarm shift; exec @ARGV' "${SITE_TIMEOUT:-1800}" \
  env DYLD_INSERT_LIBRARIES="$FIX" "$WEBOTS" --batch --mode=fast --minimize --stdout --stderr "$tmp" \
  > "$ROOT/site/.build.log" 2>&1 || true
rm -f "$tmp"
for f in dual_scara.html dual_scara.json dual_scara.w3d; do
  [ -s "$ROOT/site/$f" ] || { echo "site/$f missing (see site/.build.log)"; exit 1; }
done
python3 - "$ROOT/site/dual_scara.html" <<'PY'
import re, sys
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
desc = ("<p><strong>Două SCARA Epson T6</strong> (Webots R2025a): un ciclu de ambalare, "
        "de la banda de intrare până la ieșirea tăvilor.<br><br>"
        "Două Epson T6-602S împart același opritor. Fiecare umple tava lui cu 6 piese. "
        "Tava plină iese spre nord. Un operator traversează câmpurile scannerului.<br>"
        "Click stânga: orbită · click dreapta: panoramare · rotița: zoom · bara de jos: redare / derulare.<br><br>"
        "Sursă: <a href='https://github.com/sergiudanstan/dual-scara-cell-webots'>github.com/sergiudanstan/dual-scara-cell-webots</a></p>")
s = re.sub(r'<div class="description-container">.*?</div>',
           '<div class="description-container">' + desc + '</div>', s, count=1, flags=re.S)
s = re.sub(r"<h1>.*?</h1>", "<h1>Animație 3D: <strong>Două SCARA Epson T6</strong></h1>", s, count=1, flags=re.S)
s = re.sub(r"<title>.*?</title>", "<title>Două SCARA Epson T6 (Webots)</title>", s, count=1, flags=re.S)
s = s.replace("<html xmlns=", "<html lang=\"ro\" xmlns=")
open(p, "w", encoding="utf-8").write(s)
PY
cat > "$ROOT/site/index.html" <<'HTML'
<!DOCTYPE html>
<html lang="ro">
<head>
  <meta charset="utf-8">
  <meta http-equiv="refresh" content="0; url=dual_scara.html">
  <title>Două SCARA Epson T6 (Webots)</title>
</head>
<body><p><a href="dual_scara.html">Deschide animația 3D a celulei</a></p></body>
</html>
HTML
touch "$ROOT/site/.nojekyll"
grep -E '\[PLC|thumbnail|Traceback|error:' "$ROOT/site/.build.log" | tail -30 || true
rm -f "$ROOT/site/.build.log"
du -sh "$ROOT/site"
