#!/usr/bin/env bash
# Install (or remove) macOS LaunchAgents that keep moza2keelson and, optionally,
# moza-rudder-sim running for this user, restarting them after the base drops off USB.
#
#   scripts/install-macos-agents.sh -e <entity> [--connect tcp/localhost:7447] [--realm rise]
#                                   [--with-sim [--hull road-ferry-78m]] [-- <extra moza2keelson flags>]
#   scripts/install-macos-agents.sh --uninstall
#
# The connector gets --ffb-remote and the no-pedals profile unless the extra flags say
# otherwise. Logs: ~/Library/Logs/moza2keelson.log and ~/Library/Logs/moza-rudder-sim.log.
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
AGENTS="$HOME/Library/LaunchAgents"
TEMPLATE="$REPO/deploy/macos/agent.plist.template"
CONNECTOR_LABEL="com.rise-maritime.moza2keelson"
SIM_LABEL="com.rise-maritime.moza-rudder-sim"
DOMAIN="gui/$(id -u)"

entity="" connect="tcp/localhost:7447" realm="rise" with_sim=0 hull="road-ferry-78m" uninstall=0
extra=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -e|--entity-id) entity="$2"; shift 2 ;;
    --connect) connect="$2"; shift 2 ;;
    --realm) realm="$2"; shift 2 ;;
    --with-sim) with_sim=1; shift ;;
    --hull) hull="$2"; shift 2 ;;
    --uninstall) uninstall=1; shift ;;
    --) shift; extra=("$@"); break ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

unload() {
  launchctl bootout "$DOMAIN/$1" 2>/dev/null || true
  rm -f "$AGENTS/$1.plist"
}

if [[ $uninstall -eq 1 ]]; then
  unload "$CONNECTOR_LABEL"
  unload "$SIM_LABEL"
  echo "Removed $CONNECTOR_LABEL and $SIM_LABEL."
  exit 0
fi
[[ -n "$entity" ]] || { echo "-e <entity> is required" >&2; exit 2; }
[[ -x "$REPO/.venv/bin/moza2keelson" ]] || { echo "run 'uv sync --locked' in $REPO first" >&2; exit 2; }

xml_args() {  # one <string> per argument, XML-escaped
  local a
  for a in "$@"; do
    a="${a//&/&amp;}"; a="${a//</&lt;}"; a="${a//>/&gt;}"
    printf '    <string>%s</string>\n' "$a"
  done
}

install_agent() {  # label log args...
  local label="$1" log="$2"; shift 2
  local plist="$AGENTS/$label.plist" args
  args="$(xml_args "$@")"
  mkdir -p "$AGENTS" "$HOME/Library/Logs"
  unload "$label"
  python3 - "$TEMPLATE" "$plist" "$label" "$REPO" "$log" "$args" <<'PY'
import sys
template, out, label, repo, log, args = sys.argv[1:]
text = open(template).read()
for key, value in (("@LABEL@", label), ("@REPO@", repo), ("@LOG@", log), ("@ARGS@", args.rstrip("\n"))):
    text = text.replace(key, value)
open(out, "w").write(text)
PY
  plutil -lint -s "$plist"
  launchctl bootstrap "$DOMAIN" "$plist"
  echo "Installed $label: $plist (log $log)"
}

connector_args=("$REPO/.venv/bin/moza2keelson" --mode client --connect "$connect" -r "$realm" -e "$entity")
if [[ ${#extra[@]} -eq 0 ]]; then
  connector_args+=(-p r21-ultra-no-pedals --ffb-remote)
else
  connector_args+=("${extra[@]}")
fi
install_agent "$CONNECTOR_LABEL" "$HOME/Library/Logs/moza2keelson.log" "${connector_args[@]}"

if [[ $with_sim -eq 1 ]]; then
  install_agent "$SIM_LABEL" "$HOME/Library/Logs/moza-rudder-sim.log" \
    "$REPO/.venv/bin/moza-rudder-sim" --mode client --connect "$connect" -r "$realm" -e "$entity" --hull "$hull"
fi
