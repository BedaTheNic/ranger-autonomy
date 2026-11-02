#!/usr/bin/env bash
# Update helpers, rename containers in place, and restart only the host-mounted gateway.
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
revision="${1:?Usage: bash apply_update.sh VERIFIED_COMMIT_SHA}"
[[ "$revision" =~ ^[0-9a-f]{40}$ ]] || { echo 'Use the full verified commit SHA.' >&2; exit 2; }
[[ -f "$hardware_dir/.env" ]] || { echo 'Run this from an existing deployment with .env.' >&2; exit 2; }
stage="$(mktemp -d "$hardware_dir/.update.XXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
files=(compose.yaml compose.gui.yaml compose.drive.yaml drive_guard.py robot.sh viewer.sh drive.sh
       inspect_drive.py ackermann_diagnostics.py ackermann_check.sh README.md)
for file in "${files[@]}"; do
  curl -fsSL "https://raw.githubusercontent.com/BedaTheNic/ranger-autonomy/$revision/autonomy_stack/hardware/$file" -o "$stage/$file"
done
python3 -m py_compile "$stage"/*.py
bash -n "$stage/robot.sh" "$stage/viewer.sh" "$stage/drive.sh" "$stage/ackermann_check.sh"
dc=(docker compose --project-directory "$hardware_dir" --env-file "$hardware_dir/.env")
for config in compose.yaml compose.gui.yaml compose.drive.yaml; do
  "${dc[@]}" -f "$stage/$config" --profile '*' config --quiet
  "${dc[@]}" -f "$stage/$config" --profile '*' config --format json > "$stage/$config.json"
  ids="$("${dc[@]}" -f "$hardware_dir/$config" --profile '*' ps --all --quiet)"
  for id in $ids; do
    id="$(docker inspect "$id" --format '{{.Id}}')"
    oneoff="$(docker inspect "$id" --format '{{ index .Config.Labels "com.docker.compose.oneoff" }}')"
    [[ "$oneoff" != True && "$oneoff" != true ]] || continue
    service="$(docker inspect "$id" --format '{{ index .Config.Labels "com.docker.compose.service" }}')"
    target="$(python3 - "$stage/$config.json" "$service" <<'PY'
import json,sys
services=json.load(open(sys.argv[1]))['services']
service=services.get(sys.argv[2])
if service is None:
    raise SystemExit('Custom service '+sys.argv[2]+' requires a manual Compose merge; nothing changed.')
print(service['container_name'])
PY
)"
    current="$(docker inspect "$id" --format '{{.Name}}')";current="${current#/}"
    printf '%s\t%s\t%s\t%s\n' "$id" "$service" "$current" "$target" >> "$stage/containers.tsv"
    if existing="$(docker inspect "$target" --format '{{.Id}}' 2>/dev/null)"; then
      [[ "$existing" == "$id" ]] || { echo "Name $target is already used by another container; nothing changed." >&2; exit 2; }
    fi
  done
done
touch "$stage/containers.tsv"
python3 - "$stage/containers.tsv" <<'PY'
import sys
rows=[line.rstrip('\n').split('\t') for line in open(sys.argv[1])]
targets=[row[3] for row in rows]
if len(set(targets))!=len(targets):raise SystemExit('Multiple containers resolve to one name; nothing changed.')
PY
mkdir -p "$hardware_dir/update_backups"
backup="$(mktemp -d "$hardware_dir/update_backups/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
for file in .env "${files[@]}"; do
  [[ ! -f "$hardware_dir/$file" ]] || cp -p -- "$hardware_dir/$file" "$backup/$file"
done
echo "Validated revision $revision. Backup: $backup"
cp -- "$stage/containers.tsv" "$backup/containers.before.tsv"
drive_id=''
while IFS=$'\t' read -r id service current target; do
  if [[ "$service" == drive-guard && "$(docker inspect "$id" --format '{{.State.Running}}')" == true ]]; then
    drive_id="$id"
  fi
done < "$stage/containers.tsv"
if [[ -n "$drive_id" ]]; then
  bash "$hardware_dir/drive.sh" disarm
  docker stop --signal SIGINT --time 10 "$drive_id" >/dev/null
fi
while IFS=$'\t' read -r id service current target; do
  if [[ "$current" != "$target" ]]; then
    echo "Renaming $current -> $target"
    docker rename "$id" "$target"
  fi
done < "$stage/containers.tsv"
for file in "${files[@]}"; do cp -- "$stage/$file" "$hardware_dir/$file"; done
if [[ -n "$drive_id" ]]; then docker start "$drive_id" >/dev/null; fi
echo 'Update complete. Gateway starts disarmed. When nodes are active: bash drive.sh arm'
