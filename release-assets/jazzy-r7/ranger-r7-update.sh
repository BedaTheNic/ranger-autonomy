#!/usr/bin/env bash
set -Eeuo pipefail
trap 'status=$?; printf "R7 update stopped at line %s with exit code %s.\n" "$LINENO" "$status" >&2; exit "$status"' ERR

deployment=/home/ranger/ranger_autonomy/ranger-hardware
image=ghcr.io/bedathenic/ranger-autonomy:jazzy-r7
release=https://github.com/BedaTheNic/ranger-autonomy/releases/download/jazzy-r7
files=(compose.yaml compose.bag.yaml bag.sh bag_record.py bag_export.py
       config/bag_topics.yaml config/bag_qos.yaml)

printf 'Checking the R6 installation and downloading the R7 release files.\n'
for file in .env robot.sh drive.sh viewer.sh; do
  test -f "$deployment/$file"
done
stage="$(mktemp -d)"
trap 'rm -rf -- "$stage"' EXIT
(
  cd "$stage"
  for file in ranger-hardware-deploy.tar.gz ranger-hardware-deploy.sha256 ranger-image.id; do
    curl -fsSL --retry 3 "$release/$file" -o "$file"
  done
  sha256sum -c ranger-hardware-deploy.sha256
  tar -xzf ranger-hardware-deploy.tar.gz
)
for file in "${files[@]}"; do
  test -f "$stage/ranger-hardware/$file"
done

printf 'Pulling the tested R7 image from GHCR.\n'
expected_image="$(cat "$stage/ranger-image.id")"
[[ "$expected_image" =~ ^sha256:[0-9a-f]{64}$ ]]
docker pull --platform linux/amd64 "$image"
test "$(docker image inspect -f '{{.Id}} {{.Os}}/{{.Architecture}}' "$image")" = "$expected_image linux/amd64"

cd "$deployment"
printf 'Preparing the existing deployment for the update.\n'
if [[ "$(docker inspect -f '{{.State.Running}}' ranger_hero_drive-guard 2>/dev/null)" == true ]]; then
  bash drive.sh disarm
fi
if [[ "$(docker inspect -f '{{.State.Running}}' ranger_hero_bag-recorder 2>/dev/null)" == true ]]; then
  bash bag.sh stop
fi

backup="$(mktemp -d "$PWD/r7-backup.XXXXXX")"
cp -p -- .env "$backup/"
for file in "${files[@]}"; do
  if [[ -f "$file" ]]; then cp --parents -p -- "$file" "$backup/"; fi
done
printf 'R6 configuration backup: %s\n' "$backup"
for file in "${files[@]}"; do
  install -D -m 0644 "$stage/ranger-hardware/$file" "$file"
done
sed -i -E '/^(AUTONOMY_IMAGE|NAV_PROFILE|FOOTPRINT_ESCAPE_ENABLED|RANGER_GUI_NAVIGATION_TOOLS)=/d' .env
printf '\n%s\n' "AUTONOMY_IMAGE=$image" 'NAV_PROFILE=adaptive' \
  'FOOTPRINT_ESCAPE_ENABLED=true' 'RANGER_GUI_NAVIGATION_TOOLS=true' >> .env

printf 'Starting R7 navigation, drive gateway, and RViz.\n'
bash robot.sh external
bash drive.sh up
bash viewer.sh up
bash robot.sh status
