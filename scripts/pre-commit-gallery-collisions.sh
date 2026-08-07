#!/usr/bin/env bash
# Pre-commit gate: multi-body gallery demos must not pick up new solid
# collisions across their animation cycle. Skip when the commit does not
# touch geometry, demos, or the gate itself.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "${ROOT}" ]]; then
  exit 0
fi
cd "${ROOT}"

# Staged paths that can change collision results.
if git rev-parse --verify HEAD >/dev/null 2>&1; then
  CHANGED="$(git diff --cached --name-only --diff-filter=ACMR)"
else
  # Initial commit: everything is new.
  CHANGED="$(git diff --cached --name-only --diff-filter=ACMR || true)"
fi

RELEVANT=0
while IFS= read -r path; do
  [[ -z "${path}" ]] && continue
  case "${path}" in
    mechlib/*|gallery/*|tests/test_gallery_collisions.py|scripts/pre-commit-gallery-collisions.sh)
      RELEVANT=1
      break
      ;;
  esac
done <<< "${CHANGED}"

if [[ "${RELEVANT}" -eq 0 ]]; then
  exit 0
fi

echo "gallery collision gate (pre-commit)..."
# Slightly fewer samples than CI for a snappier commit; the pytest suite
# uses the full default sample count.
exec python3 gallery/collision_gate.py -q --samples 6
