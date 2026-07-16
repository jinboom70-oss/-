#!/usr/bin/env bash
set -eo pipefail

# This wrapper intentionally keeps navigation parameters in start_navigation.sh.
# Put only QR-entry parameters here when they do not already exist there.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# QR entry starts the USB camera by default. Override with START_CAMERA=0 if needed.
export START_CAMERA="${START_CAMERA:-1}"

exec bash "${SCRIPT_DIR}/start_navigation.sh"
