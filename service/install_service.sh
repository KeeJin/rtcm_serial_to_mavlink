#!/usr/bin/env bash
# install_service.sh — Install rtcm_bridge as a systemd service.
#
# Run as the user who owns the repo (not root).  The script will sudo only
# where needed (copying the unit file and reloading systemd).
#
# Usage:
#   chmod +x install_service.sh
#   ./install_service.sh
#
# Optional env overrides:
#   RTCM_PORT      serial port for RTCM input   (default: /dev/ttyACM0)
#   RTCM_BAUD      baud rate for RTCM input     (default: 115200)
#   MAVLINK_TARGET MAVLink UDP endpoint          (default: udpout:127.0.0.1:14550)

set -euo pipefail

SERVICE_NAME="rtcm_bridge"
UNIT_FILE="${SERVICE_NAME}.service"
SYSTEMD_DIR="/etc/systemd/system"

DEPLOY_USER="$(id -un)"
DEPLOY_HOME="$(eval echo ~"${DEPLOY_USER}")"
REPO_DIR="${DEPLOY_HOME}/rtcm_serial_to_mavlink"
VENV_PYTHON="${REPO_DIR}/.venv/bin/python"
TEMPLATE="${REPO_DIR}/${UNIT_FILE}"

RTCM_PORT="${RTCM_PORT:-/dev/ttyACM0}"
RTCM_BAUD="${RTCM_BAUD:-115200}"
MAVLINK_TARGET="${MAVLINK_TARGET:-udpout:127.0.0.1:14550}"

# Derive the systemd device unit name for the RTCM serial port.
# systemd-escape --path converts e.g. /dev/ttyACM0 → dev-ttyACM0
# then we append .device to get the proper unit name.
RTCM_DEVICE_UNIT="$(systemd-escape --path "${RTCM_PORT}").device"

echo "=== rtcm_bridge service installer ==="
echo "User       : ${DEPLOY_USER}"
echo "Home       : ${DEPLOY_HOME}"
echo "Repo       : ${REPO_DIR}"
echo "RTCM port  : ${RTCM_PORT}  @ ${RTCM_BAUD}  (device unit: ${RTCM_DEVICE_UNIT})"
echo "MAVLink    : ${MAVLINK_TARGET}"
echo ""

# --- Preflight checks ---

if [[ ! -d "${REPO_DIR}" ]]; then
    echo "ERROR: repo not found at ${REPO_DIR}" >&2
    exit 1
fi

if [[ ! -x "${VENV_PYTHON}" ]]; then
    echo "ERROR: venv python not found at ${VENV_PYTHON}" >&2
    echo "       Run:  cd ${REPO_DIR} && python3 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
    exit 1
fi

if [[ ! -f "${TEMPLATE}" ]]; then
    echo "ERROR: unit template not found at ${TEMPLATE}" >&2
    exit 1
fi

# Ensure the user is in the dialout group (required for serial access).
if ! id -nG "${DEPLOY_USER}" | grep -qw dialout; then
    echo "WARNING: ${DEPLOY_USER} is not in the dialout group."
    echo "         Run:  sudo usermod -aG dialout ${DEPLOY_USER}"
    echo "         Then log out and back in, or run:  newgrp dialout"
    echo "         Continuing install — add to group before starting service."
fi

# --- Generate the resolved unit file in /tmp ---

RESOLVED_UNIT="$(mktemp /tmp/${SERVICE_NAME}.XXXXXX.service)"

sed \
    -e "s|__USER__|${DEPLOY_USER}|g" \
    -e "s|__HOME__|${DEPLOY_HOME}|g" \
    -e "s|__RTCM_DEVICE__|${RTCM_DEVICE_UNIT}|g" \
    -e "s|--rtcm-port /dev/ttyACM0|--rtcm-port ${RTCM_PORT}|g" \
    -e "s|--rtcm-baud 115200|--rtcm-baud ${RTCM_BAUD}|g" \
    -e "s|udpout:127.0.0.1:14550|${MAVLINK_TARGET}|g" \
    "${TEMPLATE}" > "${RESOLVED_UNIT}"

echo "--- Generated unit file ---"
cat "${RESOLVED_UNIT}"
echo "---------------------------"
echo ""

# --- Install ---

echo "Installing ${SYSTEMD_DIR}/${UNIT_FILE} ..."
sudo cp "${RESOLVED_UNIT}" "${SYSTEMD_DIR}/${UNIT_FILE}"
sudo chmod 644 "${SYSTEMD_DIR}/${UNIT_FILE}"
rm -f "${RESOLVED_UNIT}"

echo "Reloading systemd daemon ..."
sudo systemctl daemon-reload

echo "Enabling ${SERVICE_NAME} (start at boot) ..."
sudo systemctl enable "${SERVICE_NAME}"

echo ""
echo "=== Done ==="
echo ""
echo "Start now  :  sudo systemctl start ${SERVICE_NAME}"
echo "Status     :  sudo systemctl status ${SERVICE_NAME}"
echo "Live logs  :  journalctl -u ${SERVICE_NAME} -f"
echo "Stop       :  sudo systemctl stop ${SERVICE_NAME}"
echo "Disable    :  sudo systemctl disable ${SERVICE_NAME}"
echo "Uninstall  :  sudo systemctl disable --now ${SERVICE_NAME} && sudo rm ${SYSTEMD_DIR}/${UNIT_FILE}"
