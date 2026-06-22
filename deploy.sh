#!/usr/bin/env bash
# deploy.sh — push TeamCode to the Control Hub over WiFi (no USB needed).
#
# Usage:
#   1. Connect this laptop to the FTC-XXXX WiFi hotspot.
#   2. Run:  ./deploy.sh
#
# The Control Hub is always the hotspot gateway at 192.168.43.1.
# ADB wireless runs on port 5555; no prior USB setup required on REV hardware.

set -euo pipefail

ADB="/home/dequackdealer/Android/Sdk/platform-tools/adb"
CONTROL_HUB="192.168.43.1:5555"
MAX_ATTEMPTS=10

echo "==> Connecting to Control Hub at $CONTROL_HUB ..."
"$ADB" disconnect "$CONTROL_HUB" &>/dev/null || true

connected=false
for i in $(seq 1 $MAX_ATTEMPTS); do
    result=$("$ADB" connect "$CONTROL_HUB" 2>&1)
    if echo "$result" | grep -q "connected"; then
        echo "    Connected: $result"
        connected=true
        break
    fi
    echo "    Attempt $i/$MAX_ATTEMPTS failed ($result) — retrying in 2s..."
    sleep 2
done

if [ "$connected" = false ]; then
    echo ""
    echo "ERROR: Could not reach the Control Hub after $MAX_ATTEMPTS attempts."
    echo "  - Make sure this laptop is on the FTC-XXXX WiFi network."
    echo "  - Make sure the Control Hub is powered on."
    exit 1
fi

echo ""
echo "==> Building and installing TeamCode ..."
./gradlew TeamCode:installDebug \
    --daemon \
    -Pandroid.builder.sdkDownload=false

echo ""
echo "==> Done! OpModes are live on the robot."
