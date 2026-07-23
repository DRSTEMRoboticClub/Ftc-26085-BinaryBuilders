#!/usr/bin/env bash
# deploy.sh — build, connect, and deploy TeamCode to the FTC Control Hub over WiFi.
#
# Usage:
#   1. Connect this laptop to the FTC-XXXX WiFi hotspot broadcast by the Control Hub.
#   2. Run:  ./deploy.sh
#
# Options:
#   --build-only    Only build, skip connecting and deploying.
#   --no-build      Skip the build, only connect and deploy an existing APK.
#   --usb           Deploy over USB instead of WiFi.
#
# The Control Hub is always the hotspot gateway at 192.168.43.1.
# ADB wireless runs on port 5555; no prior USB setup required on REV hardware.

set -euo pipefail

# ─── Configuration ───────────────────────────────────────────────────────────
CONTROL_HUB_IP="192.168.43.1"
CONTROL_HUB="${CONTROL_HUB_IP}:5555"
MAX_ATTEMPTS=10
RETRY_DELAY=2

# ─── Java (requires 17+) ─────────────────────────────────────────────────────
java_version_ok() {
    "$1/bin/java" -version 2>&1 | grep -qE '"(1[7-9]|[2-9][0-9])\.'
}

if [[ -n "${JAVA_HOME:-}" ]] && java_version_ok "$JAVA_HOME"; then
    : # already good
else
    found=false
    for candidate in \
        /usr/lib/jvm/java-17-openjdk-amd64 \
        /usr/lib/jvm/java-21-openjdk-amd64 \
        "$HOME/.jdks/corretto-17"* \
        "$HOME/.jdks/temurin-17"* \
        "$HOME/android-studio/jbr" \
        "/opt/android-studio/jbr" \
        "/snap/android-studio/current/android-studio/jbr"; do
        if [[ -x "$candidate/bin/java" ]] && java_version_ok "$candidate"; then
            export JAVA_HOME="$candidate"
            found=true
            break
        fi
    done
    if [[ "$found" == false ]]; then
        echo "ERROR: Java 17+ is required but not found."
        echo "  Install it with:  sudo apt-get install -y openjdk-17-jdk"
        echo "  Or set JAVA_HOME to a Java 17+ installation."
        exit 1
    fi
fi
export PATH="$JAVA_HOME/bin:$PATH"
echo "Using JAVA_HOME: $JAVA_HOME"

# ─── Parse arguments ─────────────────────────────────────────────────────────
BUILD=true
DEPLOY=true
USE_USB=false

for arg in "$@"; do
    case "$arg" in
        --build-only) DEPLOY=false ;;
        --no-build)   BUILD=false ;;
        --usb)        USE_USB=true ;;
        -h|--help)
            sed -n '2,/^$/s/^# //p' "$0"
            exit 0
            ;;
        *)
            echo "Unknown option: $arg"
            exit 1
            ;;
    esac
done

# ─── Locate ADB ──────────────────────────────────────────────────────────────
find_adb() {
    # Prefer the SDK's adb to avoid version mismatches with Gradle's internal adb.
    local sdk_dir="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-$HOME/Android/Sdk}}"
    if [[ -x "$sdk_dir/platform-tools/adb" ]]; then
        echo "$sdk_dir/platform-tools/adb"
        return
    fi
    # Fallback: common install locations
    local candidates=(
        "$HOME/Android/Sdk/platform-tools/adb"
        "$HOME/Library/Android/sdk/platform-tools/adb"
        "/usr/local/android-sdk/platform-tools/adb"
    )
    for candidate in "${candidates[@]}"; do
        if [[ -x "$candidate" ]]; then
            echo "$candidate"
            return
        fi
    done
    # Last resort: system PATH
    if command -v adb &>/dev/null; then
        command -v adb
        return
    fi
    return 1
}

ADB=$(find_adb) || {
    echo "ERROR: Could not find adb. Install Android SDK platform-tools or add adb to PATH."
    exit 1
}
echo "Using adb: $ADB"

# Export ANDROID_HOME so Gradle uses the same SDK (and same adb) as this script
export ANDROID_HOME="${ANDROID_HOME:-$HOME/Android/Sdk}"

# ─── Build ────────────────────────────────────────────────────────────────────
if [[ "$BUILD" == true ]]; then
    echo ""
    echo "==> Building TeamCode ..."
    ./gradlew TeamCode:assembleDebug \
        --daemon \
        -Pandroid.builder.sdkDownload=false
    echo "    Build successful."
fi

# ─── Connect & Deploy ─────────────────────────────────────────────────────────
if [[ "$DEPLOY" == true ]]; then

    if [[ "$USE_USB" == false ]]; then
        echo ""
        echo "==> Connecting to Control Hub at $CONTROL_HUB ..."
        "$ADB" disconnect "$CONTROL_HUB" &>/dev/null || true

        connected=false
        for i in $(seq 1 $MAX_ATTEMPTS); do
            result=$("$ADB" connect "$CONTROL_HUB" 2>&1)
            if echo "$result" | grep -qi "connected"; then
                echo "    Connected: $result"
                connected=true
                break
            fi
            echo "    Attempt $i/$MAX_ATTEMPTS failed ($result) — retrying in ${RETRY_DELAY}s..."
            sleep "$RETRY_DELAY"
        done

        if [[ "$connected" == false ]]; then
            echo ""
            echo "ERROR: Could not reach the Control Hub after $MAX_ATTEMPTS attempts."
            echo "  - Make sure this laptop is on the FTC-XXXX WiFi network."
            echo "  - Make sure the Control Hub is powered on."
            echo "  - Try pinging $CONTROL_HUB_IP first."
            exit 1
        fi
    else
        echo ""
        echo "==> Using USB connection..."
        "$ADB" devices | grep -q "device$" || {
            echo "ERROR: No device found over USB. Connect the Control Hub via USB and enable USB debugging."
            exit 1
        }
    fi

    echo ""
    echo "==> Installing TeamCode on Control Hub ..."
    # Reconnect in case the adb server was restarted during the build
    if [[ "$USE_USB" == false ]]; then
        "$ADB" connect "$CONTROL_HUB" &>/dev/null || true
        sleep 1
    fi

    APK="TeamCode/build/outputs/apk/debug/TeamCode-debug.apk"
    if [[ ! -f "$APK" ]]; then
        echo "ERROR: APK not found at $APK — did the build succeed?"
        exit 1
    fi
    # -r = replace existing, -d = allow version downgrade
    # If signatures don't match, uninstall the old app first then retry.
    if ! "$ADB" -s "$CONTROL_HUB" install -r -d "$APK" 2>&1 | tee /dev/stderr | grep -q "Success"; then
        echo "    Install failed — attempting uninstall of old app and retry..."
        "$ADB" -s "$CONTROL_HUB" uninstall com.qualcomm.ftcrobotcontroller || true
        "$ADB" -s "$CONTROL_HUB" install -r -d "$APK"
    fi

    echo ""
    echo "==> Done! OpModes are now live on the robot."
fi
