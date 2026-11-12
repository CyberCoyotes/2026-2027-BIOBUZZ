#!/bin/bash
# Compiles our team code against the REAL FTC SDK 12.0.0 jars and runs the small checks in checks/.
# Needs: java (17 or newer), curl, unzip. The first run downloads the SDK jars from Maven Central
# (about 8 MB, kept in ~/.cache/biobuzz-ftc-sdk-12.0.0, so later runs need no internet).
#
# This is NOT robot code and Gradle ignores this folder. It does not replace a robot test.
# See README.md in this folder.
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src/main/java"
CACHE="${BIOBUZZ_SDK_CACHE:-$HOME/.cache/biobuzz-ftc-sdk-12.0.0}"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# Downloads one file, trying up to 4 times. Maven Central sometimes answers "429 too many requests"
# when a network downloads a lot in a short time. Waiting a minute and running the script again fixes it.
download() {
    for attempt in 1 2 3 4; do
        if curl -sS -f -o "$2" "$1"; then
            return 0
        fi
        echo "  download failed (attempt $attempt of 4), waiting a few seconds ..."
        sleep $((attempt * 5))
    done
    echo
    echo "COULD NOT DOWNLOAD $1"
    echo "If the error above was 429, the server is rate-limiting this network: wait a minute and run the script again."
    exit 1
}

# ---- 1. the FTC SDK jars (downloaded once) ----
mkdir -p "$CACHE"
for name in RobotCore Hardware FtcCommon Vision; do
    if [ ! -f "$CACHE/$name.jar" ]; then
        echo "Downloading $name 12.0.0 ..."
        download "https://repo1.maven.org/maven2/org/firstinspires/ftc/$name/12.0.0/$name-12.0.0.aar" "$WORK/$name.aar"
        unzip -q -o "$WORK/$name.aar" classes.jar -d "$WORK/$name"
        mv "$WORK/$name/classes.jar" "$CACHE/$name.jar"
    fi
done
if [ ! -f "$CACHE/android.jar" ]; then
    echo "Downloading the Android stubs ..."
    download "https://repo1.maven.org/maven2/com/google/android/android/4.1.1.4/android-4.1.1.4.jar" "$CACHE/android.jar"
fi
SDK="$CACHE/RobotCore.jar:$CACHE/Hardware.jar:$CACHE/FtcCommon.jar:$CACHE/Vision.jar:$CACHE/android.jar"

# ---- 2. our team code (everything that does not need Pedro Pathing) ----
echo "Compiling team code against the real SDK ..."
cd "$SRC/org/firstinspires/ftc/teamcode"
FILES=$(ls *.java subsystems/*.java | grep -v -E "ShootAndParkAuto|LeaveAndParkAuto|PedroForwardBackAlpha")
javac -nowarn -proc:none -cp "$SDK" -d "$WORK/team" $FILES
echo "  OK ($(echo $FILES | wc -w) files)"

# ---- 3. a do-nothing android.util.Log, so RobotLog can run on a laptop ----
javac -nowarn -d "$WORK/shadow" "$HERE/shadow/android/util/Log.java"

# ---- 4. the checks ----
javac -nowarn -proc:none -cp "$WORK/shadow:$SDK:$WORK/team" -d "$WORK/checks" "$HERE"/checks/*.java 2>&1 | grep -E "error" || true
FAILED=0
for check in PowerMonitorDipCheck DriveAccelerationCheck ImuSubsystemCheck; do
    echo
    echo "=== $check ==="
    if ! java -cp "$WORK/shadow:$SDK:$WORK/team:$WORK/checks" "$check" 2>&1 | grep -v JAVA_TOOL_OPTIONS; then
        FAILED=1
    fi
    if [ "${PIPESTATUS[0]}" -ne 0 ]; then FAILED=1; fi
done
echo
if [ "$FAILED" -eq 0 ]; then echo "ALL LAPTOP CHECKS PASSED"; else echo "SOMETHING FAILED (see above)"; exit 1; fi
