# Limelight Distance Calculation - FTC-26085 Fix Guide

## Summary of Issues Fixed

### ✅ FIXED: Null Pointer Exceptions
**Problem**: Code was calling `pose.getPosition()` multiple times without checking if `getPosition()` returns null.
- Even when `Pose3D` object exists, its `getPosition()` method can return null if camera calibration data is missing
- This caused intermittent crashes when the tag was visible (LED green) but 3D pose was unavailable

**Files Fixed**:
1. [ShooterSubsystem.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/subsystems/ShooterSubsystem.java#L831)
   - `distanceFromFiducial()` - now checks `getPosition()` before using
   - `computeTargetCameraSpace()` - added defensive null checks for position & orientation

2. [LimelightManager.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tools/LimelightManager.java#L217)
   - `getTagDistanceCm()` - returns -1 safely if position is null

---

## Why Distance Still Shows As "none" (When LED Is Green)

### Root Cause: Missing Camera Calibration
When the Limelight sees a tag (green LED), it successfully performs 2D detection (TX, TY, TA).
However, 3D pose calculation requires **camera calibration data** for the current resolution.

**Symptom**: 
- LED is **GREEN** ✓ (2D detection working)
- Distance = **-1 or "none"** ✗ (3D pose unavailable)

### Why This Happens
1. The AprilTag pipeline (`AprilTags.vpr`) has `fiducial_skip3d:0` (3D enabled)
2. But the Limelight's **intrinsic camera calibration** was never uploaded for this pipeline
3. Without calibration, `getTargetPoseCameraSpace()` returns null
4. Code falls back to TY-based distance calculation (see [ShooterConfig.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java#L170))

---

## How to Fix: Re-Calibrate Your Limelight

### Option 1: Upload Camera Calibration via Web UI (Recommended)
1. **Connect to Limelight Web Interface**:
   - Find LL IP: Open FTC app → Three-dot menu → Limelight Info
   - Navigate to: `http://<LIMELIGHT_IP>:5801` (e.g., `http://192.168.43.1:5801`)

2. **Calibrate the Camera**:
   - Go to **Settings** → **Camera Calibration**
   - Select appropriate **resolution** for your pipeline
   - Print and position an AprilTag at various distances/angles
   - Click **Calibrate** and follow on-screen prompts
   - **DO NOT skip this step** — calibration is required for 3D pose

3. **Verify Pipeline Has 3D Enabled**:
   - Go to **Settings** → **Pipelines**
   - Select your AprilTag pipeline
   - Ensure "3D" is **ON** (this is already set in `AprilTags.vpr`)

4. **Export & Update Local Pipeline**:
   - After calibration, **export** the pipeline JSON from web UI
   - Save to: `TeamCode/src/main/assets/AprilTags.vpr`
   - Redeploy app (it will auto-upload on next init)

### Option 2: Fallback to TY-Based Distance
If 3D calibration is not available, the code automatically falls back to TY-based trigonometry.
Ensure these config values are **accurate** (measure on your robot):

In [ShooterConfig.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configs/ShooterConfig.java#L177):
```java
public static double CAMERA_HEIGHT_CM     = 29.0;      // Measure from floor to camera lens
public static double TAG_CENTER_HEIGHT_CM = 75.0;      // AprilTag center height (from game spec)
public static double CAMERA_TILT_DEG      = 15.0;      // Angle camera tilts UP from horizontal
```

---

## Testing Your Fix

### Step 1: Run Limelight Health Check
Run the **"LL Health Check"** OpMode ([LimelightHealthCheck.java](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autoop/LimelightHealthCheck.java)):
```
OpMode: LL Health Check (Debug)
Expected output:
  ✓ HW Map: FOUND
  ✓ Pipeline: fiducial
  ✓ Fiducials: 1 tag(s) (or more)
  ✓ isValid: true
```

### Step 2: Check Distance Calculation
In any OpMode, add this telemetry:
```java
if (limelight != null) {
    telemetry.addData("FiducialCount", "...");  // Should show 1+ when tag visible
    telemetry.addData("DistanceCm", getDistanceCm());  // Should show positive value, not -1
    telemetry.addData("DistSource", "3D or TY");  // Shows which method worked
}
```

### Step 3: Verify No Crashes
- Run TeleOp with auto-aim enabled
- Point at a tag
- Watch for crashes or distance flickering

---

## Crash Root Cause Analysis

The intermittent crashes were caused by:

1. **Uncaught NullPointerException** from `pose.getPosition().toUnit(...)` 
   - `getPosition()` returned null, but code didn't check
   - Happened when tag was detected but 3D calibration was missing

2. **Silent Exception Swallowing** 
   - Code used `catch (Throwable ignored)` to hide the error
   - Made debugging much harder
   - **NOTE**: Exception handling was intentionally broad for robustness, but the underlying null check was missing

**Fixed by**: Adding null checks BEFORE calling methods on potentially-null objects

---

## Diagnostic Commands

### Check Pipeline 3D Status
```bash
# SSH into Limelight (if you have access)
curl http://192.168.43.1:5801/api/pipeline/config
# Look for: "fiducial_skip3d": 0  (0 = 3D enabled)
```

### Manual Distance Calculation (for TY method)
```java
double tyDeg = result.getTy();
double cameraTiltDeg = ShooterConfig.CAMERA_TILT_DEG;
double heightDeltaCm = ShooterConfig.TAG_CENTER_HEIGHT_CM - ShooterConfig.CAMERA_HEIGHT_CM;
double distCm = heightDeltaCm / Math.tan(Math.toRadians(cameraTiltDeg + tyDeg));
// If distCm < 0, your camera tilt or height config is wrong
```

---

## Configuration Checklist

- [ ] Limelight is powered and connected via USB to Control Hub
- [ ] Limelight web UI is accessible at `http://<IP>:5801`
- [ ] Camera calibration has been run for your pipeline resolution
- [ ] AprilTag pipeline is on pipeline slot 0 (or configured in `ShooterConfig.APRILTAG_PIPELINE`)
- [ ] `CAMERA_HEIGHT_CM`, `TAG_CENTER_HEIGHT_CM`, `CAMERA_TILT_DEG` are accurate
- [ ] `LIMELIGHT_ENABLED = true` in ShooterConfig.java
- [ ] Pipeline JSON exported from web UI and saved to `TeamCode/src/main/assets/AprilTags.vpr`

---

## If You Still Have Issues

1. **Distance still shows -1**: Camera calibration is missing. Follow "Option 1" above.
2. **Distance flickers**: Limelight is losing USB connection. Check cable, power, and Control Hub port.
3. **App still crashes**: Run LL Health Check first to isolate the problem.
4. **LED is RED or OFF**: Limelight not receiving power or wrong pipeline is active.

---

## References

- **Limelight 3A Docs**: https://limelightvision.io/docs
- **FTC AprilTag Field Coordinates**: https://ftc-docs.firstinspires.org
- **Camera Calibration Guide**: https://limelightvision.io/docs/camera-calibration
