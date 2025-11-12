package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import android.util.Size;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class CameraController {

    public enum Mode
    {
        TAG_MODE,
        BLOB_MODE_UP,
        BLOB_MODE_DOWN
    }
    private final Servo cameraServo;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -180, 0, 0);
    private final ColorBlobLocatorProcessor  purpleLocator;

    private final ColorBlobLocatorProcessor  greenLocator;

    private AprilTagProcessor aprilTagLocator;

    private Mode cameraMode;

    private final Telemetry screenLogger;
    VisionPortal visionPortal;

    public CameraController(Servo servo, HardwareMap hardwareMap, Telemetry telemetry) {
        cameraServo = servo;
        screenLogger = telemetry;

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(0, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        aprilTagLocator = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessors(purpleLocator, greenLocator, aprilTagLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        swithMode(Mode.TAG_MODE);
    }

    public void Down() {
        cameraServo.setPosition(0.05);
    }

    public void Up() {
        cameraServo.setPosition(0.2);
    }

    public void Front() {
        cameraServo.setPosition(0.95);
    }

    public void swithMode(Mode mode) {
        switch(mode) {
            case TAG_MODE:
                Front();
                cameraMode = Mode.TAG_MODE;
                //visionPortal.stopStreaming();
                break;
            case BLOB_MODE_UP:
                Up();
                cameraMode = Mode.BLOB_MODE_UP;
                //visionPortal.stopStreaming();
                break;
            case BLOB_MODE_DOWN:
                Down();
                cameraMode = Mode.BLOB_MODE_DOWN;
                //visionPortal.resumeStreaming();
                break;
        }
    }

    public double get_artifact_location()
    {
        double closest_distance = 0;
        double closest_x = 0;
        if (cameraMode != Mode.TAG_MODE) {
            List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(purpleLocator.getBlobs());
            blobs.addAll(greenLocator.getBlobs());
            screenLogger.addData("Blobs Number", blobs.size());
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    250, 200000, blobs);  // filter out very small blobs.
            screenLogger.addData("Blobs Number1", blobs.size());
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.3, 1, blobs);     /* filter out non-circular blobs.
             * NOTE: You may want to adjust the minimum value depending on your use case.
             * Circularity values will be affected by shadows, and will therefore vary based
             * on the location of the camera on your robot and venue lighting. It is strongly
             * encouraged to test your vision on the competition field if your event allows
             * sensor calibration time.
             */
            screenLogger.addData("Blobs Number2", blobs.size());
            for (ColorBlobLocatorProcessor.Blob blob : blobs) {
                Circle circle = blob.getCircle();
                if (circle.getRadius() > closest_distance) {
                    closest_distance = circle.getRadius();
                    closest_x = circle.getX();
                }
            }
        }
        screenLogger.addData("Blob", closest_x);
        return closest_x;
    }

    public int get_mission_tag()
    {
        if (cameraMode == Mode.TAG_MODE) {
            List<AprilTagDetection> detections = aprilTagLocator.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == 21 || d.id == 22 || d.id == 23) {
                    return d.id;
                }
            }
        }
        return 0;
    }

    public Pose3D get_robot_pose(int id)
    {
        if (cameraMode == Mode.TAG_MODE)
        {
            List<AprilTagDetection> detections = aprilTagLocator.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == id) {
                    return d.robotPose;
                }
            }
        }
        return null;
    }
}
