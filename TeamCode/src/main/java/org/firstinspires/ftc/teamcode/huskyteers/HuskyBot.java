package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.OptionalDouble;

abstract public class HuskyBot extends LinearOpMode {
    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     * <p>
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     * <p>
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     * <p>
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     * <p>
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    public MecanumDrive drive;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public boolean seesTag(int tagNumber) {
        return aprilTag.getDetections().stream().anyMatch((aprilTagDetection) ->
             aprilTagDetection.id == tagNumber
        );
    }



    public void instantiateMotors(Pose2d pose) {
        drive = new MecanumDrive(hardwareMap, pose);
    }

    public void driveRobot(double drive, double strafe, double turn, double speed) {
        PoseVelocity2d pw = new PoseVelocity2d(
                new Vector2d(
                        -drive * speed,
                        strafe * speed
                ), turn * speed
        );

        this.drive.setDrivePowers(pw);
    }

    public void fieldCentricDriveRobot(double gamepadLeftStickY, double gamepadLeftStickX, double gamepadRightStickX, double speed) {
        drive.updatePoseEstimate();

        Vector2d angleVector = this.drive.pose.heading.vec();
        double angle = -Math.atan2(angleVector.y, angleVector.x);

        double rotatedX = gamepadLeftStickX * Math.cos(angle) - gamepadLeftStickY * Math.sin(angle);
        double rotatedY = gamepadLeftStickX * Math.sin(angle) + gamepadLeftStickY * Math.cos(angle);

        driveRobot(rotatedY, rotatedX, gamepadRightStickX, speed);
    }

    public void localizeRobot() {
        ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
        OptionalDouble averageX = detections.stream().mapToDouble(aprilTagDetection -> aprilTagDetection.robotPose.getPosition().x).average();
        OptionalDouble averageY = detections.stream().mapToDouble(aprilTagDetection -> aprilTagDetection.robotPose.getPosition().y).average();
        OptionalDouble averageYaw = detections.stream().mapToDouble(aprilTagDetection -> aprilTagDetection.robotPose.getOrientation().getYaw()).average();
        if (averageX.isPresent() && averageY.isPresent() && averageYaw.isPresent()) {
            this.drive.pose = new Pose2d(averageX.getAsDouble(), averageY.getAsDouble(), averageYaw.getAsDouble());
        }
    }
}
