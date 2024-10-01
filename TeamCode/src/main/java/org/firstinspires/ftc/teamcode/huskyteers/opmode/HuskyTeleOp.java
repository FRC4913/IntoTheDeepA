package org.firstinspires.ftc.teamcode.huskyteers.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.huskyteers.HuskyBot;
import org.firstinspires.ftc.teamcode.huskyteers.utils.GamepadUtils;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class HuskyTeleOp extends HuskyBot {

    private static final Pose2d TEAM_ZONE = new Pose2d(0, 0, 0); // Example coordinates
    private static final Pose2d TEAM_SUBMERSIBLE = new Pose2d(1, 1, Math.PI / 2); // Example coordinates
    private static final Pose2d BASKET = new Pose2d(2, 2, Math.PI); // Example coordinates

    private AprilTagDetectionPipeline aprilTagDetectionPipeline;  // Pipeline for detecting AprilTags

    @Override
    public void runOpMode() {
        instantiateMotors(new Pose2d(0, 0, 0));
        initAprilTag();

        waitForStart();
        if (isStopRequested()) return;
        GamepadUtils gamepad1Utils = new GamepadUtils();
        GamepadUtils gamepad2Utils = new GamepadUtils();
        gamepad1Utils.addRisingEdge("start", (pressed) -> drive.pose = new Pose2d(this.drive.pose.position, 0));

        AtomicBoolean usingFieldCentric = new AtomicBoolean(false);

        gamepad1Utils.addRisingEdge("a", (pressed) -> {
            usingFieldCentric.set(!usingFieldCentric.get());
            gamepad1.rumble(200);
        });
        gamepad1Utils.addRisingEdge("dpad_up", (pressed) -> visionPortal.stopStreaming());
        gamepad1Utils.addRisingEdge("dpad_up", (pressed) -> visionPortal.resumeStreaming());

        // Align the robot to specific positions based on AprilTag detection when right bumper is pressed
        gamepad1Utils.addRisingEdge("right_bumper", (pressed) -> {
            int tagID = getDetectedAprilTagID();  // Method to get the ID of detected AprilTag
            if (tagID != -1) {  // If a tag is detected
                switch (tagID) {
                    case 1: // Align to Team Zone
                        driveToPose(TEAM_ZONE);
                        break;
                    case 2: // Align to Team Submersible
                        driveToPose(TEAM_SUBMERSIBLE);
                        break;
                    case 3: // Align to Basket
                        driveToPose(BASKET);
                        break;
                    default:
                        telemetry.addData("AprilTag", "Unknown tag detected");
                        break;
                }
            } else {
                telemetry.addData("AprilTag", "No tag detected");
            }
            telemetry.update();
        });

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Utils.processUpdates(gamepad1);
            gamepad2Utils.processUpdates(gamepad2);



            localizeRobot();

            double speed = (0.35 + 0.5 * gamepad1.left_trigger);
            if (usingFieldCentric.get()) {
                telemetry.addData("Drive Mode", "Field Centric");
                fieldCentricDriveRobot(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, speed);
            } else {
                telemetry.addData("Drive Mode", "Tank");
                driveRobot(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, speed);
            }

            telemetry.update();
            sleep(20);
        }
        visionPortal.close();

    }
    private int getDetectedAprilTagID() {
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();  // Get latest detections
        if (detections.size() > 0) {
            return detections.get(0).id;  // Return the ID of the first detected AprilTag
        } else {
            return -1;  // Return -1 if no tag is detected
        }
    }

    // Method to drive to a specific Pose2d position
    private void driveToPose(Pose2d targetPose) {
        // Add logic to drive the robot to the targetPose (using PID or other control mechanism)
        // Example:
        // this.drive.followTrajectory(new TrajectoryBuilder(drive.pose, new Constraints())
        //          .splineTo(targetPose.position, targetPose.heading)
        //          .build());
        telemetry.addData("Driving to", targetPose);
        telemetry.update();
    }
}

