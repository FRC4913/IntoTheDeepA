package org.firstinspires.ftc.teamcode.huskyteers.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.huskyteers.HuskyBot;
import org.firstinspires.ftc.teamcode.huskyteers.utils.GamepadUtils;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp
public class HuskyTeleOp extends HuskyBot {
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
        gamepad1Utils.addRisingEdge("right_bumper", (pressed) -> {
            if (seesTag(0)) {

            };

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
}
