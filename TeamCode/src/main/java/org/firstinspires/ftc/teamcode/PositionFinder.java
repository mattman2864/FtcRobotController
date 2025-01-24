package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="PositionFinder")
public class PositionFinder extends LinearOpMode {
    Lift lift;
    Drivetrain drive;
    Controller controller;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap);
        drive = new Drivetrain(hardwareMap);
        controller = new Controller(gamepad1);

        lift.setMode(Lift.Mode.TESTING);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry.addData("Right Trigger: ", controller.right_trigger);
                telemetry.addData("Left Trigger: ", controller.left_trigger);
                telemetry.addData("Right Bumper: ", controller.rightBumper());
                telemetry.addData("Left Bumper: ", controller.leftBumper());
                telemetry.addData("Dpad Left: ", controller.dpadLeft());
                telemetry.addData("Dpad Right: ", controller.dpadRight());

                if (controller.right_trigger > 0) {
                    lift.targetLift += 10;
                } else if (controller.left_trigger > 0) {
                    lift.targetLift -= 10;
                } else if (controller.rightBumper()) {
                    lift.targetRotator += 10;
                } else if (controller.leftBumper()) {
                    lift.targetRotator -= 10;
                } else if (controller.dpadLeft()) {
                    lift.targetFlip += 3;
                } else if (controller.dpadRight()) {
                    lift.targetFlip -= 3;
                } else if (controller.dpadUp()) {
                    lift.targetWrist += 0.01;
                } else if (controller.dpadDown()) {
                    lift.targetWrist -= 0.01;
                }

                if (lift.mode == Lift.Mode.HOME) drive.setSpeed(1);
                else drive.setSpeed(0.4);

                if (controller.A()) {
                    lift.setMode(Lift.Mode.TESTING);
                }
                if (controller.B()) {
                    lift.setMode(Lift.Mode.SPECIMAN_PICKUP);
                }
                if (controller.Y()) {
                    lift.setMode(Lift.Mode.SPECIMAN_PRESET);
                }
                if (controller.X()) {
                    lift.setMode(Lift.Mode.SPECIMAN_PLACE);
                }
                // Intake
                //lift.intake(controller.right_trigger - controller.left_trigger);

                //Drive
                drive.joystickDrive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);

                lift.update();
                telemetry.update();
                controller.update();

                telemetry.addData("Rotator Position: ", lift.rotatorPosition);
                telemetry.addData("Target Rotator: ", lift.targetRotator);
                telemetry.addData("Flipper Position: ", lift.flipPosition);
                telemetry.addData("Target Flipper: ", lift.targetFlip);
                telemetry.addData("Lift Position: ", lift.liftPosition);
                telemetry.addData("Target Lift: ", lift.targetLift);
                telemetry.addData("Wrist Position: ", lift.wristPosition);
                telemetry.addData("Target Wrist: ", lift.targetWrist);

                telemetry.addData("Mode: ", lift.mode);
            }
        }
    }
}
