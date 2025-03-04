package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Single Driver")
public class SingleDriver extends LinearOpMode {
    Lift lift;
    Drivetrain drive;
    Controller controller;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap);
        drive = new Drivetrain(hardwareMap);
        controller = new Controller(gamepad1);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (controller.A()) {
                    lift.setMode(Lift.Mode.HOME);
                } else if (controller.B()) {
                    lift.setMode(Lift.Mode.FRONT_PICKUP);
                } else if (controller.X()) {
                    lift.setMode(Lift.Mode.HIGH_BASKET);
                } else if (controller.Y()) {
                    lift.setMode(Lift.Mode.LOW_BASKET);
                } else if (controller.leftBumper()) {
                    lift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                } else if (controller.rightBumper()) {
                    lift.setMode(Lift.Mode.SPECIMEN_PICKUP);
                } else if (controller.dpadUp()) {
                    lift.setMode(Lift.Mode.HANG);
                }

                if (lift.mode == Lift.Mode.HIGH_BASKET) {
                    drive.setSpeed(0.4);
                } else {
                    drive.setSpeed(1);
                }

                lift.fineTuneFlipper(controller.right_stick_y);

                // Intake
                lift.intake(controller.right_trigger - controller.left_trigger);

                //Drive
                drive.joystickDrive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
                telemetry.addData("flip target", lift.targetFlip);
                telemetry.addData("flip position", lift.flipper.getCurrentPosition());
                telemetry.addData("flip power", lift.flipper.getPower());
                telemetry.addData("rotator target", lift.targetRotator);
                telemetry.addData("rotator position", lift.rotator.getCurrentPosition());
                telemetry.addData("rotator power", lift.rotator.getPower());
                telemetry.addData("stateTimer", lift.stateTimer.milliseconds());
                telemetry.addData("motorMode", lift.flipper.getMode());
                telemetry.addData("stateMode", lift.mode);
                lift.update();
                telemetry.update();
                controller.update();
            }
        }
    }
}
