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
                } else if (controller.dpadUp()) {
                    lift.setMode(Lift.Mode.REAR_PICKUP);
                } else if (controller.X()) {
                    lift.setMode(Lift.Mode.HIGH_BASKET);
                } else if (controller.Y()) {
                    lift.setMode(Lift.Mode.LOW_BASKET);
                }
                if (lift.mode == Lift.Mode.FRONT_PICKUP || lift.mode == Lift.Mode.FRONT_PICKUP_DROP) {
                    if (controller.dpadDown()) {
                        lift.setMode(Lift.Mode.FRONT_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.FRONT_PICKUP);
                    }
                }
                if (lift.mode == Lift.Mode.REAR_PICKUP || lift.mode == Lift.Mode.REAR_PICKUP_DROP) {
                    if (controller.dpadDown()) {
                        lift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.REAR_PICKUP);
                    }
                }

                if (lift.mode == Lift.Mode.HOME) drive.setSpeed(1);
                else drive.setSpeed(0.4);

                // Intake
                lift.intake(controller.right_trigger - controller.left_trigger);

                //Drive
                drive.joystickDrive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);

                lift.update();
                telemetry.update();
                controller.update();
            }
        }
    }
}
