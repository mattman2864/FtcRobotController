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

                if (lift.mode == Lift.Mode.FRONT_PICKUP || lift.mode == Lift.Mode.FRONT_PICKUP_DROP) {
                    if (controller.dpadDown()) {
                        lift.setMode(Lift.Mode.FRONT_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.FRONT_PICKUP);
                    }
                } else if (lift.mode == Lift.Mode.REAR_PICKUP || lift.mode == Lift.Mode.REAR_PICKUP_DROP) {
                    if (controller.dpadDown()) {
                        lift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.REAR_PICKUP);
                    }
                } else if (controller.dpadLeft()) {
                    lift.setMode(Lift.Mode.REAR_PICKUP);
                }

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

                telemetry.addLine("CONTROL: COMMAND");
                telemetry.addLine("A: HOME");
                telemetry.addLine("B: FRONTPICKUP PRESET");
                telemetry.addLine("B - DPAD DOWN: FRONTPICKUP PICKUP");
                telemetry.addLine("X: HIGH BASKET");
                telemetry.addLine("Y: LOW BASKET");
                telemetry.addLine("LEFT BUMPER/RIGHT BUMPER: SPECIMEN PLACE HIGH");
                telemetry.addLine("LB/RB - LB: SPECIMENPICKUP LOW");
                telemetry.addLine("LB/RB - RB: SPECIMENPICKUP HIGH");
                telemetry.addLine("RIGHT TRIGGER: INTAKE IN");
                telemetry.addLine("LEFT TRIGGER: INTAKE OUT");

                lift.update();
                telemetry.update();
                controller.update();
            }
        }
    }
}
