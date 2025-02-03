package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="Two Driver")
public class TwoDriver extends LinearOpMode {
    Lift lift;
    Drivetrain drive;
    Controller controller1;
    Controller controller2;
    @Override
    public void runOpMode() {
        lift = new Lift(hardwareMap);
        drive = new Drivetrain(hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (lift.mode == Lift.Mode.FRONT_PICKUP || lift.mode == Lift.Mode.FRONT_PICKUP_DROP) {
                    if (controller1.dpadDown()) {
                        lift.setMode(Lift.Mode.FRONT_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.FRONT_PICKUP);
                    }
                } else if (lift.mode == Lift.Mode.REAR_PICKUP || lift.mode == Lift.Mode.REAR_PICKUP_DROP) {
                    if (controller1.dpadDown()) {
                        lift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    } else {
                        lift.setMode(Lift.Mode.REAR_PICKUP);
                    }
                } else if (controller1.dpadLeft()) {
                    lift.setMode(Lift.Mode.REAR_PICKUP);
                }

                if (controller1.A()) {
                    lift.setMode(Lift.Mode.HOME);
                } else if (controller1.B()) {
                    lift.setMode(Lift.Mode.FRONT_PICKUP);
                } else if (controller1.X()) {
                    lift.setMode(Lift.Mode.HIGH_BASKET);
                } else if (controller1.Y()) {
                    lift.setMode(Lift.Mode.LOW_BASKET);
                } else if (controller1.leftBumper()) {
                    lift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                } else if (controller1.rightBumper()) {
                    lift.setMode(Lift.Mode.SPECIMEN_PICKUP);
                } else if (controller1.dpadUp()) {
                    lift.setMode(Lift.Mode.HANG);
                }

                if (lift.mode == Lift.Mode.HIGH_BASKET) {
                    drive.setSpeed(0.4);
                } else {
                    drive.setSpeed(1);
                }

                lift.fineTuneFlipper(controller1.right_stick_y);
                lift.fineTuneLift(-controller1.left_stick_y);

                // Intake
                lift.intake(controller1.right_trigger - controller1.left_trigger);

                //Drive
                drive.joystickDrive(controller2.left_stick_x, controller2.left_stick_y, controller2.right_stick_x);

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
                controller1.update();
                controller2.update();
            }
        }
    }
}
