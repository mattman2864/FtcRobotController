package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Photon
@TeleOp (name="Main")
public class TeleOpMain extends LinearOpMode {
    Robot robot;
    Controller controller;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        controller = new Controller(gamepad1);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (controller.A()) {
                    robot.lift.setMode(Lift.Mode.HOME);
                } else if (controller.B()) {
                    robot.lift.setMode(Lift.Mode.PICKUP);
                } else if (controller.X()) {
                    robot.lift.setMode(Lift.Mode.HIGH_BASKET);
                } else if (controller.Y()) {
                    robot.lift.setMode(Lift.Mode.LOW_BASKET);
                }
                robot.drivetrain.joystickDrive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
                robot.update();
                telemetry.update();
                controller.update();
            }
        }
    }
}
