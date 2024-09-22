package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    Controller controller;
    Robot robot;

    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        robot = new Robot(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.tankDrive(controller.left_stick_y, controller.right_stick_y);
        }

    }
}
