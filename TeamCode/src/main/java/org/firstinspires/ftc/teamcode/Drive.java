package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {
    Controller controller;
    Drivetrain drive;

    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        drive = new Drivetrain(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            drive.drive2(controller.left_stick_y, controller.right_stick_y);
        }

    }
}
