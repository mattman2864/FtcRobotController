package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                if (gamepad1.a) {
                    robot.lift.home();
                } else if (gamepad1.b) {
                    robot.lift.intake();
                }
                robot.lift.update();
                telemetry.update();
            }
        }
    }
}
