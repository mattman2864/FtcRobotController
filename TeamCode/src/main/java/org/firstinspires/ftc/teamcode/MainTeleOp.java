package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Drive;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    TeleOpControl control;
    Controller controller;
    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        control = new TeleOpControl(controller, hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controller.update();
                control.drive();
                control.intake();
                control.lift();
                control.flipGrip();
            }
        }
    }
}