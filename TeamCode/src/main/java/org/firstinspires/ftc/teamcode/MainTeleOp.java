package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    TeleOpControl control;
    Controller controller;
    @Override
    public void runOpMode() {
        controller = new Controller(gamepad1);
        control = new TeleOpControl(controller, hardwareMap);
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controller.update();
                control.drive();
                control.intake();
                control.lift();
                control.flipGrip(time);
                control.launch();
                telemetry.addData("lift position", control.lift.getPosition());
                telemetry.addData("lift power", control.lift.lift.getPower());
                telemetry.update();

            }
        }
    }
}