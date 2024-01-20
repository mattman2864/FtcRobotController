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
        ElapsedTime liftTime = new ElapsedTime();
        ElapsedTime intakeTime = new ElapsedTime();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controller.update();
                control.drive();
                control.intake(intakeTime, gamepad1);
                control.lift(liftTime);
                control.launch();
                telemetry.addData("lift state", control.lift.liftState);
                telemetry.addData("hold state", control.lift.holdState);
                telemetry.addData("current position", control.lift.lift.getCurrentPosition());
                telemetry.addData("target position", control.lift.lift.getTargetPosition());
                telemetry.addData("lift mode", control.lift.lift.getMode());
                telemetry.update();

            }
        }
    }
}