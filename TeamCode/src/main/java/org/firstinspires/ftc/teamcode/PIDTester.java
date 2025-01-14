package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PID Tester")
public class PIDTester extends LinearOpMode {
    DcMotor arm;
    DcMotor rotator;
    BasicPID armController;

    FtcDashboard dash;
    public static PIDCoefficients armCoefficients = new PIDCoefficients(0, 0, 0);
    int target;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class, "flipper");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController = new BasicPID(armCoefficients);
        target = 700;
        rotator = hardwareMap.get(DcMotor.class, "rotator");


        dash = FtcDashboard.getInstance();
        Telemetry dashboard = dash.getTelemetry();
        Controller controller = new Controller(gamepad1);

        waitForStart();
        while(opModeIsActive()) {
            if (controller.A()) target = 600;
            else target = 1400;

            arm.setPower(armController.calculate(target, arm.getCurrentPosition()));
            dashboard.addData("position", arm.getCurrentPosition());
            dashboard.addData("target", target);
            dashboard.update();
            controller.update();
        }
    }
}
