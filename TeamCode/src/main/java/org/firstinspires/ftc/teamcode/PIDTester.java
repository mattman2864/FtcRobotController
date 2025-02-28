package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@Config
@TeleOp(name = "PID Tester")
public class PIDTester extends LinearOpMode {
    public static int target = 0;
    public static double kG = 0;
    public static PIDCoefficients coefficients = new PIDCoefficients(0, 0, 0);

    @Override
    public void runOpMode() {
        DcMotor arm = hardwareMap.get(DcMotor.class, "flipper");
        OctoQuad octo = hardwareMap.get(OctoQuad.class, "octoquad");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DoubleSupplier motorPosition = () -> octo.readSinglePosition(0);
        BasicPID controller = new BasicPID(coefficients);
        NoFeedforward feedforward = new NoFeedforward();
        RawValue noFilter = new RawValue(motorPosition);
        BasicSystem system = new BasicSystem(noFilter, controller, feedforward);

        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry dashboard = dash.getTelemetry();
        Controller gamepad = new Controller(gamepad1);

        waitForStart();
        while(opModeIsActive()) {
            double angle = motorPosition.getAsDouble() / 4000 * Math.PI;
            double power = system.update(target) + kG * Math.cos(angle);
            arm.setPower(power);
            dashboard.addData("position", octo.readSinglePosition(0));
            dashboard.addData("target", target);
            dashboard.addData("power", power);
            dashboard.update();
            gamepad.update();
        }
    }
}
