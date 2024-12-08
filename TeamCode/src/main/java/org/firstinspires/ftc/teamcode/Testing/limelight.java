package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "limelight")
public class limelight extends LinearOpMode {
    Limelight3A limelight;
    Servo servo;
    FtcDashboard dashboard;
    double angle = 0;
    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "servo");
        telemetry.setMsTransmissionInterval(11);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        limelight.pipelineSwitch(0);
        double angle = 0;
        double servopos = 0;

        limelight.start();

        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && !result.getColorResults().isEmpty()) {
                List<List<Double>> corners = result.getColorResults().get(0).getTargetCorners();
                double maxhyp = 0;
                double maxx = 0;
                double maxy = 0;
                for (int i = 0; i < corners.size(); i++) {
                    int j = (i + 1) % corners.size();
                    double x = corners.get(i).get(0)-corners.get(j).get(0);
                    double y = corners.get(i).get(1)-corners.get(j).get(1);
                    double hyp = Math.hypot(x, y);
                    if (hyp > maxhyp) {
                        maxhyp = hyp;
                        maxx = x;
                        maxy = y;
                    }
                }
                double error = -Math.atan(maxy / maxx) - angle;
                if (error > Math.PI/2) {
                    error = -Math.atan(maxy/maxx) - (angle + Math.PI);
                    telemetry.addLine("flip");
                } else {
                    telemetry.addLine("no flip");
                }
                angle += error * 1;
                angle %= Math.PI/2;
                servopos = Math.max(0, Math.min(1, (angle+Math.PI/2)/(Math.PI)));
                if (result.isValid()) {
                    servo.setPosition(servopos);
                }
            }
            if (result != null) {
                telemetry.addLine("found");
                if (!result.getDetectorResults().isEmpty()){
                    telemetry.addLine("not empty");
                }
            }
            telemetry.addData("angle", angle);
            telemetry.addData("servopos", servopos);
            telemetry.addLine("test");
            telemetry.update();

        }
    }
}