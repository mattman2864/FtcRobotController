package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    HardwareMap hardwareMap;
    public Drivetrain(HardwareMap map) {
        hardwareMap = map;
    }

    public DcMotor frontLeft = hardwareMap.get(DcMotor.class, RobotMap.Drivetrain.frontLeft);
    public DcMotor frontRight = hardwareMap.get(DcMotor.class, RobotMap.Drivetrain.frontRight);
    public DcMotor rearLeft = hardwareMap.get(DcMotor.class, RobotMap.Drivetrain.rearLeft);
    public DcMotor rearRight = hardwareMap.get(DcMotor.class, RobotMap.Drivetrain.rearRight);

    public void mecanumDrive(double theta, double power, double turn) {

        // Finding components of desired vector
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Calculating power for each motor based on desired vector
        double fl = power * cos/max + turn;
        double fr = power * sin/max - turn;
        double rl = power * sin/max + turn;
        double rr = power * cos/max - turn;

        // Normalizing motor powers so that the maximum is no greater than 1
        if ((power + Math.abs(turn)) > 1) {
            fl /= power + turn;
            fr /= power + turn;
            rl /= power + turn;
            rr /= power + turn;
        }

        // Applying calculated power to motors
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.rearLeft.setPower(rl);
        this.rearRight.setPower(rr);
    }

    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX) {

        // joystick y axis is reversed, where up is + and down is -
        double theta = Math.atan2(-leftStickY, leftStickX);
        double power = Math.hypot(leftStickX, -leftStickY);
        this.mecanumDrive(theta, power, rightStickX);
    }

    public void tankDrive(double leftStickY, double rightStickY) {
        // Basic tank drive for wheelie bot
        this.frontLeft.setPower(-leftStickY);
        this.rearLeft.setPower(-leftStickY);
        this.frontRight.setPower(-rightStickY);
        this.rearRight.setPower(-rightStickY);
    }
}