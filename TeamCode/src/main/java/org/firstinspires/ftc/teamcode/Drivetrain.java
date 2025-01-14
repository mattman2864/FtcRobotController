package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

;

public class Drivetrain {
    HardwareMap hardwareMap;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    double speed;
    public Drivetrain(HardwareMap map) {
        hardwareMap = map;
        frontLeft = hardwareMap.get(DcMotor.class, Config.Drivetrain.frontLeft);
        frontRight = hardwareMap.get(DcMotor.class, Config.Drivetrain.frontRight);
        rearLeft = hardwareMap.get(DcMotor.class, Config.Drivetrain.rearLeft);
        rearRight = hardwareMap.get(DcMotor.class, Config.Drivetrain.rearRight);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        speed = 1;
    }
    public void setSpeed(double newSpeed) {
        this.speed = newSpeed;
    }
    public void mecanumDrive(double theta, double power, double turn, double speed) {
        // Finding components of desired vector
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Calculating power for each motor based o/home/matt/StudioProjects/WafflesRC/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Testingn desired vector
        double fl = power * cos/max + turn;
        double fr = power * sin/max - turn;
        double rl = power * sin/max + turn;
        double rr = power * cos/max - turn;

        // Normalizing motor powers so that the maximum is no greater than 1
        if ((power + Math.abs(turn)) > speed) {
            fl /= power + turn;
            fl *= speed;
            fr /= power + turn;
            fr *= speed;
            rl /= power + turn;
            rl *= speed;
            rr /= power + turn;
            rr *= speed;
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
        this.mecanumDrive(theta, power, rightStickX, speed);
    }

    public void tankDrive(double leftStickY, double rightStickY) {
        // Basic tank drive for wheelie bot
        this.frontLeft.setPower(-leftStickY);
        this.rearLeft.setPower(-leftStickY);
        this.frontRight.setPower(-rightStickY);
        this.rearRight.setPower(-rightStickY);
    }
}