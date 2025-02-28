package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public void joystickDrive(double leftStickX, double leftStickY, double rightStickX) {
        // joystick y axis is reversed, where up is + and down is -
        double y = -(leftStickY * leftStickY) * (leftStickY >= 0 ? 1 : -1);
        double x = (leftStickX * leftStickX) * (leftStickX >= 0 ? 1 : -1); // Counter imperfect strafing
        double rx = (rightStickX * rightStickX) * (rightStickX >= 0 ? 1 : -1);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double fl = (y + x + rx) / denominator;
        double rl = (y - x + rx) / denominator;
        double fr = (y - x - rx) / denominator;
        double rr = (y + x - rx) / denominator;

        // Applying speed
        fl *= speed;
        rl *= speed;
        fr *= speed;
        rr *= speed;

        // Applying calculated power to motors
        this.frontLeft.setPower(fl);
        this.frontRight.setPower(fr);
        this.rearLeft.setPower(rl);
        this.rearRight.setPower(rr);
    }

    public void tankDrive(double leftStickY, double rightStickY) {
        // Basic tank drive for wheelie bot
        this.frontLeft.setPower(-leftStickY);
        this.rearLeft.setPower(-leftStickY);
        this.frontRight.setPower(-rightStickY);
        this.rearRight.setPower(-rightStickY);
    }
}