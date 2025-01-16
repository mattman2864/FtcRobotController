package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    DcMotor rotator;
    DcMotor lift;
    DcMotor flipper;
    Servo wrist;
    CRServo intakeLeft;
    CRServo intakeRight;
    int targetRotator = 0;
    int targetLift = 0;
    int targetFlip = 0;
    double targetWrist;
    BasicPID rotatorController;
    BasicPID liftController;
    BasicPID flipperController;
    public enum Mode {
        HOME,
        FRONT_PICKUP,
        FRONT_PICKUP_DROP,
        REAR_PICKUP,
        REAR_PICKUP_DROP,
        HIGH_BASKET,
        LOW_BASKET
    }
    public Mode mode = Mode.HOME;
    DigitalChannel mag1;

    public Lift(HardwareMap map) {
        hardwareMap = map;
        rotator = hardwareMap.get(DcMotor.class, Config.Rotator.rotator);
        lift = hardwareMap.get(DcMotor.class, Config.Lift.lift);
        flipper = hardwareMap.get(DcMotor.class, Config.Arm.flipper);
        wrist = hardwareMap.get(Servo.class, Config.Intake.wrist);
        intakeLeft = hardwareMap.get(CRServo.class, Config.Intake.intakeLeft);
        intakeRight = hardwareMap.get(CRServo.class, Config.Intake.intakeRight);

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(targetRotator);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(targetLift);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setTargetPosition(targetFlip);
        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setPower(0.8);

    }
    private void setRotatorTarget(int target) {
        targetRotator = (int)clamp(target, Config.Rotator.rotatorMin, Config.Rotator.rotatorMax);
    }
    private void setLiftTarget(int target) {
        targetLift = (int)clamp(target, Config.Lift.liftMin, Config.Lift.liftMax);
    }
    private void setFlipTarget(int target) {
        targetFlip = (int)clamp(target, Config.Arm.flipMin, Config.Arm.flipMax);
    }
    public void intake(double speed) {
        speed *= 0.8; // servo speed coefficient
        intakeLeft.setPower(-speed);
        intakeRight.setPower(speed);
    }
    public void setMode(Mode mode) {
        this.mode = mode;
    }
    public void update() {
        // MANAGE STATE
        switch (mode) {
            case HOME:
                setFlipTarget(0);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP:
                setFlipTarget(1480);
                setLiftTarget(500);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP_DROP:
                setFlipTarget(1600);
                setLiftTarget(500);
                setRotatorTarget(0);
                targetWrist = 0.05;
                intake(1);
                break;
            case REAR_PICKUP:
                setFlipTarget(160);
                setLiftTarget(500);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                break;
            case REAR_PICKUP_DROP:
                setFlipTarget(160);
                setLiftTarget(10);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                intake(1);
                break;
            case HIGH_BASKET:
                setFlipTarget(1000);
                setLiftTarget(4500);
                setRotatorTarget(2847);
                targetWrist = 1;
                break;
            case LOW_BASKET:
                setFlipTarget(1200);
                setLiftTarget(2500);
                setRotatorTarget(3000);
                targetWrist = 1;
        }
        if (Math.abs(rotator.getCurrentPosition() - targetRotator) > 100) {
            if (mode == Mode.HOME) {
                flipper.setTargetPosition(targetFlip);
            }
            lift.setTargetPosition(targetLift);
            wrist.setPosition(0);
            if (Math.abs(lift.getCurrentPosition()) < 20) {
                rotator.setTargetPosition(targetRotator);
            }
        } else {
            lift.setTargetPosition(targetLift);
            flipper.setTargetPosition(targetFlip);
            wrist.setPosition(targetWrist);
        }
    }
}