package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    int targetRotatorFinal = 0;
    int targetLift = 0;
    int targetLiftFinal = 0;
    int targetFlip;
    int targetFlipFinal = 0;
    double targetWrist;
    BasicPID rotatorController;
    BasicPID liftController;
    BasicPID flipperController;
    enum State {
        OUT,
        IN
    }
    public enum Mode {
        HOME,
        FRONT_PICKUP,
        FRONT_PICKUP_DROP,
        REAR_PICKUP,
        REAR_PICKUP_DROP,
        HIGH_BASKET,
        LOW_BASKET
    }
    State state = State.IN;
    public Mode mode = Mode.HOME;
    int startingRotation = 0;
    int startingLift = 0;
    public Lift(HardwareMap map) {
        hardwareMap = map;
        rotator = hardwareMap.get(DcMotor.class, Config.Rotator.rotator);
        lift = hardwareMap.get(DcMotor.class, Config.Lift.lift);
        flipper = hardwareMap.get(DcMotor.class, Config.Arm.flipper);
        wrist = hardwareMap.get(Servo.class, Config.Intake.wrist);
        intakeLeft = hardwareMap.get(CRServo.class, Config.Intake.intakeLeft);
        intakeRight = hardwareMap.get(CRServo.class, Config.Intake.intakeRight);

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDCoefficients rotatorCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        rotatorController = new BasicPID(rotatorCoefficients);

        PIDCoefficients liftCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        liftController = new BasicPID(liftCoefficients);

        PIDCoefficients flipperCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        flipperController = new BasicPID(flipperCoefficients);
    }
    private void setRotatorTarget(int target) {
        targetRotatorFinal = (int)clamp(target, Config.Rotator.rotatorMin, Config.Rotator.rotatorMax);
    }
    private void setLiftTarget(int target) {
        targetLiftFinal = (int)clamp(target, Config.Lift.liftMin, Config.Lift.liftMax);
    }
    private void setFlipTarget(int target) {
        targetFlipFinal = (int)clamp(target, Config.Arm.flipMin, Config.Arm.flipMax);
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
                setFlipTarget(740);
                setLiftTarget(500);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP_DROP:
                setFlipTarget(800);
                setLiftTarget(500);
                setRotatorTarget(0);
                targetWrist = 0.05;
                intake(1);
                break;
            case REAR_PICKUP:
                setFlipTarget(80);
                setLiftTarget(500);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                break;
            case REAR_PICKUP_DROP:
                setFlipTarget(80);
                setLiftTarget(10);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                intake(1);
                break;
            case HIGH_BASKET:
                setFlipTarget(600);
                setLiftTarget(4500);
                setRotatorTarget(3000);
                targetWrist = 1;
                break;
            case LOW_BASKET:
                setFlipTarget(600);
                setLiftTarget(1750);
                setRotatorTarget(3000);
                targetWrist = 1;
        }
        if (Math.abs(rotator.getCurrentPosition() - targetRotatorFinal) > 100) {
            targetFlip = 20;
            targetLift = 0;
            wrist.setPosition(0);
            if (Math.abs(flipper.getCurrentPosition()) < 20 && Math.abs(lift.getCurrentPosition()) < 20) {
                targetRotator = targetRotatorFinal;
            }
        } else {
            targetLift = targetLiftFinal;
            targetFlip = targetFlipFinal;
            wrist.setPosition(targetWrist);
        }

        lift.setPower(liftController.calculate(targetLift, lift.getCurrentPosition()));
        rotator.setPower(rotatorController.calculate(targetRotator, rotator.getCurrentPosition()));
        flipper.setPower(clamp(flipperController.calculate(targetFlip, flipper.getCurrentPosition()), -0.5, 0.5));
    }
}