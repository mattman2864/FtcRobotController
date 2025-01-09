package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    DcMotor rotator;
    DcMotor lift;
    Servo flipRight;
    Servo flipLeft;
    Servo wrist;
    CRServo intakeLeft;
    CRServo intakeRight;
    int targetRotator = 0;
    int targetRotatorFinal = 0;
    int targetLift = 0;
    int targetLiftFinal = 0;
    double targetFlip;
    double targetWrist;
    BasicPID rotatorController;
    BasicPID liftController;

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
        flipRight = hardwareMap.get(Servo.class, Config.Arm.flipRight);
        flipLeft = hardwareMap.get(Servo.class, Config.Arm.flipLeft);
        wrist = hardwareMap.get(Servo.class, Config.Intake.wrist);
        intakeLeft = hardwareMap.get(CRServo.class, Config.Intake.intakeLeft);
        intakeRight = hardwareMap.get(CRServo.class, Config.Intake.intakeRight);

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        flipLeft.setDirection(Servo.Direction.REVERSE);

        PIDCoefficients rotatorCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        rotatorController = new BasicPID(rotatorCoefficients);

        PIDCoefficients liftCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        liftController = new BasicPID(liftCoefficients);
    }

    private void setRotatorTarget(int target) {
        targetRotatorFinal = (int)clamp(target, Config.Rotator.rotatorMin, Config.Rotator.rotatorMax);
    }

    private void setLiftTarget(int target) {
        targetLiftFinal = (int)clamp(target, Config.Lift.liftMin, Config.Lift.liftMax);
    }

    private void setFlip(double val) {
        flipRight.setPosition(val);
        flipLeft.setPosition(val);
    }

    public void intake(double speed) {
        speed *= 0.8; // servo speed coefficient
        intakeLeft.setPower(-speed);
        intakeRight.setPower(speed);
    }
    public void setMode(Mode mode) {
        this.mode = mode;
    }
    void update() {
        // MANAGE STATE
        switch (mode) {
            case HOME:
                targetFlip = 0;
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP:
                targetFlip = 0.7;
                setLiftTarget(2000);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP_DROP:
                targetFlip = 0.75;
                setLiftTarget(2000);
                setRotatorTarget(0);
                targetWrist = 0.05;
                intake(1);
                break;
            case REAR_PICKUP:
                targetFlip = 0.08;
                setLiftTarget(500);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                break;
            case REAR_PICKUP_DROP:
                targetFlip = 0.08;
                setLiftTarget(200);
                setRotatorTarget(3500);
                targetWrist = 0.68;
                intake(1);
                break;
            case HIGH_BASKET:
                targetFlip = 0.5;
                setLiftTarget(3500);
                setRotatorTarget(3500);
                targetWrist = 0.7;
                break;
            case LOW_BASKET:
                targetFlip = 0.8;
                setLiftTarget(1200);
                setRotatorTarget(3500);
                targetWrist = 0.7;
        }
        if (Math.abs(rotator.getCurrentPosition() - targetRotatorFinal) > 10) {
            if (Math.abs(lift.getCurrentPosition()) > 10) {
                // First Stage
                targetLift = 0;
                setFlip(0.1);
                wrist.setPosition(0);
            } else {
                // Second Stage
                targetRotator = targetRotatorFinal;
            }
        } else {
            // Third Stage
            setFlip(targetFlip);
            wrist.setPosition(targetWrist);
            targetLift = targetLiftFinal;
            targetRotator = targetRotatorFinal;
        }




        lift.setPower(liftController.calculate(targetLift, lift.getCurrentPosition()));
        rotator.setPower(rotatorController.calculate(targetRotator, rotator.getCurrentPosition()));
    }
}