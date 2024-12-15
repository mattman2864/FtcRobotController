package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
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
    int targetRotator = 0;
    int targetLift = 0;
    BasicPID rotatorController;
    BasicPID liftController;
    enum State {
        OUT,
        IN
    }
    enum Mode {
        HOME,
        PICKUP,
        HIGH_BASKET,
        LOW_BASKET
    }
    State state = State.IN;
    Mode mode = Mode.HOME;
    int startingRotation = 0;
    int startingLift = 0;

    public Lift (HardwareMap map) {
        hardwareMap = map;
        rotator = hardwareMap.get(DcMotor.class, Config.Rotator.rotator);
        lift = hardwareMap.get(DcMotor.class, Config.Lift.lift);
        flipRight = hardwareMap.get(Servo.class, Config.Arm.flipRight);
        flipLeft = hardwareMap.get(Servo.class, Config.Arm.flipLeft);

//        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    void setRotatorTarget(int target) {
        startingRotation = rotator.getCurrentPosition();
        targetRotator = clamp(target, Config.Rotator.rotatorMin, Config.Rotator.rotatorMax);
    }
    void setLiftTarget(int target) {
        targetLift = clamp(target, Config.Lift.liftMin, Config.Lift.liftMax);
    }
    void setFlip(double val){
        flipRight.setPosition(val);
        flipLeft.setPosition(val);
    }
    public void setMode(Mode mode) {
        this.mode = mode;
    }
    void update() {
        // MANAGE STATE
        switch (mode) {
            case HOME:
                setFlip(0);
                setLiftTarget(10);
                setRotatorTarget(0);
                break;
            case PICKUP:
                setFlip(0.85);
                setLiftTarget(2000);
                setRotatorTarget(0);
                break;
            case HIGH_BASKET:
                setFlip(0.5);
                setLiftTarget(3000);
                setRotatorTarget(3500);
                break;
            case LOW_BASKET:
                setFlip(0.5);
                setLiftTarget(1200);
                setRotatorTarget(3500);
        }

        // UPDATE POSITION
        // LIFT
//        if ((float) (Math.abs(rotator.getCurrentPosition() - startingRotation)) / (float) (Math.abs(targetRotator - startingRotation + 1)) > 0.5 ||
//                     Math.abs(rotator.getCurrentPosition() - startingRotation) < 100) {
            lift.setPower(liftController.calculate(targetLift, lift.getCurrentPosition()));
//        } else {
//            lift.setPower(liftController.calculate(10, lift.getCurrentPosition()));
//        }

        // ROTATOR
//        if (Math.abs(lift.getCurrentPosition() / (targetLift + 1)) < 0.5 || lift.getCurrentPosition() < 500 ||
//            (float) (Math.abs(rotator.getCurrentPosition() - startingRotation)) / (float) (Math.abs(targetRotator - startingRotation + 1)) > 0.5) {
            rotator.setPower(rotatorController.calculate(targetRotator, rotator.getCurrentPosition()));
//        } else {
//            rotator.setPower(rotatorController.calculate(rotator.getCurrentPosition(), rotator.getCurrentPosition()));
//        }
    }
}