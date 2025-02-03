package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Lift {

    // Hardware classes
    HardwareMap hardwareMap;
    DcMotor rotator;
    DcMotor lift;
    DcMotor flipper;
    Servo wrist;
    CRServo intakeLeft;
    CRServo intakeRight;

    public int targetRotator = 0;
    public int targetLift = 0;
    public int targetFlip = 0;
    public double targetWrist;

    BasicPID rotatorController;
    BasicPID liftController;
    BasicPID flipperController;

    public int rotatorPosition;
    public int liftPosition;
    public int flipPosition;
    public double wristPosition;

    public enum Mode {
        HOME,
        FRONT_PICKUP,
        FRONT_PICKUP_DROP,
        REAR_PICKUP,
        REAR_PICKUP_DROP,
        HIGH_BASKET,
        LOW_BASKET,
        HANG,
        PARK,
        SPECIMAN_PICKUP,
        SPECIMAN_PRESET,
        SPECIMAN_PLACE_HIGH,
        TESTING,
    }
    public Mode mode = Mode.HOME;
    TouchSensor touch;
    ElapsedTime stateTimer;

    public Lift(HardwareMap map) {
        hardwareMap = map;
        rotator = hardwareMap.get(DcMotor.class, Config.Rotator.rotator);
        lift = hardwareMap.get(DcMotor.class, Config.Lift.lift);
        flipper = hardwareMap.get(DcMotor.class, Config.Arm.flipper);
        wrist = hardwareMap.get(Servo.class, Config.Intake.wrist);
        intakeLeft = hardwareMap.get(CRServo.class, Config.Intake.intakeLeft);
        intakeRight = hardwareMap.get(CRServo.class, Config.Intake.intakeRight);
        touch = hardwareMap.get(TouchSensor.class, "touch");

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(targetRotator);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(targetLift);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipper.setTargetPosition(targetFlip);
        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipper.setPower(0.8);

        stateTimer = new ElapsedTime();
        stateTimer.reset();
//      mag1 = hardwareMap.get(DigitalChannel.class, "mag1");
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
        if (mode != Mode.HOME && this.mode == Mode.HOME) {
            // Reset encoder position to prevent lift
            flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        this.mode = mode;
        stateTimer.reset();
    }

    public void update() {
        // MANAGE STATE
        // Modes can be defined here, need to be instantiated above
        switch (mode) {
            case HOME:
                setFlipTarget(0);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP:
                setFlipTarget(1400);
                setLiftTarget(200);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP_DROP:
                setFlipTarget(1550);
                setLiftTarget(200);
                setRotatorTarget(0);
                targetWrist = 0.05;
                intake(1);
                break;
            case REAR_PICKUP:
                setFlipTarget(160);
                setLiftTarget(500);
                setRotatorTarget(300);
                targetWrist = 0.68;
                break;
            case REAR_PICKUP_DROP:
                setFlipTarget(80);
                setLiftTarget(10);
                setRotatorTarget(2847);
                targetWrist = 0.68;
                intake(1);
                break;
            case HIGH_BASKET:
                setFlipTarget(1000);
                setLiftTarget(4300);
                setRotatorTarget(2847);
                targetWrist = 1;
                break;
            case LOW_BASKET:
                setFlipTarget(1200);
                setLiftTarget(1750);
                setRotatorTarget(2847);
                targetWrist = 1;
                break;
            case HANG:
                break;
            case PARK:
                break;
            case SPECIMAN_PICKUP:
                setFlipTarget(210);
                setLiftTarget(0);
                setRotatorTarget(1940);
                targetWrist = 0.3;
                break;
            case SPECIMAN_PLACE_HIGH:
                setFlipTarget(1550);
                setLiftTarget(1910);
                setRotatorTarget(2090);
                targetWrist = 0;
                break;
            case TESTING:
                setFlipTarget(targetFlip);
                setRotatorTarget(targetRotator);
                setLiftTarget(targetLift);
                wrist.setPosition(targetWrist);

                flipPosition = flipper.getCurrentPosition();
                rotatorPosition = rotator.getCurrentPosition();
                liftPosition = lift.getCurrentPosition();
                wristPosition = wrist.getPosition();
                break;
        }

        if (touch.isPressed() && mode == Mode.HOME) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if ((Math.abs(rotator.getCurrentPosition() - targetRotator) > 100)) {
            flipper.setTargetPosition(0);
            lift.setTargetPosition(0);
            wrist.setPosition(0);
            if (Math.abs(flipper.getCurrentPosition()) < 20 && Math.abs(lift.getCurrentPosition()) < 20) {
                rotator.setTargetPosition(targetRotator);
            }
        } else {
            lift.setTargetPosition(targetLift);
            flipper.setTargetPosition(targetFlip);
            wrist.setPosition(targetWrist);
            rotator.setTargetPosition(targetRotator);
            if (Math.abs(flipper.getCurrentPosition() - targetFlip) < 10 && mode == Mode.HOME) {
                flipper.setPower(0);
            } else {
                flipper.setPower(1);
            }
        }
    }
}