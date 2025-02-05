package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
        SPECIMEN_PICKUP,
        SPECIMEN_PLACE_LOW,
        SPECIMEN_PLACE_HIGH,
        HANG,
        PARK,
        TESTING,
    }
    public Mode mode = Mode.HOME;
    TouchSensor touch;
    ElapsedTime stateTimer;
    double adjustFlip = 0;
    double adjustLift = 0;
    ColorSensor color;
    Servo led;

    public Lift(HardwareMap map) {
        hardwareMap = map;
        rotator = hardwareMap.get(DcMotor.class, Config.Rotator.rotator);
        lift = hardwareMap.get(DcMotor.class, Config.Lift.lift);
        flipper = hardwareMap.get(DcMotor.class, Config.Arm.flipper);
        wrist = hardwareMap.get(Servo.class, Config.Intake.wrist);
        intakeLeft = hardwareMap.get(CRServo.class, Config.Intake.intakeLeft);
        intakeRight = hardwareMap.get(CRServo.class, Config.Intake.intakeRight);
        touch = hardwareMap.get(TouchSensor.class, "touch");
        color = hardwareMap.get(ColorSensor.class, Config.Color.color);
        led = hardwareMap.get(Servo.class, Config.Color.led);

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
    public void setMode(Mode newMode) {
        this.mode = newMode;
        stateTimer.reset();
    }

    public void fineTuneFlipper(double amount) {
        adjustFlip = amount;
    }

    public void fineTuneLift(double amount) {
        adjustLift = amount;
    }

    private void updateLED() {
        if (color.red() > color.blue() && color.red() > color.green() && color.red() > 130) {
            led.setPosition(0.28); // RED
        } else if (color.blue() > color.green() && color.blue() > 130) {
            led.setPosition(0.65); // BLUE
        } else if (color.green() > 300) {
            led.setPosition(0.35); // YELLOW
        } else {
            led.setPosition(0); // OFF
        }
    }

    public void update() {
        int flipTweak = (int)(100 * adjustFlip);
        int liftTweak = (int)(300 * adjustLift);
        // MANAGE STATE
        switch (mode) {
            case HOME:
                setFlipTarget(0);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP:
                setFlipTarget(1300);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0;
                break;
            case FRONT_PICKUP_DROP:
                setFlipTarget(1550);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0.05;
                intake(1);
                break;
            case REAR_PICKUP:
                setFlipTarget(60);
                setLiftTarget(200);
                setRotatorTarget(3000);
                targetWrist = 0.68;
                break;
            case REAR_PICKUP_DROP:
                setFlipTarget(0);
                setLiftTarget(0);
                setRotatorTarget(3000);
                targetWrist = 0.68;
                intake(0.8);
                break;
            case HIGH_BASKET:
                setFlipTarget(1000 - flipTweak);
                setLiftTarget(4400 + liftTweak);
                setRotatorTarget(3000);
                targetWrist = 1;
                break;
            case LOW_BASKET:
                setFlipTarget(1200 - flipTweak);
                setLiftTarget(1000 + liftTweak);
                setRotatorTarget(3000);
                targetWrist = 1;
                break;
            case SPECIMEN_PICKUP:
                setFlipTarget(153 - flipTweak);
                setLiftTarget(60);
                setRotatorTarget(1940);
                targetWrist = 0.33;
                break;
            case SPECIMEN_PLACE_LOW:
                setFlipTarget(1670 + flipTweak);
                setLiftTarget(liftTweak);
                setRotatorTarget(1940);
                targetWrist = 0.12;
                break;
            case SPECIMEN_PLACE_HIGH:
                setFlipTarget(1550 + flipTweak);
                setLiftTarget(2130 + liftTweak);
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
            case HANG:
                setFlipTarget(0);
                setLiftTarget(1390);
                setRotatorTarget(3000);
                targetWrist = 0;
                break;
            case PARK:
                setFlipTarget(850);
                setLiftTarget(0);
                setRotatorTarget(0);
                targetWrist = 0.4;
                break;
        }

        if (touch.isPressed() && mode == Mode.HOME) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if ((Math.abs(rotator.getCurrentPosition() - targetRotator) > 100)) {
            if (mode != Mode.SPECIMEN_PLACE_HIGH) {
                flipper.setTargetPosition(0);
            }
            lift.setTargetPosition(50);
            wrist.setPosition(0);
            if (Math.abs(flipper.getCurrentPosition()) < 50 && Math.abs(lift.getCurrentPosition()) < 100 || stateTimer.milliseconds() > 1000) {
                rotator.setTargetPosition(targetRotator);
            }
        } else {
            lift.setTargetPosition(targetLift);
            flipper.setTargetPosition(targetFlip);
            wrist.setPosition(targetWrist);
            rotator.setTargetPosition(targetRotator);
            if ((Math.abs(flipper.getCurrentPosition() - targetFlip) < 50 && this.mode == Mode.HOME)) {
                flipper.setPower(0);
            } else {
                flipper.setPower(1);
            }
        }
        updateLED();
    }
}