package org.firstinspires.ftc.teamcode;
// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
// For Camera
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
// For Vision
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//Other Java Imports
import java.util.List;
import java.util.Objects;
class Intake {
    DcMotor lower_infeed;
    DcMotor upper_infeed;
    DistanceSensor dist;
    public Intake(HardwareMap hardwareMap) {
        // Assigning motors
        lower_infeed = hardwareMap.get(DcMotor.class, "lower_infeed");
        upper_infeed = hardwareMap.get(DcMotor.class, "upper_infeed");
        lower_infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        dist = hardwareMap.get(DistanceSensor.class, "dist");
    }

    public void on(double speed) {
        lower_infeed.setPower(speed);
        upper_infeed.setPower(speed);
    }

    public void reverse(double speed) {
        lower_infeed.setPower(-speed);
        upper_infeed.setPower(-speed);
    }
    public void off() {
        lower_infeed.setPower(0);
        upper_infeed.setPower(0);
    }
    public boolean isFull () {
        return dist.getDistance(DistanceUnit.MM) < 60;
    }
}
class Lift {
    DcMotor lift;
    TouchSensor liftTouch;
    Servo flip;
    Servo release;
    final int maxHeight = 4300;
    public String liftState;
    public String holdState;
    // Preset Positions
    final int pos1 = 2100;
    final int pos2 = 3200;
    final int pos3 = maxHeight;
    int prePos = 0;
    public Lift (HardwareMap hardwareMap) {
        // Initialize lift motor
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        liftState = "down";

        // Initialize touch sensor
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");

        // Initialize release servo
        release = hardwareMap.get(Servo.class, "release");
        release.setPosition(0);

        // Initialize flip servo
        flip = hardwareMap.get(Servo.class, "flip");
        flip.setPosition(0.0);
        holdState = "inhold";

    }
    public void setPosition (int position) {
        position = Math.max(Math.min(position, maxHeight), 0);
        lift.setPower(1);

        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftState = "moving";
    }
    public void update (ElapsedTime time) {
        checkForZero();
        switch (liftState) {
            case "moving":
                if (Math.abs(lift.getCurrentPosition()-lift.getTargetPosition()) < 1000 && lift.getCurrentPosition() > 1500) {
                    liftState = "position";
                    open(false);
                }
                break;
            case "down":
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lift.setPower(0);
                close();
                break;
        }
        if (holdState.equals("outdrop")) {
            if (time.milliseconds() > 70) {
                release.setPosition(0);
                holdState = "outhold";
            }
        }
    }
    public void open(boolean extra) {
        flip.setPosition(0.65);
        holdState = "outhold";
    }
    public void close() {
        release.setPosition(0);
        flip.setPosition(0.0);
        holdState = "inhold";
    }
    private void checkForZero () {
        if (lift.getTargetPosition() == 0 && (liftTouch.isPressed() || lift.getCurrentPosition() < 30)) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftState = "down";
            holdState = "inhold";
        }
    }
    public void drop () {
        if (!Objects.equals(holdState, "outhold")) {
            open(false);
        }
        release.setPosition(0.5);
        holdState = "outdrop";
    }
    public void place () {
        setPosition(pos1);
        prePos = 1;
    }
    public void down () {
        if (lift.getCurrentPosition() < 2000 && Objects.equals(liftState, "outhold")) {
            return;
        }
        holdState = "inhold";
        setPosition(0);
        liftState = "moving";
        close();
    }
    public boolean isUp() {
        return !Objects.equals(liftState, "down");
    }
    public boolean isAtPosition () {
        return Objects.equals(liftState, "position");
    }
    public boolean isManual () {
        return Objects.equals(liftState, "manual");
    }
    public void manualUp () {
        liftState = "manual";
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (lift.getCurrentPosition() < maxHeight-100){
            lift.setPower(1);
        } else {
            manualHold();
        }
    }
    public void manualDown (boolean override) {
        liftState = "manual";
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        checkForZero();
        if (lift.getCurrentPosition() > 500 && Objects.equals(holdState, "outhold")) {
            lift.setPower(-1);
        } else if (lift.getCurrentPosition() > 10 && Objects.equals(holdState, "inhold")){
            lift.setPower(-1);
        } else if (override && !(liftTouch.isPressed())) {
            lift.setPower(-0.5);
        }else {
            setPosition(lift.getCurrentPosition());
        }
    }
    public void manualHold () {
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

    }
    public void prePosUp () {
        if ((Objects.equals(liftState, "manual"))) {
            if (lift.getCurrentPosition() > 3200) {
                prePos = 2;
            } else {
                prePos = 1;
            }
        }
        switch (prePos) {
            case 1:
                prePos = 2;
                setPosition(pos2);
                break;
            case 2:
                prePos = 3;
                setPosition(pos3);
                break;
        }
    }
    public void prePosDown () {
        if (Objects.equals(liftState, "manual")) {
            if (lift.getCurrentPosition() < 3200) {
                prePos = 3;
            } else {
                prePos = 2;
            }
        }
        switch (prePos) {
            case 3:
                prePos = 2;
                setPosition(pos2);
                break;
            case 2:
                prePos = 1;
                setPosition(pos1);
                break;
        }
    }
    public boolean isOut () {
        return Objects.equals(holdState, "outhold") || Objects.equals(holdState, "outdrop");
    }
}
class Launcher {
    Servo launch;
    public Launcher (HardwareMap hardwareMap) {
        launch = hardwareMap.get(Servo.class, "launcher");
        launch.setPosition(.17);
    }
    public void shoot () {
        launch.setPosition(.3);
    }
}
class Drive {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor rear_left;
    DcMotor rear_right;
    public Drive (HardwareMap hardwareMap) {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        rear_left = hardwareMap.get(DcMotor.class, "rear_left");
        rear_right = hardwareMap.get(DcMotor.class, "rear_right");
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void drive (double x, double y, double r) {
        double speed = 0.7; // 0-1
        front_left.setPower((y - x - r) * speed);
        front_right.setPower((y + x + r) * speed);
        rear_left.setPower((y + x - r) * speed);
        rear_right.setPower((y - x + r) * speed);
    }
}
