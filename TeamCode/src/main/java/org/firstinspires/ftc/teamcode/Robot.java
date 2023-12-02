package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// Controller
// For Camera
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
// For Vision

class Intake {
    DcMotor lower_infeed;
    DcMotor upper_infeed;
    public Intake(HardwareMap hardwareMap) {
        // Assigning motors
        lower_infeed = hardwareMap.get(DcMotor.class, "lower_infeed");
        upper_infeed = hardwareMap.get(DcMotor.class, "upper_infeed");
        lower_infeed.setDirection(DcMotorSimple.Direction.REVERSE);
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
}

class Lift {
    DcMotor lift;
    double power;
    final int maxHeight;
    public boolean manual;
    public Lift (HardwareMap hardwareMap) {
        // Initializing lift motor and power
        lift = hardwareMap.get(DcMotor.class, "lift");
        power = 1;
        maxHeight = 4300;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manual = false;
        off();
    }
    void off() {
        lift.setPower(0);
    }
    void goToTop () {
        manual = false;
        setPosition(maxHeight);
    }
    void setPosition(int encoderPosition) {
        lift.setPower(power);
        lift.setTargetPosition(encoderPosition);
    }
    boolean isAtBottom () {
        return lift.getTargetPosition() == 0;
    }
    void checkForZero() {
        if (lift.getTargetPosition() == 0 && Math.abs(lift.getCurrentPosition()) < 10) {
            off();
        }
    }
    void slowMove (int iterate) {
        if (lift.getCurrentPosition() < maxHeight - iterate) {
            setPosition(lift.getCurrentPosition() + iterate);
        } else {
            setPosition(maxHeight);
        }
    }
    int getPosition() {
        return lift.getCurrentPosition();
    }
}

class Launcher {
    DcMotor launch;
    public Launcher (HardwareMap hardwareMap) {
        launch = hardwareMap.get(DcMotor.class, "launch");
    }
    public void shoot (double speed) {
        launch.setPower(speed);
    }
    public void off (double speed) {
        launch.setPower(0);
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
        double speed = 0.5; // 0-1
        front_left.setPower((y - x - r) * speed);
        front_right.setPower((y + x + r) * speed);
        rear_left.setPower((y + x - r) * speed);
        rear_right.setPower((y - x + r) * speed);
    }
}

class FlipGrip {
    Servo flip;
    Servo grip;
    public boolean flipped;
    public boolean gripped;
    public FlipGrip (HardwareMap hardwareMap) {
        flip = hardwareMap.get(Servo.class, "flip");
        grip = hardwareMap.get(Servo.class, "release");
        flip.setPosition(0.6);
        grip.setPosition(0);
        flipped = false;
        gripped = false;
    }
    void flip () {
        if (flipped) {
            flip.setPosition(0.54);
        }
        else {
            flip.setPosition(0.05);
        }
        flipped = !flipped;
    }
    void grip () {
        if (gripped) {
            grip.setPosition(0);
        }
        else {
            grip.setPosition(0.5);
        }
        gripped = !gripped;
    }
    public boolean isFlipped () {
        return flipped;
    }
    public boolean isGripped() {
        return gripped;
    }
}