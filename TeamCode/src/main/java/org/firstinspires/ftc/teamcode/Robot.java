package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// Controller
// For Camera
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
// For Vision
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//Other Java Imports
import java.util.ArrayList;
import java.util.List;
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
    TouchSensor liftTouch;
    double power;
    final int maxHeight;
    public boolean manual;
    public Lift (HardwareMap hardwareMap) {
        // Initializing lift motor and power
        lift = hardwareMap.get(DcMotor.class, "lift");
        liftTouch = hardwareMap.get(TouchSensor.class, "liftTouch");
        power = 1;
        maxHeight = 4300;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        manual = false;
        off();
    }
    void off() {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setPower(0);
    }
    void goToTop () {
        manual = false;
        setPosition(maxHeight);
    }
    void setPosition(int encoderPosition) {
        lift.setPower(power);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(encoderPosition);
    }
    boolean isAtBottom () {
        return lift.getTargetPosition() == 0 || Math.abs(lift.getCurrentPosition()) < 10 ;
    }
    void checkForZero() {
        if ((lift.getTargetPosition() == 0 && Math.abs(lift.getCurrentPosition()) < 10) || liftTouch.isPressed()) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            off();
        }
    }
    void slowMove (boolean up) {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (lift.getCurrentPosition() < maxHeight-50 && up) {
            lift.setPower(power);
        } else if (lift.getCurrentPosition() > 50 && !up) {
            lift.setPower(-power);
        } else {
            lift.setPower(0);
        }
    }
    int getPosition() {
        return lift.getCurrentPosition();
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

class FlipGrip {
    Servo flip;
    Servo grip;
    public boolean flipped;
    public boolean gripped;
    public FlipGrip (HardwareMap hardwareMap) {
        flip = hardwareMap.get(Servo.class, "flip");
        grip = hardwareMap.get(Servo.class, "release");
        flip.setPosition(0.59);
        grip.setPosition(0);
        flipped = false;
        gripped = false;
    }
    void flip (boolean extra) {
        if (flipped) {
            flip.setPosition(0.59);
        }
        else {
            if (extra) {
                flip.setPosition(0.15);
            } else {
                flip.setPosition(0.25);
            }
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

class ObjectDetector {

    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    VisionPortal myVisionPortal;
    HardwareMap hardwareMap;
    String modelName;
    public ObjectDetector (HardwareMap map, String name) {
        this.hardwareMap = map;
        modelName = name;
    }
    public void initTfod() {
        USE_WEBCAM = true;
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName(modelName);
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("RedTP", "Object"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public int get_position() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        // Iterate through list and call a function to display info for each recognized object.
        float xFinal = 0;
        float confidenceThreshold = 0.6f;
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            if (myTfodRecognition_item.getConfidence() > confidenceThreshold) {
                xFinal = x;
            }
        }
        if (xFinal > 300) {
            return 2;
        } else if (myTfodRecognitions.size() > 0) {
            return 1;
        } else {
            return 0;
        }
    }
}