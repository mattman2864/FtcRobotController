package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// Controller
// For Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
// For Vision
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

//Other Java Imports
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
    double power;
    final int maxHeight;
    public Lift (HardwareMap hardwareMap) {
        // Initializing lift motor and power
        lift = hardwareMap.get(DcMotor.class, "lift");
        power = 1;
        maxHeight = 4300;
//         Resetting Encoder of lift motor and setting it to "RUN_TO_POSITION" mode
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        off();
    }
    void off() {
        lift.setPower(0);
    }
    void goToTop () {
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
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        rear_right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void drive (double x, double y, double r) {
        double speed = 0.5; // 0-1
        front_left.setPower((y + x + r) * speed);
        front_right.setPower((y - x - r) * speed);
        rear_left.setPower((y + x - r) * speed);
        rear_right.setPower((y - x + r) * speed);
    }
}

class FlipGrip {
    Servo flip;
    Servo grip;
    boolean flipped;
    boolean gripped;
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
    void open() {
        flip.setPosition(0.05);
    }
    void close() {
        flip.setPosition(0.54);
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
}

class ObjectDetector {

    private TfodProcessor tfod;

    private VisionPortal visionPortal;
    public ObjectDetector(HardwareMap hardwareMap, String modelFile) {

        initTfod(modelFile);

    }
    private void initTfod(String modelFile) {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(modelFile)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    public String detectLocation() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        int maxConfidence = 0;


        for (Recognition recognition : currentRecognitions) {
            /**
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
             */

            if (recognition.getConfidence() >= maxConfidence) {
                String label = recognition.getLabel();
            }

        }

        return label;
        // end for() loop
    }
}