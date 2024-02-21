package org.firstinspires.ftc.teamcode;
// For hardwareMap and telemetry
import android.util.Size;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
// For Camera
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;

//Other Java Imports
import java.util.ArrayList;
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
    public void manualDown () {
        liftState = "manual";
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        checkForZero();
        if (lift.getCurrentPosition() > 500 && Objects.equals(holdState, "outhold")) {
            lift.setPower(-1);
        } else if (lift.getCurrentPosition() > 10 && Objects.equals(holdState, "inhold")){
            lift.setPower(-1);
        } else {
            setPosition(lift.getCurrentPosition());
        }
    }
    public void manualHold () {
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);

    }
    public void prePosUp () {
        if (!(Objects.equals(liftState, "moving") || Objects.equals(liftState, "position"))) {
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
        double speed = 1; // 0-1
        front_left.setPower((y - x - r) * speed);
        front_right.setPower((y + x + r) * speed);
        rear_left.setPower((y + x - r) * speed);
        rear_right.setPower((y - x + r) * speed);
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
        for (int i = 1; i < 1000; i++) {
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
            }
        }
        return 0;
    }
}

class ExtraCommands {

    HardwareMap hardwareMap;
    SampleMecanumDrive drive;

    public ExtraCommands (HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        this.drive = new SampleMecanumDrive(hardwareMap);

    }

    public Pose2d setStart() {
        return new Pose2d();
    }

    public void moveRight() {

        Trajectory moveRight = this.drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        drive.followTrajectory(moveRight);

    }

    public void moveLeft() {

        Trajectory moveLeft = this.drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(10)
                .build();

        drive.followTrajectory(moveLeft);
    }

}

class Displayer {

    ArrayList<ArrayList<String>> display;

    public Displayer() {
        display = new ArrayList<ArrayList<String>>();
    }

    public void add(ArrayList<String> info) {
        this.display.add(info);
    }

}

class AprilTagDetector {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private HardwareMap hardwareMap;

    public AprilTagDetector (HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        initAprilTag();

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    private List<AprilTagDetection> detectAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        return currentDetections;

    }   // end method telemetryAprilTag()

}