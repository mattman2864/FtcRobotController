package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RED_LEFT", preselectTeleOp = "MainTeleOp")
public class RedLeft extends LinearOpMode {
    OpenCvWebcam webcam;
    static final int liftLowerHeight = 800;
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff


        // Initialize stuff
        // Initialize EOCV
        int CameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), CameraMonitorViewId);
        telemetry.addLine("Yay!");
        telemetry.update();
        sleep(2000);
        ObjectDetector detector = new ObjectDetector(telemetry, true);
        webcam.setPipeline(detector);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("error! :(");
                telemetry.update();
            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        FlipGrip flipgrip = new FlipGrip(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);
//        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, "RedModel.tflite");


        // move for camera visualization
        Trajectory cameraLineup = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, -4, 0))
                .build();

        // Line up to left line
        TrajectorySequence left = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, 8, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker(intake::off)
                .lineToLinearHeading(new Pose2d(15, -5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -60, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .lineToConstantHeading(new Vector2d(28.5, -95))
                .addTemporalMarker(() -> {
                    lift.setPosition(liftLowerHeight);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(lift::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    lift.setPosition(2200);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(lift::close)
                .build();

        // center
        TrajectorySequence center = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(29, -4, 0))
                .lineToLinearHeading(new Pose2d(27.2, -4, 0))
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker(intake::off)
                .lineToLinearHeading(new Pose2d(15, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -60, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .lineToConstantHeading(new Vector2d(26, -95))
                .addTemporalMarker(() -> {
                    lift.setPosition(liftLowerHeight);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(lift::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    lift.setPosition(2200);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(lift::close)
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(24, -9, Math.toRadians(-40)))
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker(intake::off)
                .lineToLinearHeading(new Pose2d(15, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(51, -50, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .lineToConstantHeading(new Vector2d(23, -94))
                .addTemporalMarker(() -> {
                    lift.setPosition(liftLowerHeight);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(lift::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    lift.setPosition(2200);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(lift::close)
                .build();

        //park after placing pixel
        TrajectorySequence park = drive.trajectorySequenceBuilder(left.end())
                .addDisplacementMarker(lift::down)
                .lineToConstantHeading(new Vector2d(55, -85))
                .build();


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        ObjectDetector.Location loc = detector.getLocation();

        // Do stuff
        drive.followTrajectory(cameraLineup);
        sleep(1000);
        telemetry.update();
        double intakeSpeed = 0.3;
        int intakeTimeMS = 2000;
//        objectDetector.enable(false);
        switch (loc) {
            case NOT_FOUND:
                // left
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(park);
                break;
            case CENTER:
                // Center
                drive.followTrajectorySequence(center);
                drive.followTrajectorySequence(park);
                break;
            case RIGHT:
                // Right
                drive.followTrajectorySequence(right);
                drive.followTrajectorySequence(park);
                break;
        }
    }
}