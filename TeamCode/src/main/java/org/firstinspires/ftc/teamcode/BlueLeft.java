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

@Autonomous(name = "BLUE_LEFT", preselectTeleOp = "MainTeleOp")
public class BlueLeft extends LinearOpMode {
    OpenCvWebcam webcam;
    static final int liftLowerHeight = 800;
    static final double ICE_WAIT = 0; // CHANGE THIS FOR ICE
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
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);




        // move for camera visualization
        Trajectory cameraLineup = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, -4, 0))
                .build();

        // Line up to left line
        TrajectorySequence left = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineTo(new Vector2d(17, 7))
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker( intake::off)
                .waitSeconds(ICE_WAIT) // ADD WAIT HERE FOR ICE (IN SECONDS)
                .lineToConstantHeading(new Vector2d(10, -1))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .splineToLinearHeading(new Pose2d(10, 25, Math.toRadians(90)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .splineToConstantHeading(new Vector2d(21, 38), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    lift.setPosition(liftLowerHeight);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(lift::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    lift.setPosition(2200);
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(lift::close)
                .build();

        // Line up to center line
        TrajectorySequence center = drive.trajectorySequenceBuilder(cameraLineup.end())
                .splineTo(new Vector2d(32, 4), 0)
                .splineToConstantHeading(new Vector2d(27, -1), 0)
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker( intake::off)
                .waitSeconds(ICE_WAIT) // ADD WAIT HERE FOR ICE (IN SECONDS)
                .lineToConstantHeading(new Vector2d(15, -1))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .splineToLinearHeading(new Pose2d(20, 25, Math.toRadians(90)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .splineToConstantHeading(new Vector2d(28, 38), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    lift.setPosition(liftLowerHeight);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(lift::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    lift.setPosition(2200);
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(lift::close)
                .build();

        // Line up to right line
        TrajectorySequence right = drive.trajectorySequenceBuilder(cameraLineup.end())
                .splineTo(new Vector2d(20, 0), 0)
                .splineToSplineHeading(new Pose2d(23, -8, Math.toRadians(-40)), Math.toRadians(-70))
                .addTemporalMarker(() -> {
                    intake.reverse(0.3);
                })
                .waitSeconds(1)
                .addTemporalMarker(intake::off)
                .waitSeconds(ICE_WAIT) // ADD WAIT HERE FOR ICE (IN SECONDS)
                .lineToConstantHeading(new Vector2d(15, -1))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .splineToLinearHeading(new Pose2d(20, 25, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .splineToConstantHeading(new Vector2d(37, 38), Math.toRadians(90))
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
                .addDisplacementMarker(lift::close)
                .build();

        //park after placing pixel
        TrajectorySequence park = drive.trajectorySequenceBuilder(left.end())
                .lineToLinearHeading(new Pose2d(1, 25, Math.toRadians(90)))
                .addDisplacementMarker(lift::down)
                .lineToLinearHeading(new Pose2d(1, 35, Math.toRadians(90)))
                .build();


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();
        if(isStopRequested()) return;
        ObjectDetector.Location loc = detector.getLocation();

        drive.followTrajectory(cameraLineup);
        sleep(1000);
        telemetry.update();
        double intakeSpeed = 0.3;
        int intakeTimeMS = 2000;
        switch (loc) {
            case NOT_FOUND:
                // Left
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(park);
                break;
            case CENTER:
                // Middle
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