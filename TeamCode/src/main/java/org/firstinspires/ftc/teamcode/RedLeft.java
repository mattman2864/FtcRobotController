package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RED_LEFT", preselectTeleOp = "MainTeleOp")
public class RedLeft extends LinearOpMode {
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff


        // Initialize stuff
        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, "RedModel.tflite");
        objectDetector.initTfod();

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
        TrajectorySequence toLeftLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, 8, Math.toRadians(0)))
                .build();

        // from left line to board
        TrajectorySequence leftToBoard = drive.trajectorySequenceBuilder(toLeftLine.end())
                .lineToLinearHeading(new Pose2d(15, -5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(28.5, -95))
                .addTemporalMarker(1, () -> {
                    lift.setPosition(1180);
                })
                .addTemporalMarker(2, () -> {
                    lift.open(false);
                })
                .build();

        // Line up to center line
        TrajectorySequence toCenterLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(29, -4, 0))
                .lineToLinearHeading(new Pose2d(27.2, -4, 0))
                .build();

        // from center line to board
        TrajectorySequence centerToBoard = drive.trajectorySequenceBuilder(toCenterLine.end())
                .lineToLinearHeading(new Pose2d(15, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(26, -95))
                .addTemporalMarker(1, () -> {
                    lift.setPosition(1180);
                })
                .addTemporalMarker(2, () -> {
                    lift.open(false);
                })
                .build();

        TrajectorySequence toRightLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(24, -9, Math.toRadians(-40)))
                .build();

        TrajectorySequence rightToBoard = drive.trajectorySequenceBuilder(toRightLine.end())
                .lineToLinearHeading(new Pose2d(15, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, 10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(51, -70, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(16, -94))
                .addTemporalMarker(1, () -> {
                    lift.setPosition(1180);
                })
                .addTemporalMarker(2, () -> {
                    lift.open(false);
                })
                .build();

        //park after placing pixel
        TrajectorySequence park = drive.trajectorySequenceBuilder(leftToBoard.end())
                .lineToConstantHeading(new Vector2d(40, -85))
                .build();


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        // Do stuff
        drive.followTrajectory(cameraLineup);
        sleep(1000);
        int side = objectDetector.get_position();
        telemetry.addData("side", side);
        telemetry.update();
        double intakeSpeed = 0.3;
        int intakeTimeMS = 2000;
//        objectDetector.enable(false);
        switch (side) {
            case 0:
                // Left
                drive.followTrajectorySequence(toLeftLine);
                intake.reverse(intakeSpeed);
                sleep(intakeTimeMS);
                intake.off();
                drive.followTrajectorySequence(leftToBoard);
                break;
            case 1:
                // Middle
                drive.followTrajectorySequence(toCenterLine);
                intake.reverse(intakeSpeed);
                sleep(intakeTimeMS);
                intake.off();
                drive.followTrajectorySequence(centerToBoard);
                break;
            case 2:
                // Right
                drive.followTrajectorySequence(toRightLine);
                intake.reverse(intakeSpeed);
                sleep(intakeTimeMS);
                intake.off();
                drive.followTrajectorySequence(rightToBoard);
                break;

        }
        lift.setPosition(1000);
        sleep(500);
        lift.drop();
        sleep(200);
        lift.setPosition(2200);
        sleep(500);
        lift.open(false);
        sleep(1000);
        drive.followTrajectorySequence(park);
        lift.down();
        sleep(2000);
    }
}