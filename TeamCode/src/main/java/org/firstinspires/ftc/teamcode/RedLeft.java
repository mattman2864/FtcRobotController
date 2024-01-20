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
                .lineToLinearHeading(new Pose2d(51, -60, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(32, -93))
                .build();

        // Line up to center line
        TrajectorySequence toCenterLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(27.2, 2, 0))
                .build();

        // from center line to board
        TrajectorySequence centerToBoard = drive.trajectorySequenceBuilder(toCenterLine.end())
                .lineToLinearHeading(new Pose2d(20, 16, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, 16, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(51, -60))
                .lineToConstantHeading(new Vector2d(33.5, -88))
                .build();

        TrajectorySequence toRightLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(30, -4, Math.toRadians(-90)))
                .build();

        TrajectorySequence rightToBoard = drive.trajectorySequenceBuilder(toRightLine.end())
                .lineToConstantHeading(new Vector2d(53, 8))
                .lineToConstantHeading(new Vector2d(53, -70))
                .lineToConstantHeading(new Vector2d(27, -88))
                .build();

        //park after placing pixel
        TrajectorySequence park = drive.trajectorySequenceBuilder(leftToBoard.end())
                .lineToConstantHeading(new Vector2d(4, -80))
                .lineTo(new Vector2d(4, -92))
                .build();


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        // Do stuff
        drive.followTrajectory(cameraLineup);
        sleep(1000);
        int side = 2;//objectDetector.get_position();
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
        lift.setPosition(1180);
        sleep(700);
//        flipgrip.flip(true);
        lift.open();
        sleep(1000);
        lift.setPosition(1000);
        sleep(500);
//        flipgrip.grip();
        lift.drop();
        sleep(200);
        lift.setPosition(2200);
        sleep(500);
        lift.open();
        sleep(1000);
        lift.down();
        sleep(1500);
        drive.followTrajectorySequence(park);




    }
}