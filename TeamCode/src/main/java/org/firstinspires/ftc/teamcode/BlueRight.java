package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name="BLUE_RIGHT", preselectTeleOp = "MainTeleOp")
public class BlueRight extends LinearOpMode {
    public void runOpMode() {
        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff


        // Initialize stuff

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FlipGrip flipgrip = new FlipGrip(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        // TODO: add launcher
        Lift lift = new Lift(hardwareMap);


        // move for camera visualization
        Trajectory cameraLineup = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 4, 0))
                .build();

        // Line up to left line
        TrajectorySequence toLeftLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(30, 4, Math.toRadians(90)))
                .build();

        // from left line to board
        TrajectorySequence leftToBoard = drive.trajectorySequenceBuilder(toLeftLine.end())
                .lineToConstantHeading(new Vector2d(51, -8))
                .lineToConstantHeading(new Vector2d(51, 70))
                .lineToConstantHeading(new Vector2d(23.5, 88))
                .build();

        // Line up to center line
        TrajectorySequence toCenterLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(27.2, -2, 0))
                .build();

        // from center line to board
        TrajectorySequence centerToBoard = drive.trajectorySequenceBuilder(toCenterLine.end())
                .lineToLinearHeading(new Pose2d(20, -16, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, -16, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(51, 60))
                .lineToConstantHeading(new Vector2d(32, 88))
                .build();

        TrajectorySequence toRightLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, -7, Math.toRadians(0)))
                .build();

        TrajectorySequence rightToBoard = drive.trajectorySequenceBuilder(toRightLine.end())
                .lineToLinearHeading(new Pose2d(20, -21, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(52, -21, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(51, 60))
                .lineToConstantHeading(new Vector2d(35, 88))
                .build();

        //park after placing pixel
        TrajectorySequence park = drive.trajectorySequenceBuilder(leftToBoard.end())
                .lineToConstantHeading(new Vector2d(4, 80))
                .lineTo(new Vector2d(4, 92))
                .build();


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        // Do stuff
        drive.followTrajectory(cameraLineup);
        //TODO: Camera detects which side
        int side = 2;
        switch (side) {
            case 0:
                // Left
                drive.followTrajectorySequence(toLeftLine);
                intake.reverse(0.4);
                sleep(1500);
                intake.off();
                drive.followTrajectorySequence(leftToBoard);
                break;
            case 1:
                // Middle
                drive.followTrajectorySequence(toCenterLine);
                intake.reverse(0.4);
                sleep(1000);
                intake.off();
                drive.followTrajectorySequence(centerToBoard);
                break;
            case 2:
                // Right
                drive.followTrajectorySequence(toRightLine);
                intake.reverse(0.4);
                sleep(1000);
                intake.off();
                drive.followTrajectorySequence(rightToBoard);
                break;

        }
        lift.setPosition(1180);
        sleep(700);
        flipgrip.flip(true);
        sleep(1000);
        lift.setPosition(950);
        sleep(500);
        flipgrip.grip();
        sleep(1000);
        flipgrip.grip();
        sleep(100);
        lift.setPosition(2000);
        sleep(300);
        flipgrip.flip(false);
        sleep(1000);
        lift.setPosition(0);
        sleep(1500);
        lift.checkForZero();
        drive.followTrajectorySequence(park);
    }
}
