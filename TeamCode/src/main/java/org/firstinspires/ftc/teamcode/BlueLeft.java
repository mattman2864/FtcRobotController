package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.ObjectDetector;
import org.firstinspires.ftc.teamcode.RRDrive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.drive.opmode.StrafeTest;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BLUE_LEFT", preselectTeleOp = "MainTeleOp")
public class BlueLeft extends LinearOpMode {
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff


        // Initialize stuff
        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, "BlueModel.tflite");
        objectDetector.initTfod();

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
                .waitSeconds(0) // ADD WAIT HERE FOR ICE (IN SECONDS)
                .lineToConstantHeading(new Vector2d(10, -1))
                .addTemporalMarker(() -> {
                    lift.setPosition(1300);
                })
                .splineToLinearHeading(new Pose2d(10, 25, Math.toRadians(90)), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    lift.open(false);
                })
                .splineToConstantHeading(new Vector2d(23, 38), Math.toRadians(90))
                .addTemporalMarker(() -> {
                    lift.setPosition(1000);
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
        TrajectorySequence toCenterLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(30, -4, 0))
                .lineToLinearHeading(new Pose2d(27.2, -1, 0))
                .build();

        // from center line to board
        TrajectorySequence centerToBoard = drive.trajectorySequenceBuilder(toCenterLine.end())
                .lineToLinearHeading(new Pose2d(15, -1, 0))
                .splineToLinearHeading(new Pose2d(20, 25, Math.toRadians(90)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(26, 38, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    lift.setPosition(1180);
                })
                .addTemporalMarker(2, () -> {
                    lift.open(false);
                })
                .build();

        TrajectorySequence toRightLine = drive.trajectorySequenceBuilder(cameraLineup.end())
                .lineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(24, -6, Math.toRadians(-70)))
                .build();

        TrajectorySequence rightToBoard = drive.trajectorySequenceBuilder(toRightLine.end())
                .lineToLinearHeading(new Pose2d(15, -1, 0))
                .splineToLinearHeading(new Pose2d(20, 25, Math.toRadians(90)), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(35, 38, Math.toRadians(90)))
                .addTemporalMarker(1, () -> {
                    lift.setPosition(1180);
                })
                .addTemporalMarker(2, () -> {
                    lift.open(false);
                })
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

        // Do stuff
        // Do stuff
        drive.followTrajectory(cameraLineup);
        sleep(1000);
        int side = objectDetector.get_position();
        telemetry.addData("side", side);
        telemetry.update();
        double intakeSpeed = 0.3;
        int intakeTimeMS = 2000;
        switch (side) {
            case 0:
                // Left
                drive.followTrajectorySequence(left);
                drive.followTrajectorySequence(park);
                break;
            case 1:
                // Middle
                drive.followTrajectorySequence(toCenterLine);
                intake.reverse(intakeSpeed);
                sleep(intakeTimeMS);
                intake.off();
//                sleep(7000);
                drive.followTrajectorySequence(centerToBoard);
                break;
            case 2:
                // Right
                drive.followTrajectorySequence(toRightLine);
                intake.reverse(intakeSpeed);
                sleep(intakeTimeMS);
                intake.off();
//                sleep(7000);
                drive.followTrajectorySequence(rightToBoard);
                break;

        }
    }
}