package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RRDrive.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RRDrive.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.drive.opmode.StrafeTest;
import org.firstinspires.ftc.teamcode.RRDrive.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Camera-Test", preselectTeleOp = "MainTeleOp")
public class CameraTest extends LinearOpMode {
    public void runOpMode() {
        ObjectDetector objectDetector = new ObjectDetector(hardwareMap, "RedTest.tflite");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("x Position", objectDetector.detectLocation());
            telemetry.update();
        }
    }
}