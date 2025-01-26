package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Basket")
public class BasketAuto extends OpMode {
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose lift = new Pose(10, 11, Math.toRadians(-45));
    private final Pose outtake = new Pose(10, 23.5, Math.toRadians(-45));
    private final Pose outtake2 = new Pose(10.5, 22, Math.toRadians(-45));
    private final Pose outtake3 = new Pose(10.5, 22, Math.toRadians(-45));
    private final Pose firstPickup = new Pose(28, 15, Math.toRadians(180));
    private final Pose secondPickup = new Pose(28, 24, Math.toRadians(180));
    private final Pose parkLineup = new Pose(53, 15, Math.toRadians(-90));
    private final Pose parkOnBar = new Pose(53, 0, Math.toRadians(-90));
    private Timer pathTimer, actionTimer, opmodeTimer;
    int pathState = 0;
    Follower follower;
    Path alignLift;
    Path dropoff1;
    Path backup;
    Path pickup1;
    Path dropoff2;
    Path pickup2;
    Path dropoff3;
    Path prepark;
    Path park;
    Lift robotLift;
    public void buildPaths() {
        alignLift = new Path(
                new BezierLine(new Point(startPose), new Point(lift))
        );
        alignLift.setLinearHeadingInterpolation(startPose.getHeading(), lift.getHeading());

        dropoff1 = new Path(
                new BezierLine(new Point(lift), new Point(outtake))
        );
        dropoff1.setLinearHeadingInterpolation(lift.getHeading(), outtake.getHeading());

        backup = new Path(
                new BezierLine(new Point(outtake), new Point(lift))
        );
        backup.setLinearHeadingInterpolation(outtake.getHeading(), lift.getHeading());

        pickup1 = new Path(
                new BezierLine(new Point(lift), new Point(firstPickup))
        );
        pickup1.setLinearHeadingInterpolation(lift.getHeading(), firstPickup.getHeading());

        dropoff2 = new Path(
                new BezierLine(new Point(firstPickup), new Point(outtake))
        );
        dropoff2.setLinearHeadingInterpolation(firstPickup.getHeading(), outtake.getHeading());

        pickup2 = new Path(
                new BezierLine(new Point(lift), new Point(secondPickup))
        );
        pickup2.setLinearHeadingInterpolation(lift.getHeading(), secondPickup.getHeading());

        dropoff3 = new Path(
                new BezierLine(new Point(secondPickup), new Point(outtake))
        );
        dropoff3.setLinearHeadingInterpolation(secondPickup.getHeading(), outtake.getHeading());

        prepark = new Path(
                new BezierLine(new Point(lift), new Point(parkLineup))
        );
        prepark.setLinearHeadingInterpolation(lift.getHeading(), parkLineup.getHeading());

        park = new Path(
                new BezierLine(new Point(parkLineup), new Point(parkOnBar))
        );
        park.setLinearHeadingInterpolation(parkLineup.getHeading(), parkOnBar.getHeading());
    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(alignLift);
                setPathState(1);
                robotLift.setMode(Lift.Mode.HIGH_BASKET);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(dropoff1);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 1500) {
                    robotLift.intake(-1);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(backup);
                    robotLift.intake(0);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 500) {
                    robotLift.setMode(Lift.Mode.REAR_PICKUP);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 1000) {
                    robotLift.intake(1);
                    follower.followPath(pickup1);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 1500) {
                    robotLift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 700) {
                    robotLift.setMode(Lift.Mode.HIGH_BASKET);
                    robotLift.intake(0);
                    setPathState(8);
                }
            case 8:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(dropoff2);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > 3000) {
                    robotLift.intake(-1);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(backup);
                    robotLift.intake(0);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 800) {
                    robotLift.setMode(Lift.Mode.REAR_PICKUP);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(pickup2);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 1500) {
                    robotLift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    robotLift.intake(1);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 700) {
                    robotLift.setMode(Lift.Mode.HIGH_BASKET);
                    robotLift.intake(0);
                    follower.followPath(dropoff3);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 3000) {
                    robotLift.intake(-1);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 500) {
                    robotLift.intake(0);
                    follower.followPath(backup);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTime() > 500) {
                    robotLift.setMode(Lift.Mode.PARK);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(prepark);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    public double calculatePoseError(Pose position, Pose target) {
        return Math.abs((target.getX() - position.getX()) + (target.getY() - position.getY())
                + 2 * (target.getHeading() - position.getHeading()));
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robotLift = new Lift(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        follower.update();
        autoStateUpdate();
        robotLift.update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}