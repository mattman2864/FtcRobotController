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
    private final Pose lift = new Pose(8, 11, Math.toRadians(-45));
    private final Pose outtake = new Pose(5, 19, Math.toRadians(-45));
    private final Pose firstPickup = new Pose(21, 17, Math.toRadians(180));
    private final Pose secondPickup = new Pose(21, 25, Math.toRadians(180));
    private final Pose parkOnBar = new Pose(38.9, -9, Math.toRadians(90));
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
                new BezierLine(new Point(outtake), new Point(firstPickup))
        );
        pickup1.setLinearHeadingInterpolation(outtake.getHeading(), firstPickup.getHeading());

        dropoff2 = new Path(
                new BezierLine(new Point(firstPickup), new Point(outtake))
        );
        dropoff2.setLinearHeadingInterpolation(firstPickup.getHeading(), outtake.getHeading());

        pickup2 = new Path(
                new BezierLine(new Point(outtake), new Point(secondPickup))
        );
        pickup2.setLinearHeadingInterpolation(outtake.getHeading(), secondPickup.getHeading());

        dropoff3 = new Path(
                new BezierLine(new Point(secondPickup), new Point(outtake))
        );
        dropoff3.setLinearHeadingInterpolation(secondPickup.getHeading(), outtake.getHeading());

        park = new Path(
                new BezierLine(new Point(outtake), new Point(parkOnBar))
        );
        park.setLinearHeadingInterpolation(outtake.getHeading(), parkOnBar.getHeading());
    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(alignLift);
                setPathState(1);
                robotLift.setMode(Lift.Mode.HIGH_BASKET);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(dropoff1);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 2000) {
                    robotLift.intake(-1);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(backup);
                    robotLift.intake(0);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 1000) {
                    robotLift.setMode(Lift.Mode.HOME);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 2000) {
                    robotLift.setMode(Lift.Mode.REAR_PICKUP_DROP);
                    robotLift.intake(1);
                    follower.followPath(pickup1);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 2000) {
                    robotLift.intake(0);
                    robotLift.setMode(Lift.Mode.HOME);
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