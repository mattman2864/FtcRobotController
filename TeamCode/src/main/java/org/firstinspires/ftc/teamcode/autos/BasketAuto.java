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
    private final Pose lift = new Pose(5, 5, 5.75);
    private final Pose outtake = new Pose(6, 18, 5.5);
    private final Pose firstPickup = new Pose(18, 12, 3);
    private final Pose secondPickup = new Pose(18, 20, 3.07);
    private final Pose parkOnBar = new Pose(36.75, -9.23, 1.457);
    private Timer pathTimer, actionTimer, opmodeTimer;
    int pathState = 0;
    Follower follower;
    Path alignLift;
    Path dropoff1;
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
//            case 1:
//                if (pathTimer.getElapsedTime() > 5000) {
//                    follower.followPath(dropoff1);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (calculatePoseError(follower.getPose(), outtake) < 1) {
//                    robotLift.intake(-1);
//                    pathTimer.resetTimer();
//                    setPathState(3);
//                }
//                break;
//            case 4:
//                if (pathTimer.getElapsedTime() > 1000) {
//                    robotLift.setMode(Lift.Mode.REAR_PICKUP_DROP);
//                    follower.followPath(pickup1);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (calculatePoseError(follower.getPose(), firstPickup) < 1 && pathTimer.getElapsedTime() > 5000) {
//                    setPathState(-1);
//                }
//                break;

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
