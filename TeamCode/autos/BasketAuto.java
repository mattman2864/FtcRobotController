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
    private final Pose lift = new Pose(15, 9, 5.75);
    private final Pose outtake = new Pose(4.75, 16.9, 5.5);
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
        alignLift.setLinearHeadingInterpolation(lift.getHeading(), outtake.getHeading());
    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(alignLift);
                setPathState(1);
                break;
            case 1:
                if (calculatePoseError(follower.getPose(), lift) < 10) {
                    robotLift.setMode(Lift.Mode.HIGH_BASKET);
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(dropoff1);
                setPathState(-1);
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
