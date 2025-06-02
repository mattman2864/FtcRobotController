package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous (name="Spiral")
public class Spiral extends OpMode {
    // PLACING SPECIMEN
    private final double dpt = 20;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start
    private final Pose pose1 = new Pose(dpt * 5, 0, Math.toRadians(0));
    private final Pose pose2 = new Pose(dpt * 5, dpt*5, Math.toRadians(0));
    private final Pose pose3 = new Pose(0, dpt*5, Math.toRadians(0));
    private final Pose pose4 = new Pose(0, dpt, Math.toRadians(0));
    private final Pose pose5 = new Pose(dpt*4, dpt, Math.toRadians(0));
    private final Pose pose6 = new Pose(dpt*4, dpt*4, Math.toRadians(0));
    private final Pose pose7 = new Pose(dpt, dpt*4, Math.toRadians(0));
    private final Pose pose8 = new Pose(dpt, dpt*2, Math.toRadians(0));
    private final Pose pose9 = new Pose(dpt*3, dpt*2, Math.toRadians(0));
    private final Pose pose10 = new Pose(dpt*3, dpt*3, Math.toRadians(180));

    private Timer pathTimer, opmodeTimer;
    int pathState = 0;
    Follower follower;
    Path p12;
    Path p23;
    Path p34;
    Path p45;
    Path p56;
    Path p67;
    Path p78;
    Path p89;
    Path p910;
    PathChain pushChain1;
    Lift robotLift;
    public Path createPathFromPoints(Pose pose1, Pose pose2) {
        Path newPath = new Path(
                new BezierLine(new Point(pose1), new Point(pose2))
        );
        newPath.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());
        return newPath;
    }
    public void buildPaths() {
        p12 = createPathFromPoints(pose1, pose2);
        p23 = createPathFromPoints(pose2, pose3);
        p34 = createPathFromPoints(pose3, pose4);
        p45 = createPathFromPoints(pose4, pose5);
        p56 = createPathFromPoints(pose5, pose6);
        p67 = createPathFromPoints(pose6, pose7);
        p78 = createPathFromPoints(pose7, pose8);
        p89 = createPathFromPoints(pose8, pose9);
        p910 = createPathFromPoints(pose9, pose10);

    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(p12);
                robotLift.setMode(Lift.Mode.HOME);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p23);
                setPathState(2);
                break;
            case 2:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p34);
                setPathState(3);
                break;
            case 3:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p45);
                setPathState(4);
                break;
            case 4:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p56);
                setPathState(5);
                break;
            case 5:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p67);
                setPathState(6);
                break;
            case 6:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p78);
                setPathState(7);
                break;
            case 7:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p89);
                setPathState(8);
                break;
            case 8:
                if (pathTimer.getElapsedTime() < 2000)  {
                    return;
                }
                follower.followPath(p910);
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
        robotLift.resetFlip();

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
