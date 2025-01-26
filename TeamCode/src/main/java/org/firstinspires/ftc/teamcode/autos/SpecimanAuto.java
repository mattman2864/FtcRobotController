package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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

@Autonomous(name = "Speciman Auto")
public class SpecimanAuto extends OpMode {

    //
    // Poses and Points
    //

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // Lineup Poses

    private final double lineupSpacing = 3.0;

    private final double distanceToClip = 18.0;

    private final Pose lineupPose1 = new Pose(10.6, 0, Math.toRadians(0));

    private final Pose lineupPose2 = new Pose(lineupPose1.getX(), lineupPose1.getY()-lineupSpacing, Math.toRadians(0));

    private final Pose lineupPose3 = new Pose(lineupPose1.getX(), lineupPose1.getY()-2*lineupSpacing, Math.toRadians(0));

    private final Pose lineupPose4 = new Pose(lineupPose1.getX(), lineupPose1.getY()-3*lineupSpacing, Math.toRadians(0));

    // Clipping Poses

    private final Pose clippingPose1 = new Pose(lineupPose1.getX()+distanceToClip, lineupPose1.getY(), Math.toRadians(0));

    private final Pose clippingPose2 = new Pose(clippingPose1.getX(), lineupPose1.getY()-lineupSpacing, Math.toRadians(0));

    private final Pose clippingPose3 = new Pose(clippingPose1.getX(), lineupPose1.getY()-2*lineupSpacing, Math.toRadians(0));

    private final Pose clippingPose4 = new Pose(clippingPose1.getX(), lineupPose1.getY()-3*lineupSpacing, Math.toRadians(0));

    // Ground pick up Poses

    private final Pose groundPickupPose1 = new Pose(18.87, -38.90, Math.toRadians(0));

    private final Pose groundPickupPose2 = new Pose(groundPickupPose1.getX(), groundPickupPose1.getY() + 10, Math.toRadians(0));

    private final Pose groundPickupPose3 = new Pose();

    // Dropoff Poses

    private final Pose dropoffPose1 = new Pose();

    private final Pose dropoffPose2 = new Pose();

    private final Pose dropoffPose3 = new Pose();

    // Speciman Pickup Pose

    private final Pose specimanPickupLineupPose = new Pose();

    private final Pose specimanPickupPose = new Pose();

    //
    // Paths
    //

    PathChain initLineup;

    PathChain initClip;

    PathChain toGroundPickup1;

    PathChain dropoff1;

    PathChain toGroundPickup2;

    PathChain dropoff2;

    PathChain toGroundPickup3;

    PathChain dropoff3;

    PathChain retrieveFirstSpecimen;

    PathChain clip1;

    PathChain retrieveSecondSpecimen;

    PathChain clip2;

    PathChain retrieveThirdSpecimen;

    PathChain clip3;

    PathChain park;

    //
    // Other variables
    //

    private Timer pathTimer, liftTimer, opmodeTimer;

    int pathState = 0;

    int liftState = 0;

    Follower follower;

    Lift lift;

    public void buildPaths() {

        initLineup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(lineupPose1)))
                .setConstantHeadingInterpolation(0)
                .build();

        initClip = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineupPose1), new Point(clippingPose1)))
                .setConstantHeadingInterpolation(0)
                .build();

        toGroundPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clippingPose1), new Point(groundPickupPose1)))
                .setConstantHeadingInterpolation(0)
                .build();

        dropoff1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundPickupPose1), new Point(dropoffPose1)))
                .setConstantHeadingInterpolation(0)
                .build();

        toGroundPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose1), new Point(groundPickupPose2)))
                .setConstantHeadingInterpolation(0)
                .build();

        dropoff2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundPickupPose2), new Point(dropoffPose2)))
                .setConstantHeadingInterpolation(0)
                .build();

        toGroundPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose2), new Point(groundPickupPose3)))
                .setConstantHeadingInterpolation(0)
                .build();

        dropoff3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(groundPickupPose3), new Point(dropoffPose3)))
                .setConstantHeadingInterpolation(0)
                .build();

        retrieveFirstSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropoffPose3), new Point(specimanPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimanPickupLineupPose), new Point(specimanPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimanPickupPose), new Point(lineupPose2)))
                .addPath(new BezierLine(new Point(lineupPose2), new Point(clippingPose2)))
                .setConstantHeadingInterpolation(0)
                .build();

        retrieveSecondSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clippingPose2), new Point(specimanPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimanPickupLineupPose), new Point(specimanPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimanPickupPose), new Point(lineupPose3)))
                .addPath(new BezierLine(new Point(lineupPose3), new Point(clippingPose3)))
                .setConstantHeadingInterpolation(0)
                .build();

        retrieveThirdSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clippingPose3), new Point(specimanPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimanPickupLineupPose), new Point(specimanPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimanPickupPose), new Point(lineupPose4)))
                .addPath(new BezierLine(new Point(lineupPose4), new Point(clippingPose4)))
                .setConstantHeadingInterpolation(0)
                .build();
    }
    public void pathStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(initLineup);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 2500) {
                    follower.followPath(initClip);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 4500) {
                    follower.followPath(toGroundPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(dropoff1);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(toGroundPickup2);
                    //setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(dropoff2);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(toGroundPickup3);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(dropoff3);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(retrieveFirstSpecimen);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(clip1);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(retrieveSecondSpecimen);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(clip2);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(retrieveThirdSpecimen);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(clip3);
                    setPathState(14);
                }
                break;
            case 14:
                follower.followPath(park);
            default:
                break;
        }
    }

    public void liftStateUpdate() {
        switch (liftState) {
            case 0:
                lift.setMode(Lift.Mode.SPECIMAN_PLACE_HIGH);
                setLiftState(1);
                break;
            case 1:
                if (liftTimer.getElapsedTime() > 6500)
                {
                    lift.setMode(Lift.Mode.HOME);
                    lift.intake(0);
                    setLiftState(2);
                } else if (liftTimer.getElapsedTime() > 6000)
                {
                    lift.intake(-1);
                }
                break;
            case 2:
                if (liftTimer.getElapsedTime() > 3200) {
                    lift.setMode(Lift.Mode.SPECIMAN_PICKUP);
                    lift.intake(0);
                    setLiftState(2);
                } else if (liftTimer.getElapsedTime() > 3000)
                {
                    lift.setMode(Lift.Mode.FRONT_PICKUP_DROP);
                    lift.intake(1);
                    setLiftState(2);
                } else if (liftTimer.getElapsedTime() > 2500) {
                    lift.setMode(Lift.Mode.FRONT_PICKUP);
                }
                break;
            case 3:
                if (liftTimer.getElapsedTime() > 1200)
                {
                    lift.intake(0);
                    lift.setMode(Lift.Mode.FRONT_PICKUP);
                    setLiftState(4);
                }
                else if (liftTimer.getElapsedTime() > 1000)
                {
                    lift.intake(-1);
                    //setLiftState(4);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setLiftState(int lState) {
        liftState = lState;
        liftTimer.resetTimer();
    }

    public double calculatePoseError(Pose position, Pose target) {
        return Math.abs((target.getX() - position.getX()) + (target.getY() - position.getY())
                + 2 * (target.getHeading() - position.getHeading()));
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        liftTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        lift = new Lift(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setLiftState(0);
    }

    @Override
    public void loop() {
        follower.update();

        pathStateUpdate();
        liftStateUpdate();

        lift.update();

        telemetry.addData("Path State:", pathState);
        telemetry.addData("Lift State:", liftState);
        telemetry.addData("X:", follower.getPose().getX());
        telemetry.addData("Y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.update();
    }
}