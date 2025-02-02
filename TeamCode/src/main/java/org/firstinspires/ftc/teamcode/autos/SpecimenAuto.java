package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends OpMode {

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

    // Pushing Poses

    private final Pose pushPose1 = new Pose(22.8, -26.3, 0);

    private final Pose pushPose2 = new Pose(51.2, -26.3, 0);

    private final Pose pushPose3 = new Pose(51.2, -38.2, 0);

    private final Pose pushPose4 = new Pose(12.3, -38.2, 0);

    private final Pose pushPose5 = new Pose(51.2, -48.2, 0);

    private final Pose pushPose6 = new Pose(12.3, -48.2, 0);

    private final Pose pushPose7 = new Pose(51.2, -58.2,0);

    private final Pose pushPose8 = new Pose(12.3, -58.2, 0);

    // Ground pick up Poses

    private final Pose groundPickupPose1 = new Pose(18.87, -38.90, Math.toRadians(0));

    private final Pose groundPickupPose2 = new Pose(groundPickupPose1.getX(), groundPickupPose1.getY() + 10, Math.toRadians(0));

    private final Pose groundPickupPose3 = new Pose();

    // Dropoff Poses

    private final Pose dropoffPose1 = new Pose(groundPickupPose1.getX() - 10, groundPickupPose1.getY(), 0);

    private final Pose dropoffPose2 = new Pose(groundPickupPose2.getX() - 10, groundPickupPose2.getY(), 0);

    private final Pose dropoffPose3 = new Pose();

    // Specimen Pickup Pose

    private final Pose specimenPickupLineupPose = new Pose();

    private final Pose specimenPickupPose = new Pose();

    //
    // Paths
    //

    PathChain initLineup;

    PathChain initClip;

    PathChain backtoInitLineUp;

    // For ground pickup
    PathChain toGroundPickup1;

    PathChain dropoff1;

    PathChain toGroundPickup2;

    PathChain dropoff2;

    PathChain toGroundPickup3;

    PathChain dropoff3;

    // For ground pushing

    PathChain pushPath;

    PathChain pushPath2;

    PathChain pushPath3;

    PathChain pushPath4;

    PathChain pushPath5;

    PathChain pushPath6;

    PathChain pushPath7;

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

        backtoInitLineUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(lineupPose1), new Point(startPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        // Pushing

        pushPath = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pushPose1)))
                //.addPath(new BezierLine(new Point(pushPose1), new Point(pushPose2)))
                //.addPath(new BezierLine(new Point(pushPose2), new Point(pushPose3)))
                //.addPath(new BezierLine(new Point(pushPose3), new Point(pushPose4)))
                //.addPath(new BezierLine(new Point(pushPose4), new Point(pushPose3)))
                //.addPath(new BezierLine(new Point(pushPose3), new Point(pushPose5)))
                //.addPath(new BezierLine(new Point(pushPose5), new Point(pushPose6)))
                //.addPath(new BezierLine(new Point(pushPose6), new Point(pushPose5)))
                //.addPath(new BezierLine(new Point(pushPose5), new Point(pushPose7)))
                //.addPath(new BezierLine(new Point(pushPose7), new Point(pushPose8)))
                .setConstantHeadingInterpolation(0)
                .build();

        // Ground Pickup

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
                .addPath(new BezierLine(new Point(dropoffPose3), new Point(specimenPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimenPickupLineupPose), new Point(specimenPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupPose), new Point(lineupPose2)))
                .addPath(new BezierLine(new Point(lineupPose2), new Point(clippingPose2)))
                .setConstantHeadingInterpolation(0)
                .build();

        retrieveSecondSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clippingPose2), new Point(specimenPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimenPickupLineupPose), new Point(specimenPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupPose), new Point(lineupPose3)))
                .addPath(new BezierLine(new Point(lineupPose3), new Point(clippingPose3)))
                .setConstantHeadingInterpolation(0)
                .build();

        retrieveThirdSpecimen = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clippingPose3), new Point(specimenPickupLineupPose)))
                .addPath(new BezierLine(new Point(specimenPickupLineupPose), new Point(specimenPickupPose)))
                .setConstantHeadingInterpolation(0)
                .build();

        clip3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPickupPose), new Point(lineupPose4)))
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
                if (pathTimer.getElapsedTime() > 2500) {
                    follower.followPath(backtoInitLineUp);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(pushPath);
                    //setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(toGroundPickup2);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(dropoff2);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 5000) {
                    follower.followPath(toGroundPickup3);
                    //setPathState(7);
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
                lift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                setLiftState(1);
                break;
            case 1:
                if (liftTimer.getElapsedTime() > 10500)
                {
                    lift.setMode(Lift.Mode.HOME);
                    lift.intake(0);
                    //setLiftState(2);
                } else if (liftTimer.getElapsedTime() > 10000)
                {
                    lift.intake(-1);
                }
                break;
            case 2:
                if (liftTimer.getElapsedTime() > 3200) {
                    lift.setMode(Lift.Mode.SPECIMEN_PICKUP);
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
                if (liftTimer.getElapsedTime() > 2500) {
                    lift.intake(0);
                    setLiftState(4);
                } else if (liftTimer.getElapsedTime() > 2000) {
                    lift.intake(-1);
                }
                else if (liftTimer.getElapsedTime() > 1000)
                {
                    lift.setMode(Lift.Mode.SPECIMEN_PICKUP);
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