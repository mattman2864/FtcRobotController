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

@Autonomous (name="Specimen Auto")
public class SpecimenAuto extends OpMode {
    // PLACING SPECIMEN
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start
    private final Pose poleLineup = new Pose(10, 0, Math.toRadians(0)); // Lift position
    private final Pose placeSpecimen0 = new Pose(24, 8, Math.toRadians(0)); // Place position

    private final Pose placeSpecimen1 = new Pose(24, 6, Math.toRadians(0)); // Place position
    private final Pose placeSpecimen2 = new Pose(24, 4, Math.toRadians(0)); // Place position
    private final Pose placeSpecimen3 = new Pose(24, 2, Math.toRadians(0)); // Place position
    private final Pose backup = new Pose(20, 7, Math.toRadians(0)); // Backing up after lifting


    // PUSHING
    private final Pose midPush = new Pose(52, -24 , Math.toRadians(0)); // Middle position for bezier curve
    private final Pose pushLineup1 = new Pose(18, -25, Math.toRadians(0)); // Back position before push
    private final Pose pushLineup2 = new Pose(55, -35, Math.toRadians(0)); // Behind block 1
    private final Pose pushLineup3 = new Pose(6, -37, Math.toRadians(0)); // Push block 1
    private final Pose pushLineup4 = new Pose(52, -41, Math.toRadians(0)); // Behind block 2
    private final Pose pushLineup5 = new Pose(6, -45, Math.toRadians(0)); // Push block 2
    private Timer pathTimer, opmodeTimer;
    int pathState = 0;
    Follower follower;
    Path alignLift;
    Path specimen0;
    Path back0;
    Path specimen1;
    Path back1;
    Path specimen2;
    Path back2;
    Path specimen3;
    Path back3;
    Path push1;
    Path push2;
    Path push3;
    Path push4;
    Path push5;
    Path lineup2;
    Path lineup3;
    Path park;
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
        alignLift = createPathFromPoints(startPose, poleLineup);
        specimen0 = createPathFromPoints(poleLineup, placeSpecimen0);
        back0 = createPathFromPoints(placeSpecimen0, backup);
        specimen1 = createPathFromPoints(poleLineup, placeSpecimen1);
        back1 = createPathFromPoints(placeSpecimen1, backup);
        specimen2 = createPathFromPoints(poleLineup, placeSpecimen2);
        back2 = createPathFromPoints(placeSpecimen2, backup);
        specimen3 = createPathFromPoints(poleLineup, placeSpecimen3);
        back3 = createPathFromPoints(placeSpecimen3, backup);
        push1 = createPathFromPoints(backup, pushLineup1);
        push2 = new Path(
                new BezierCurve(new Point(pushLineup1), new Point(midPush), new Point(pushLineup2))
        );
        push2.setLinearHeadingInterpolation(pushLineup1.getHeading(), pushLineup2.getHeading());
        pushChain1 = follower.pathBuilder()
                .addPath(push1)
                .addPath(push2)
                .build();

        push3 = createPathFromPoints(pushLineup2, pushLineup3);
        push4 = new Path(
                new BezierCurve(new Point(backup), new Point(startPose), new Point(pushLineup3), new Point(midPush), new Point(pushLineup4))
        );
        push4.setLinearHeadingInterpolation(pushLineup3.getHeading(), pushLineup4.getHeading());
        push5 = createPathFromPoints(pushLineup4, pushLineup5);
        lineup2 = createPathFromPoints(pushLineup3, poleLineup);
        lineup3 = createPathFromPoints(pushLineup5, poleLineup);
        park = createPathFromPoints(backup, pushLineup3);

    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(alignLift);
                robotLift.fineTuneFlipper(0.1);
                robotLift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(specimen0);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 800) {
                    robotLift.intake(-1);
                    follower.followPath(back0);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 500) {
                    robotLift.intake(0);
                    follower.followPath(pushChain1);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PICKUP);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 2500) {
                    robotLift.intake(1);
                    follower.followPath(push3);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 1800) {
                    robotLift.intake(0);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 200) {
                    follower.followPath(lineup2);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                    robotLift.fineTuneFlipper(0.25);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 1000) {
                    follower.followPath(specimen1);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 800) {
                    robotLift.intake(-1);
                    follower.followPath(back1);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > 200) {
                    robotLift.intake(0);
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTime() > 400) {
                    follower.followPath(push4);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PICKUP);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 2800) {
                    follower.followPath(push5);
                    robotLift.intake(1);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 1800) {
                    robotLift.intake(0);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime() > 200) {
                    follower.followPath(lineup3);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                    robotLift.fineTuneFlipper(0.25);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTime() > 1200) {
                    follower.followPath(specimen2);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 1000) {
                    robotLift.intake(-1);
                    follower.followPath(back2);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTime() > 500) {
                    robotLift.intake(0);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTime() > 200) {
                    follower.followPath(push4);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PICKUP);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 3000) {
                    follower.followPath(push5);
                    robotLift.intake(1);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTime() > 1800) {
                    robotLift.intake(0);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTime() > 200) {
                    follower.followPath(lineup3);
                    robotLift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                    robotLift.fineTuneFlipper(0.25);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTime() > 1200) {
                    follower.followPath(specimen3);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTime() > 800) {
                    robotLift.intake(-1);
                    follower.followPath(back3);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTime() > 300) {
                    robotLift.intake(0);
                    follower.followPath(park);
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
