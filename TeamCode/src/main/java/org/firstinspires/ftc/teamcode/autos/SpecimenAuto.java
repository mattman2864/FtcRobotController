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
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose poleLineup = new Pose(10, 0, Math.toRadians(0));
    private final Pose placeSpecimen = new Pose(28, 7, Math.toRadians(0));
    private final Pose backup = new Pose(20, 7, Math.toRadians(0));
    private final Pose midPush = new Pose(50, -30, Math.toRadians(0));
    private final Pose pushLineup1 = new Pose(18, -24, Math.toRadians(0));
    private final Pose pushLineup2 = new Pose(52, -36, Math.toRadians(0));
    private final Pose pushLineup3 = new Pose(10, -40, Math.toRadians(0));
    private final Pose pushLineup4 = new Pose(52, -44, Math.toRadians(0));
    private final Pose pushLineup5 = new Pose(10, -48, Math.toRadians(0));
    private final Pose pushLineup6 = new Pose(52, -52, Math.toRadians(0));
    private final Pose pushLineup7 = new Pose(10, -55, Math.toRadians(0));
    private Timer pathTimer, opmodeTimer;
    int pathState = 0;
    Follower follower;
    Path alignLift;
    Path specimen;
    Path back;
    Path push1;
    Path push2;
    Path push3;
    Path push4;
    Path push5;
    Path push6;
    Path push7;
    PathChain push;

    Lift robotLift;
    public void buildPaths() {
        alignLift = new Path(
                new BezierLine(new Point(startPose), new Point(poleLineup))
        );
        alignLift.setLinearHeadingInterpolation(startPose.getHeading(), poleLineup.getHeading());

        specimen = new Path(
                new BezierLine(new Point(poleLineup), new Point(placeSpecimen))
        );
        specimen.setLinearHeadingInterpolation(poleLineup.getHeading(), placeSpecimen.getHeading());

        back = new Path(
                new BezierLine(new Point(placeSpecimen), new Point(backup))
        );
        back.setLinearHeadingInterpolation(placeSpecimen.getHeading(), backup.getHeading());

        push1 = new Path(
                new BezierLine(new Point(backup), new Point(pushLineup1))
        );
        push1.setLinearHeadingInterpolation(backup.getHeading(), pushLineup1.getHeading());

        push2 = new Path(
                new BezierCurve(new Point(pushLineup1), new Point(midPush), new Point(pushLineup2))
        );
        push2.setLinearHeadingInterpolation(pushLineup1.getHeading(), pushLineup2.getHeading());

        push3 = new Path(
                new BezierLine(new Point(pushLineup2), new Point(pushLineup3))
        );
        push3.setLinearHeadingInterpolation(pushLineup2.getHeading(), pushLineup3.getHeading());

        push4 = new Path(
                new BezierCurve(new Point(pushLineup3), new Point(midPush), new Point(pushLineup4))
        );
        push4.setLinearHeadingInterpolation(pushLineup3.getHeading(), pushLineup4.getHeading());

        push5 = new Path(
                new BezierLine(new Point(pushLineup4), new Point(pushLineup5))
        );
        push5.setLinearHeadingInterpolation(pushLineup4.getHeading(), pushLineup5.getHeading());

        push6 = new Path(
                new BezierCurve(new Point(pushLineup5), new Point(midPush), new Point(pushLineup6))
        );
        push6.setLinearHeadingInterpolation(pushLineup5.getHeading(), pushLineup6.getHeading());

        push7 = new Path(
                new BezierLine(new Point(pushLineup6), new Point(pushLineup7))
        );
        push7.setLinearHeadingInterpolation(pushLineup6.getHeading(), pushLineup7.getHeading());
    }
    public void autoStateUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(alignLift);
                robotLift.fineTuneFlipper(-0.15);
                robotLift.setMode(Lift.Mode.SPECIMEN_PLACE_HIGH);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTime() > 3000) {
                    follower.followPath(specimen);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTime() > 3000) {
                    robotLift.intake(-1);
                    follower.followPath(back);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 2000) {
                    robotLift.intake(0);
                    robotLift.setMode(Lift.Mode.HOME);
                    follower.followPath(push1);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push2);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push3);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push4);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push5);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push6);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > 2000) {
                    follower.followPath(push7);
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
