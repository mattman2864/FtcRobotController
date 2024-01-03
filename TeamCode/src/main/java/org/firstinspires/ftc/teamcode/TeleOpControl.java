package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedList;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
    FlipGrip flipgrip;
    Controller controller;
    Launcher launcher;
    boolean out;
    int lastEncoder = 0;
    LinkedList<Integer> previousEncoders = new LinkedList<Integer>();
    LinkedList<Integer> currentEncoders = new LinkedList<Integer>();
    public TeleOpControl (Controller controller, HardwareMap hardwareMap) {
        driver = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        flipgrip = new FlipGrip(hardwareMap);
        launcher = new Launcher(hardwareMap);
        this.controller = controller;
        boolean out = true;
    }
    public void drive () {
        driver.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
    }
    public void intake () {
        if (controller.B() && lift.isAtBottom()) {
            intake.on(1);
        } else if (controller.dpadLeft() && lift.isAtBottom()) {
            intake.reverse(0.3);
        } else {
            intake.off();
        }
    }
    public void lift () {

        if (controller.right_trigger > 0) {
            lift.slowMove(true);
        }
        else if (controller.left_trigger > 0) {
            lift.slowMove(false);
        }
        else if (lift.lift.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            lift.setPosition(lift.getPosition());
            lift.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (controller.YOnce()) {
            if (lift.isAtBottom()) {
                lift.setPosition(3000);
                out = true;
            }
            else {
                if (flipgrip.isFlipped()){
                    flipgrip.flip(false);
                }
                if (flipgrip.isGripped()) {
                    flipgrip.grip();
                }
                lift.setPosition(0);
                out = false;
            }
        }
        lift.checkForZero();
    }
    public void flipGrip () {
        if (controller.XOnce() && !lift.isAtBottom()) {
            out = false;
            flipgrip.flip(false);
        }
        if (controller.AOnce() && !lift.isAtBottom()) {
            out = false;
            flipgrip.grip();
        }
        if (out && !flipgrip.isFlipped() && lift.getPosition() > 2000) {
            out = false;
            flipgrip.flip(false);
        }
        if (lift.getPosition() < 2000 && flipgrip.isFlipped()) {
            flipgrip.flip(false);
        }
        if (controller.leftBumper() && flipgrip.isFlipped()) {
            flipgrip.flip(true);
        }
    }
    public void launch () {
        if (controller.leftBumper() && controller.rightBumper()) {
            launcher.shoot();
        }
    }
}