package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
    FlipGrip flipgrip;
    Controller controller;
    boolean out;
    int x;
    int y;
    int r;
    public TeleOpControl (Controller controller, HardwareMap hardwareMap) {
        driver = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        flipgrip = new FlipGrip(hardwareMap);
        this.controller = controller;
        boolean out = true;
    }
    public void drive () {
        x = controller.left_stick_x;
        r = controller.right_stick_x;
        if (controller.leftBumper()) {
            y = controller.left_stick_y + 1;
        } else if (controller.rightBumper()) {
            y = controller.left_stick_y + 1;
        } else {
            y = controller.left_stick_y;
        }
        driver.drive(x, y, r);
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
}