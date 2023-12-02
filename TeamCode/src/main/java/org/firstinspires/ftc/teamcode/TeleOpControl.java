package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
    FlipGrip flipgrip;
    Controller controller;
    boolean out;
    public TeleOpControl (Controller controller, HardwareMap hardwareMap) {
        driver = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        flipgrip = new FlipGrip(hardwareMap);
        this.controller = controller;
        boolean out = true;
    }
    public void drive () {
        driver.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
    }

    public void intake () {
        if (controller.B()) {
            intake.on(1);
        } else if (controller.dpadLeft()) {
            intake.reverse(0.3);
        } else {
            intake.off();
        }
    }
    public void lift () {
        lift.checkForZero();
        if (controller.right_trigger > 0) {
            lift.slowMove(150);
        }
        if (controller.left_trigger > 0) {
            lift.slowMove(-150);
        }
        if (controller.YOnce()) {
            if (lift.isAtBottom() || !out) {
                lift.goToTop();
                out = true;
            }
            else {
                if (flipgrip.isFlipped()){
                    flipgrip.flip();
                }
                lift.setPosition(0);
                out = false;
            }
        }
    }
    public void flipGrip () {
        if (controller.XOnce() && !lift.isAtBottom()) {
            out = false;
            flipgrip.flip();
        }
        if (controller.AOnce() && !lift.isAtBottom()) {
            out = false;
            flipgrip.grip();
        }
        if (out && lift.getPosition() > 1000 && !flipgrip.isFlipped()) {
            flipgrip.flip();
        }
        if (lift.getPosition() < 2000 && flipgrip.isFlipped()) {
            flipgrip.flip();
        }
    }
}