package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
    FlipGrip flipgrip;
    Controller controller;
    public TeleOpControl (Controller controller, HardwareMap hardwareMap) {
        driver = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        flipgrip = new FlipGrip(hardwareMap);
        this.controller = controller;
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
        if (controller.dpadUpOnce()) {
            lift.goToTop();
        }
        if (controller.dpadDownOnce()) {
            lift.setPosition(0);
        }
        if (lift.getPosition() > 3000) {
            flipgrip.open();
        }
        else {
            flipgrip.close();
        }
    }
    public void flipGrip () {
        if (controller.XOnce() && !lift.isAtBottom()) {
            flipgrip.flip();
        }
        if (controller.AOnce() && !lift.isAtBottom()) {
            flipgrip.grip();
        }
    }
}