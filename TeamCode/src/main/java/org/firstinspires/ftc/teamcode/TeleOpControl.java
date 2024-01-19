package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
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
        launcher = new Launcher(hardwareMap);
        this.controller = controller;
        boolean out = true;
    }
    public void drive () {
        driver.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
    }
    public void intake (ElapsedTime time) {
        if (controller.B() && !lift.isUp()) {
            intake.on(0.75);
        } else if (controller.dpadLeft() && !lift.isUp()) {
            intake.reverse(0.3);
        } else {
            intake.off();
        }
    }
    public void lift (ElapsedTime time) {
        lift.update(time);
        if (controller.YOnce()) {
            if (lift.isUp()) {
                lift.down();
            } else {
                lift.place();
            }
        }
        if (controller.dpadUpOnce()) {
            lift.prePosUp();
        }
        if (controller.dpadDownOnce()) {
            lift.prePosDown();
        }
        if (controller.A() && lift.isUp()) {
            lift.drop();
        }
        if (controller.right_trigger > 0) {
            lift.manualUp();
        } else if (controller.left_trigger > 0) {
            lift.manualDown();
        } else if (lift.isManual()) {
            lift.manualHold();
        }
    }
    public void launch () {
        if (controller.leftBumper() && controller.rightBumper()) {
            launcher.shoot();
        }
    }
}