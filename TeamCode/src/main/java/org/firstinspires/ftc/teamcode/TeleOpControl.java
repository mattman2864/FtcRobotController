package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Objects;

public class TeleOpControl {
    Drive driver;
    Intake intake;
    Lift lift;
    Controller controller;
    Launcher launcher;
    boolean out;
    boolean start = true;
    int lastEncoder = 0;
    LinkedList<Integer> previousEncoders = new LinkedList<Integer>();
    LinkedList<Integer> currentEncoders = new LinkedList<Integer>();
    boolean lastManual = false;
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
    public void intake (ElapsedTime time, Gamepad gamepad) {
        if (controller.B() && !lift.isUp()) {
            intake.on(1);
            time.reset();
            start = false;
        } else if (controller.dpadLeft() && !lift.isUp()) {
            intake.reverse(0.3);
        } else if (time.milliseconds() < 600 && !start) {
            intake.reverse(0.75);
        } else {
            intake.off();
        }
        if (intake.isFull() && Objects.equals(lift.liftState, "down") && time.milliseconds() < 3000) {
            gamepad.rumble(0.2, 0.2,300);
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
            if (!lift.isUp()) {
                lift.place();
            } else {
                lift.prePosUp();
            }
        }
        if (controller.dpadDownOnce()) {
            if (lift.prePos == 1) {
                lift.down();
            } else {
                lift.prePosDown();
            }
        }
        if (controller.AOnce() && lift.isUp()) {
            lift.drop();
            time.reset();
        }
        if (controller.right_trigger > 0) {
            lift.manualUp();
            lastManual = true;
        } else if (controller.left_trigger > 0) {
            lift.manualDown();
            lastManual = true;
        } else if (lastManual) {
            lift.manualHold();
            lastManual = false;
        }
        if (controller.XOnce()) {
            if (lift.isOut()) {
                lift.close();
            } else {
                lift.open(false);
            }
        }
    }
    public void launch () {
        if (controller.leftBumper() && controller.rightBumper()) {
            launcher.shoot();
        }
    }
}