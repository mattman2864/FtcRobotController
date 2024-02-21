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
    Controller controller2;
    Launcher launcher;

    ExtraCommands extraCommands;

    Displayer displayer;

    AprilTagDetector aprilTagDetector;
    boolean out;
    boolean start = true;
    int lastEncoder = 0;
    LinkedList<Integer> previousEncoders = new LinkedList<Integer>();
    LinkedList<Integer> currentEncoders = new LinkedList<Integer>();
    boolean lastManual = false;
    public TeleOpControl (Controller controller, HardwareMap hardwareMap, Controller controller2) {
        driver = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        launcher = new Launcher(hardwareMap);
        extraCommands = new ExtraCommands(hardwareMap);
        displayer = new Displayer();
        aprilTagDetector = new AprilTagDetector(hardwareMap);
        this.controller = controller;
        this.controller2 = controller2;
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
        if (controller2.YOnce()) {
            if (lift.isUp()) {
                lift.down();
            } else {
                lift.place();
            }
        }
        if (controller2.dpadUpOnce()) {
            if (!lift.isUp()) {
                lift.place();
            } else {
                lift.prePosUp();
            }
        }
        if (controller2.dpadDownOnce()) {
            if (lift.prePos == 1) {
                lift.down();
            } else {
                lift.prePosDown();
            }
        }
        if (controller2.AOnce() && lift.isUp()) {
            lift.drop();
            time.reset();
        }
        if (controller2.right_trigger > 0) {
            lift.manualUp();
            lastManual = true;
        } else if (controller2.left_trigger > 0) {
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

    public void ExtraCommands() {

        if (controller.leftBumper()) {
            extraCommands.moveLeft();
        }

        if (controller.rightBumper()) {
            extraCommands.moveRight();
        }
    }
}