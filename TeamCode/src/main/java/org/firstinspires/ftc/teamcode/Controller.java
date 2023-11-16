package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

class Controller {
    // Defines controller behavior
    private Gamepad gamepad;

    /* This is kind of a neat implementation:
       https://github.com/cporter/ftc_app/blob/vv/autonomous-testing/TeamCode/src/main/java/com/suitbots/vv/Controller.java
    */

    // Per the above link, these are the int values that will increment as buttons are held down
    // private because we don't want to expose them.  We want to expose methods that use them
    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b;
    private int left_bumper, right_bumper;
    // These will simply pass the value from the gamepad object out through the Controller object
    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;
    // [TBD]


    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // This updates all of the controller object's class variables according to gamepad actions
        // this method should be called once for every teleop loop
        if (gamepad.x) {
            ++x;
        } else {
            x = 0;
        }
        if (gamepad.y) {
            ++y;
        } else {
            y = 0;
        }
        if (gamepad.a) {
            ++a;
        } else {
            a = 0;
        }
        if (gamepad.b) {
            ++b;
        } else {
            b = 0;
        }
        if (gamepad.dpad_up) {
            ++this.dpad_up;
        } else {
            this.dpad_up = 0;
        }
        if (gamepad.dpad_down) {
            ++this.dpad_down;
        } else {
            this.dpad_down = 0;
        }
        if (gamepad.dpad_left) {
            ++this.dpad_left;
        } else {
            this.dpad_left = 0;
        }
        if (gamepad.dpad_right) {
            ++this.dpad_right;
        } else {
            this.dpad_right = 0;
        }
        if (gamepad.left_bumper) {
            ++this.left_bumper;
        } else {
            this.left_bumper = 0;
        }
        if (gamepad.right_bumper) {
            ++this.right_bumper;
        } else {
            this.right_bumper = 0;
        }

        this.left_stick_x = gamepad.left_stick_x;
        this.left_stick_y = gamepad.left_stick_y;
        this.right_stick_x = gamepad.right_stick_x;
        this.right_stick_y = gamepad.right_stick_y;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;
    }

    // These methods are for "is currently pressed" (i.e. "hold" behavior)
    public boolean dpadUp() {
        return 0 < this.dpad_up;
    }

    public boolean dpadDown() {
        return 0 < this.dpad_down;
    }

    public boolean dpadLeft() {
        return 0 < this.dpad_left;
    }

    public boolean dpadRight() {
        return 0 < this.dpad_right;
    }

    public boolean X() {
        return 0 < this.x;
    }

    public boolean Y() {
        return 0 < this.y;
    }

    public boolean A() {
        return 0 < this.a;
    }

    public boolean B() {
        return 0 < this.b;
    }

    public boolean leftBumper() {
        return 0 < this.left_bumper;
    }

    public boolean rightBumper() {
        return 0 < this.right_bumper;
    }

    // These methods are for "was just pressed" (i.e. press once behavior)
    public boolean dpadUpOnce() {
        return 1 == this.dpad_up;
    }

    public boolean dpadDownOnce() {
        return 1 == this.dpad_down;
    }

    public boolean dpadLeftOnce() {
        return 1 == this.dpad_left;
    }

    public boolean dpadRightOnce() {
        return 1 == this.dpad_right;
    }

    public boolean XOnce() {
        return 1 == this.x;
    }

    public boolean YOnce() {
        return 1 == this.y;
    }

    public boolean AOnce() {
        return 1 == this.a;
    }

    public boolean BOnce() {
        return 1 == this.b;
    }

    public boolean leftBumperOnce() {
        return 1 == this.left_bumper;
    }

    public boolean rightBumperOnce() {
        return 1 == this.right_bumper;
    }

}