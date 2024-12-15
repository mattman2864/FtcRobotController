package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public Drivetrain drivetrain;
    public Lift lift;

    public Robot (HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        lift = new Lift(hardwareMap);
    }
    public void update() {
        lift.update();
    }
}
