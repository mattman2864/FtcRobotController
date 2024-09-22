package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Robot {
    public Drivetrain drivetrain;
    // Add more subsystems later

    public Robot (HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
    }
}
