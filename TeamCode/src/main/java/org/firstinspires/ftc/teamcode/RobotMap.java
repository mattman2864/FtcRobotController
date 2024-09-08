package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotMap {
    static HardwareMap hardwareMap;
    public RobotMap(HardwareMap map) {
        hardwareMap = map;
    }
    static DcMotor motor1 = hardwareMap.get(DcMotor.class, "a");
}
