package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.WafflesUtil.clamp;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

public class Lift {
    HardwareMap hardwareMap;

    DcMotor arm;
    DcMotor lift;

    enum states {
        HOME,
        INTAKE,
        OUTTAKE
    }
    states state = states.HOME;

    int targetArm = 0;
    int targetLift = 0;

    BasicPID armController;
    BasicPID liftController;

    public Lift (HardwareMap map) {
        hardwareMap = map;

        arm = hardwareMap.get(DcMotor.class, RobotMap.Lift.arm);
        lift = hardwareMap.get(DcMotor.class, RobotMap.Lift.lift);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDCoefficients armCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        armController = new BasicPID(armCoefficients);

        PIDCoefficients liftCoefficients = new PIDCoefficients(0.01, 0, 0.01);
        liftController = new BasicPID(liftCoefficients);
    }

    // retract lift
    // rotate lift
    // extend lift
    private double angleMap(double angle) {
        return angle * -28;
    }

    private void setArmTarget(int armPos) {
        targetArm = clamp(armPos, RobotMap.Lift.armMin, RobotMap.Lift.armMax);
    }

    private void setLiftTarget(int liftPos) {
        targetLift = clamp(liftPos, RobotMap.Lift.liftMin, RobotMap.Lift.liftMax);
    }

    public void update() {
        arm.setPower(armController.calculate(targetArm, arm.getCurrentPosition()));
        lift.setPower(liftController.calculate(targetLift, lift.getCurrentPosition()));
        switch (state) {
            case HOME:
                setLiftTarget(RobotMap.Lift.liftMin);
                if (isAtPosition(lift, RobotMap.Lift.liftMin)) {
                    setArmTarget(RobotMap.Lift.liftMin);
                }
                break;
            case INTAKE:
                if (!isAtPosition(arm, RobotMap.Lift.armIntake)) {
                    if (!isAtPosition(lift, RobotMap.Lift.liftMin)) {
                        setLiftTarget(RobotMap.Lift.liftMin);
                        break;
                    }
                    setArmTarget(RobotMap.Lift.armIntake);
                }
                else if (!isAtPosition(lift, RobotMap.Lift.liftIntake)) {
                    setLiftTarget(RobotMap.Lift.liftIntake);
                }
                break;
        }
    }

    public void home() {
        state = states.HOME;
    }

    public void intake() {
        state = states.INTAKE;
    }

    public void outtake() {
        state = states.OUTTAKE;
    }

    private boolean isAtPosition(DcMotor motor, int position) {
        return Math.abs(motor.getCurrentPosition() - position) < 10;
    }
}