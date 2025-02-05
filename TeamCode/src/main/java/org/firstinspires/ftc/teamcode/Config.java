package org.firstinspires.ftc.teamcode;

public class Config {
    public static class Drivetrain {
        public static String frontLeft = "frontLeft";
        public static String frontRight = "frontRight";
        public static String rearLeft = "rearLeft";
        public static String rearRight = "rearRight";
        public static double speed = 1.0;
    }

    public static class Lift {
        public static String lift = "lift";
        public static int liftMin = 0;
        public static int liftIntake = 3500; // TODO: find lift position for intake
        public static int liftMax = 4500; // REVERSE
    }
    public static class Rotator {
        public static String rotator = "rotator";
        public static int rotatorMin = 0;
        public static int rotatorIntake = 100; // TODO: Find rotator position for intake
        public static int rotatorMax = 3000; // REVERSE
    }
    public static class Arm {
        static String flipper = "flipper";
        static int flipMax = 2500; //TODO: add max
        static int flipMin = 0;
    }

    public static class Intake {
        public static String wrist = "wrist";
        public static String intakeLeft = "intakeLeft";
        public static String intakeRight = "intakeRight";
        public static String color = "color";
    }
    public static class Color {
        public static String color = "color";
        public static String led = "led";

    }
}
