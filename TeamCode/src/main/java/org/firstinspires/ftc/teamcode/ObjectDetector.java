package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Objects;

public class ObjectDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    enum Location {
        CENTER,
        RIGHT,
        NOT_FOUND
    }
    private Location location;
    Rect CENTER = new Rect (
            new Point(10, 30),
            new Point(90, 70)
    );
    Rect RIGHT = new Rect (
            new Point(140, 50),
            new Point(220, 90)
    );
    static final double PERCENT_COLOR_THRESHOLD = 0.2;
    boolean left;
    public ObjectDetector(Telemetry t, boolean l) {telemetry = t; left = l;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 100, 0);
        Scalar highHSV = new Scalar(255, 255, 255);

        if (left) {
            CENTER = new Rect (
                    new Point(110, 30),
                    new Point(190, 70)
            );
            RIGHT = new Rect (
                    new Point(240, 50),
                    new Point(320, 90)
            );
        }

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat center = mat.submat(CENTER);
        Mat right = mat.submat(RIGHT);

        double centerValue = Core.sumElems(center).val[0] / CENTER.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT.area() / 255;

        center.release();
        right.release();

        telemetry.addData("center raw value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("center percentage", Math.round(centerValue * 100) + "%");
        telemetry.addData("center percentage", Math.round(centerValue * 100) + "%");

        boolean stoneCenter = centerValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (!(stoneRight || stoneCenter)) {
            // not found
            location = Location.NOT_FOUND;
            telemetry.addData("location", "not found");
        }
        else if (stoneCenter) {
            // center
            location = Location.CENTER;
            telemetry.addData("location", "center");
        } else {
            // right
            location = Location.RIGHT;
            telemetry.addData("location", "right");
        }
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        Scalar colorDetect = new Scalar(255, 0, 0);
        Scalar colorNone = new Scalar(100, 100, 100);

        Imgproc.rectangle(mat, CENTER, location==Location.CENTER? colorDetect:colorNone);
        Imgproc.rectangle(mat, RIGHT, location==Location.RIGHT? colorDetect:colorNone);

        return mat;
    }
    public Location getLocation() {
        return location;
    }
}
