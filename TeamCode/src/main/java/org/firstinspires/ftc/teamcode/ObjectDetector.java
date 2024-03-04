package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ObjectDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public ObjectDetector(Telemetry t) {telemetry = t;}
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(70, 20, 20);
        Scalar highHSV = new Scalar(255, 100, 100);


        Core.inRange(mat, lowHSV, highHSV, mat);
    }
}
