package org.firstinspires.ftc.teamcode.Common;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CustomObjectDetectionLeftAlignment extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(20, 15),
            new Point(140, 75));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(180, 15),
            new Point(300, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public CustomObjectDetectionLeftAlignment(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        middle.release();

        //telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        //telemetry.addData("Right raw value", (int) Core.sumElems(middle).val[0]);
        // telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        // telemetry.addData("Right percentage", Math.round(middleValue * 100) + "%");

        boolean objectLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean objectMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if (objectMiddle) {
            location = Location.MIDDLE;
            telemetry.addData("Object Location", "middle");
        }
        else if (objectLeft) {
            location = Location.LEFT;
            telemetry.addData("Object Location", "left");
        }
        else {
            location = Location.RIGHT;
            telemetry.addData("Object Location", "right");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorobject = new Scalar(255, 0, 0);
        Scalar colorblank = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorblank:colorobject);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorblank:colorobject);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}