package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="OCV DET", group="TeleOp")
public class OpenCVDetection extends OpenCvPipeline {

    public static boolean DETECT_RED = true;
    public static double MINIMUM_VALUES = 100;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_HUE = 100;
    public static double MAXIMUM_BLUE_HUE = 115;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;

    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location{
        Left, Middle, Right
    }

    private Location location;

    static final Rect leftArea = new Rect(new Point(10,100), new Point(105,200));
    static final Rect middleArea = new Rect(new Point(120,100), new Point(205,200));
    static final Rect rightArea = new Rect(new Point(220,100), new Point(310,200));

//    public OpenCVDetection(Telemetry t) {
//        telemetry = t;
//    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE, MINIMUM_VALUES, MAXIMUM_VALUES);

        if (!DETECT_RED) {
            //BLUE
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat);
        }
        else {
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1, mat2, mat);
        }
        //submats
        Mat left = mat.submat(leftArea);
        Mat middle = mat.submat(middleArea);
        Mat right = mat.submat(rightArea);

        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(middle).val[0];
        double rightValue = Core.sumElems(right).val[0];

        telemetry.addData("Left Raw Value", leftValue);
        telemetry.addData("Middle Raw Value", middleValue);
        telemetry.addData("Right Raw Value", rightValue);

        left.release();
        middle.release();
        right.release();

        if (leftValue >= rightValue && leftValue >= middleValue) {
            location = Location.Left;
            telemetry.addData("Prop Location: ", "Right");
        } else if (rightValue >= middleValue) {
            location = Location.Right;
            telemetry.addData("Prop Location: ", "Left");
        } else {
            location = Location.Middle;
            telemetry.addData("Prop Location: ", "Middle");
        }

        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        //Ternary Operator - Do NOT mess with this logic.
        Imgproc.rectangle(mat, leftArea, location == Location.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, middleArea, location == Location.Middle? pixelColor:propColor);
        Imgproc.rectangle(mat, rightArea, location == Location.Right? pixelColor:propColor);

        return mat;

    }

    public Location getLocation()
    {
        return location;
    }

}
