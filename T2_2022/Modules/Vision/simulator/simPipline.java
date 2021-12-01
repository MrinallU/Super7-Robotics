package org.firstinspires.ftc.teamcode.T2_2022.Modules.Vision.simulator;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

/**
 * Note that BGR is consistent across all images, not just the ones covered by the phone cam
 * TODO: Create method that scans rectangle bounds and returns the percentage of pixels activated
 */


public class simPipline extends OpenCvPipeline {
            Telemetry telemetry;
            Mat image = new Mat();
            Mat mask = new Mat();
            Mat result = new Mat();

            public simPipline(Telemetry telemetry){
                this.telemetry = telemetry;
            }

            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input,image, Imgproc.COLOR_BGR2HSV);
                Core.inRange(image, new Scalar(102 , 90, 63), new Scalar(112 , 255, 255), mask);

                Core.bitwise_and(input, input, result, mask);
                getElementPresence(new Point(124,153));
                return mask;
            }

            public boolean getElementPresence(Point l){
                return Arrays.toString(mask.get((int) l.xP, (int) l.yP)).equals("[255.0]");
            }
}



class Point implements Comparable<Point> {
    public double xP, yP, ang;

    public Point(double xP, double yP, double ang){
        this.xP = xP;
        this.yP = yP;
        this.ang = ang;
    }

    public Point(double xP, double yP){
        this.xP = xP;
        this.yP = yP;
    }


    public void setX(double xP){
        this.xP = xP;
    }

    public void setY(double yP){
        this.yP = yP;
    }

    // Returns the distance from this point to any other point specified
    public double getDistance(Point p2){
        return Math.sqrt((p2.yP - this.yP) * (p2.yP - this.yP) + (p2.xP - this.xP) * (p2.xP - this.xP));
    }

    @Override
    public int compareTo(Point o) {
        if (this.xP < o.xP) return -1;
        if (this.xP == o.xP) return 0;
        return 1;
    }
}
