
package org.firstinspires.ftc.teamcode.T3_2022.Modules;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;
import com.spartronics4915.lib.T265Localizer;

import org.firstinspires.ftc.teamcode.Utils.Angle;

public class T3_T265Odometry {
    // Motors
    public String outStr = "";

    // Variables
    private double xPos, yPos, angle;
    private HardwareMap hardwareMap;

    private double cX = 0, cY = 0, cAng = 0;

    Pose2d startPos;
    private T265Camera.CameraUpdate update;
    private static T265Camera slamara;
    private static T265Localizer localizer;

    // conf
    public T265Camera.PoseConfidence poseConfidence;

    public T3_T265Odometry(double xPos, double yPos, double angle, HardwareMap hardwareMap) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(xPos / 39.37, yPos / 39.37, Angle.degrees_to_radians(angle));
    }

    public T3_T265Odometry(double angle, HardwareMap hardwareMap) {
        this.xPos = 0;
        this.yPos = 0;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(0, 0, Angle.degrees_to_radians(angle));
    }

    public T3_T265Odometry(HardwareMap hardwareMap) {
        this(0, hardwareMap);
    }

    public void updatePosition() {
        update = slamara.getLastReceivedCameraUpdate();

        if(update != null){
            xPos = update.pose.getX() / 0.0254;
            yPos = update.pose.getY() / 0.0254;
            angle = Angle.normalize(Angle.radians_to_degrees(update.pose.getHeading()));

            // get pose confidence
            poseConfidence = update.confidence;

            // Set string so values can be passed to telemetry
            outStr = "xPos: " + format(xPos) + "\nyPos: " + format(yPos) + "\nAngle: " + format(angle);
        }
    }

    public void initializeT265(){
        slamara = T265Helper.getCamera(new T265Camera.OdometryInfo(startPos, 1), hardwareMap.appContext);
        slamara.start();
    }

    public void stopT265(){
        T265Helper.destroyCamera();
    }

    public String displayPositions() {
        return outStr;
    }

    public double getAngle() {
        return angle;
    }

    public double getX() {
        return xPos;
    }

    public double getY() {
        return yPos;
    }


    public void setAngle(double angle){
        this.angle = angle;
    }

    private String format(double num){
        return String.format("%.3f", num);
    }
}
