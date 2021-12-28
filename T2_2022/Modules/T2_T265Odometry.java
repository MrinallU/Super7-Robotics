package org.firstinspires.ftc.teamcode.T2_2022.Modules;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;
import com.spartronics4915.lib.T265Helper;

import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Point;

public class T2_T265Odometry {
    // Motors
    public String outStr = "";

    // Variables
    private double xPos, yPos, angle;
    private HardwareMap hardwareMap;

    private double cX = 0, cY = 0, cAng = 0;
    private double xAdj = 0, yAdj = 0, angAdj = 0;
    private boolean positionAdjusted = false;


    Pose2d startPos;
    private T265Camera.CameraUpdate update;
    private static T265Camera slamara;

    // conf
    public T265Camera.PoseConfidence poseConfidence;

    public T2_T265Odometry(double xPos, double yPos, double angle, HardwareMap hardwareMap) {
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(xPos * 0.0254, yPos * 0.0254, angle);
    }

    public T2_T265Odometry(double angle, HardwareMap hardwareMap) {
        this.xPos = 0;
        this.yPos = 0;
        this.angle = angle;
        this.hardwareMap = hardwareMap;
        startPos =  new Pose2d(0 * 0.0254, 0 * 0.0254, angle);
    }

    public T2_T265Odometry(HardwareMap hardwareMap) {
        this(0, hardwareMap);
    }


    private void setCurrents(){
        cX = update.pose.getX();
        cY = update.pose.getY();
        cAng = Angle.normalize(Angle.radians_to_degrees(update.pose.getHeading()));
    }

    public void initializeT265(){
        slamara = T265Helper.getCamera(new T265Camera.OdometryInfo(startPos, 0), hardwareMap.appContext);
    }

    private void waitUpdate() {
        update = slamara.getLastReceivedCameraUpdate();

        if(update != null){
            setCurrents();

            xPos = cX;
            yPos = cY;
            angle = normalizeAngle(cAng);

            // Set string so values can be passed to telemetry
            outStr = "xPos: " + format(xPos) + "\nyPos: " + format(yPos) + "\nAngle: " + format(angle);
        }
    }


    public void startT265(){
        slamara.start();
    }

    public void stopT265(){
        slamara.stop();
//        slamara.free();
    }

    public void updatePosition() {
        update = slamara.getLastReceivedCameraUpdate();

        if(update != null){
            setCurrents();

            xPos = xAdj;
            yPos = yAdj;
            angle = angAdj;

            // get pose confidence
            poseConfidence = update.confidence;

            // Set string so values can be passed to telemetry
            outStr = "xPos: " + format(xPos) + "\nyPos: " + format(yPos) + "\nAngle: " + format(angle);
        }
    }


    public double normalizeAngle(double rawAngle) {
        double scaledAngle = rawAngle % 360;
        if ( scaledAngle < 0 ) {
            scaledAngle += 360;
        }

        if ( scaledAngle > 180 ) {
            scaledAngle -= 360;
        }

        return scaledAngle;
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

    public void resetOdometry(){
        resetOdometry(0, 0, 0);
    }

    public void resetOdometry(Point p){
        resetOdometry(p.xP, p.yP, p.ang);
    }

    public void resetOdometry(double xPos, double yPos, double angle){
        this.xPos = xPos;
        this.yPos = yPos;
        this.angle = angle;
    }

    public void setAngle(double angle){
        this.angle = angle;
    }

    private String format(double num){
        return String.format("%.3f", num);
    }
}
