
package org.firstinspires.ftc.teamcode.T3_2022;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Container;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Intake;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Outtake;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_T265Odometry;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Container;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_DifferentialDriveOdometry;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Intake;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Outtake;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_T265Odometry;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;
import org.firstinspires.ftc.teamcode.Utils.SplineGenerator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public abstract class T3_Base extends LinearOpMode {
    List<LynxModule> allHubs;
    public Motor  leftDrive   = null;
    public Motor  rightDrive  = null;
    public Motor  backleftDrive   = null;
    public Motor  backrightDrive  = null;
    public Motor carousel = null;
    Servo frontBlocker, sideBlocker;
    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    double currAngle = 0;
    public T3_Intake sweeper = null;
    public T3_Outtake arm = null;
    public T3_Container container;

    public T3_T265Odometry odometry;
    public T3_DifferentialDriveOdometry wheelOdometry;

    public static final double carouselPow = 0.4;

    /* local OpMode members. */
    ElapsedTime matchTime = new ElapsedTime();

    // Normal moveToPosition PID Coefficients
    private final double k_p = 0.01;
    private final double k_d = 0.003;
    private final double k_i = 0;
    private final double max_i = 0.01;
    private double initAngle;
    private SplineGenerator splineGenerator = new SplineGenerator();

    /* Initialize standard Hardware interfaces */
    public void init(int matchType) {
        // Save reference to Hardware map
        allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Motors
        leftDrive  = new Motor(hardwareMap, "fLeft");
        rightDrive = new Motor(hardwareMap, "fRight");
        backleftDrive  = new Motor(hardwareMap,  "bLeft");
        backrightDrive = new Motor(hardwareMap,  "bRight");
        carousel = new Motor(hardwareMap, "carousel");

        // Servos
        frontBlocker = hardwareMap.servo.get("frontBlocker");
        sideBlocker = hardwareMap.servo.get("sideBlocker");

        //Modules
        container = new T3_Container(frontBlocker, sideBlocker);
        arm = new T3_Outtake(new Motor(hardwareMap, "arm", true), container, this,5.0);
        sweeper = new T3_Intake(new Motor(hardwareMap,  "sweeper"));

        //IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start Motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors'
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        backleftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        stopBot();

        initServos();

        // Odometry
        resetAngle();
        if(matchType == 1){
            initAngle = -179;
            odometry = new T3_T265Odometry(-179, hardwareMap);
            wheelOdometry = new T3_DifferentialDriveOdometry(0, 0, -179);
        }
        else{
            initAngle = 0;
            odometry = new T3_T265Odometry(0, hardwareMap);
            wheelOdometry = new T3_DifferentialDriveOdometry(0, 0, 0);
        }
    }

    public void initServos(){
        container.init();
    }
    public void initServosAuto(){
        container.initAuto();
    }

    public void initOdometry() {
        odometry.initializeT265();
    }

    public void resetCache(){
        // Clears cache of all hubs
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }

    private void stopBot() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
    }

    private void setDrivePowers(double v, double motorPower, double v1, double motorPower1) {
        leftDrive.setPower(v);
        backleftDrive.setPower(v1);
        rightDrive.setPower(motorPower);
        backrightDrive.setPower(motorPower1);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getRelativeAngle() {

        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        return currAngle;
    }

    public double getAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle + initAngle;
    }

    // my imu based turnTo
    public void turnToV2(double targetAngle, double timeout, double powerCap, LinearOpMode opMode)  {
        double angleDiff = 100, currTime = 0;
        double prevAngleDiff = 100;
        double dAng, iAng = 0;

        ElapsedTime time = new ElapsedTime(), cycleTime = new ElapsedTime();
        double prevTime = 0;

        while (time.milliseconds() < timeout && Math.abs(targetAngle - getAngle()) > 1 && opMode.opModeIsActive())  {
            cycleTime.reset();
            resetCache();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            currTime = time.milliseconds() + 0.00001; // avoids divide by 0 error

            // error from input
            angleDiff = Angle.angleDifference(getAngle(), targetAngle);

            // 0.1 = f, tanh = makes the values approach 1 to -1
            double power = Math.tanh(k_p * angleDiff);

            setDrivePowers(-power, power, -power, power);
            telemetry.addLine(wheelOdometry.displayPositions());
            telemetry.update();

            // Teleop Breakout
            if(gamepad1.a && gamepad2.a){
                break;
            }
        }
        // stop when pos is reached
        stopBot();
    }

    public void turnToV2(double targetAngle, double timeout, LinearOpMode opMode){ turnToV2(targetAngle, timeout, 1, opMode); }

    public void xTo(double targetX, double timeout, double powerCap, double minDifference, LinearOpMode opMode, boolean negate){
        double currX = wheelOdometry.getX(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(currX - targetX) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            currX = wheelOdometry.getX();
            double xDiff = currX - targetX;

            // front is negative
            // back positive
            // 2 - 0 = 2 but cam is front so negate to go back to zero
            /*
            Back robot is not negated
            Front cam is negated
            Left cam is not negated
            Right cam is negated
             */

            double drive = Range.clip(xDiff * 0.055, -powerCap, powerCap);
            if(negate) {
                drive *= -1;
            }

            setDrivePowers(drive, drive, drive, drive);
            telemetry.addLine(wheelOdometry.displayPositions());
            telemetry.update();
        }
        stopBot();
    }

    public boolean yTo(double targetY, double timeout, double powerCap, double minDifference, LinearOpMode opMode, boolean negate){
        double Y = wheelOdometry.getY(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(Y - targetY) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            Y = wheelOdometry.getY();
            double yDiff = targetY - Y;
            double drive = Range.clip(yDiff * 0.055, -powerCap, powerCap) * -1;

            if(negate){
                drive *= -1;
            }
            // Combine drive and turn for blended motion.

            // Output the safe vales to the motor drives.
            leftDrive.setPower(drive);
            rightDrive.setPower(drive);
            backleftDrive.setPower(drive);
            backrightDrive.setPower(drive);

            telemetry.addLine(wheelOdometry.displayPositions());
            telemetry.update();
        }
        stopBot();

        // works
        return time.milliseconds() < timeout;

    }

    // tick diff should be no less than 22!
    // tick per in is 44.8
    public void moveTicks(double ticksMoved, double timeout, double powerCap, double minDifference, LinearOpMode opMode, boolean negate){
        // update odometry convert tick velocity to inch velocity
        wheelOdometry.updatePosition(
                leftDrive.encoderReading(),
                rightDrive.encoderReading(),
                getAngle());

        double currTicks = leftDrive.encoderReading(); // todo: average all the values?? (further research required)
        double destTick;
        ElapsedTime time = new ElapsedTime();

        if(negate) {
             destTick = currTicks - ticksMoved;
        }else{
             destTick = currTicks + ticksMoved;
        }

        telemetry.addLine("pos " + leftDrive.encoderReading() + " " + (destTick));
        telemetry.update();

        while(Math.abs(currTicks - (destTick)) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            // update odometry convert tick velocity to inch velocity
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            currTicks = leftDrive.encoderReading();

            double tickDiff = destTick - currTicks;
            double drive = Range.clip(tickDiff * 0.055, -powerCap, powerCap); // p-controller

            setDrivePowers(drive, drive, drive, drive);
            odometry.updatePosition();

            telemetry.addLine(wheelOdometry.displayPositions());
            telemetry.update();
        }
        stopBot();
    }

    public void moveTicksFront(double ticksMoved, double timeout, double powerCap, double minDifference, LinearOpMode opMode){
        moveTicks(ticksMoved, timeout, powerCap, minDifference, opMode, false);
    }

    public void moveTicksBack(double ticksMoved, double timeout, double powerCap, double minDifference, LinearOpMode opMode){
        moveTicks(ticksMoved, timeout, powerCap, minDifference, opMode, true);
    }

    public double getAutoAimAngle(Point p){
        double xDiff = p.xP - odometry.getX();
        double yDiff = p.yP - odometry.getY();
        return Math.toDegrees(Math.atan2(yDiff, xDiff));
    }

    // For now splines are processed in terms of increasing x, so knot splines are not possible
    // todo add a d-component to the spline
    public void traverseSpline(Point [] pts, double driveSpeedCap, double xError, int lookAheadDist, boolean reverse) {
        pts[0] = new Point(wheelOdometry.getX(), wheelOdometry.getY());
        Arrays.sort(pts);
        ArrayList<Point> wp = splineGenerator.generateSplinePath(pts, lookAheadDist); // get weighpoints
        sleep(500);
        // back to front
        if(reverse) {
            Collections.reverse(wp);
        }

        while (Math.abs(wheelOdometry.getX() - pts[pts.length - 1].xP) > xError) {
            resetCache();
            // update localizer
            wheelOdometry.updatePosition(
                    leftDrive.encoderReading(),
                    rightDrive.encoderReading(),
                    getAngle());

            // find point which fits the look ahead criteria
            Point nxtP = null;
            double fitDist = Double.MAX_VALUE;
            for (Point p : wp
            ) {
                double ptDist = wheelOdometry.getPose().getDistance(p);
                // make sure we arent going backwards in the spline... for now...
                if (wheelOdometry.getX() < p.xP) {      // find the closest point that is in front of the robot
                    if (ptDist < fitDist) {
                        fitDist = ptDist;
                        nxtP = p;
                    }
                }
            }

            if (nxtP == null) {
                stopBot();
                sleep(500);
                break;
            }

            // assign powers to follow the look-ahead point
            double yDiff = nxtP.yP - wheelOdometry.getY();
            double xDiff = nxtP.xP - wheelOdometry.getX();
            double angDiff = Angle.angleDifference( Angle.normalize(getAngle()) , // get steering angle
                    Angle.normalize( Math.toDegrees(
                            Math.atan2(yDiff, xDiff)
                            )
                    ));

            // P-Control for drive and turn
            if(Math.abs(angDiff) < 2)
                angDiff = 0;

            double turnSpeed = angDiff * 0.01; // Basic P-Control
            double driveSpeed = Range.clip((pts[pts.length - 1].xP - wheelOdometry.getX() * 0.055),
                    -driveSpeedCap, driveSpeedCap);

            setDrivePowers(driveSpeed - turnSpeed,
                    driveSpeed + turnSpeed,
                    driveSpeed - turnSpeed,
                    driveSpeed + turnSpeed);

            telemetry.addLine(wheelOdometry.displayPositions());
            telemetry.update();
        }
        stopBot();
    }


    public void autoAimToWobble(String opMode){
        double targetAng;

        if(opMode.equals("redPrimary"))
            targetAng = getAutoAimAngle(new Point(0, 1));
        else if(opMode.equals("redSecondary"))
            targetAng = getAutoAimAngle(new Point(0, 1));
        else if(opMode.equals("bluePrimary"))
            targetAng = getAutoAimAngle(new Point(0, 1));
        else
            targetAng = getAutoAimAngle(new Point(0, 1));

        turnToV2(targetAng, 6000, this);
    }


    public void startCarousel(){
        carousel.setPower(0.4);
    }
    public void startBlueCarousel(){
        carousel.setPower(-0.4);
    }
    public void stopCarousel(){
        carousel.setPower(0);
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;
}

