
package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.fenum.qual.SwingCompassDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Container;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Intake;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Outtake;
import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_T265Odometry;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;

import java.util.List;

public abstract class T2_Base extends LinearOpMode
{
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
    public T2_Intake sweeper = null;
    public T2_Outtake arm = null;
    public T2_Container container;

    T2_T265Odometry odometry;

    public static final double carouselPow = 0.4;

    /* local OpMode members. */
    ElapsedTime matchTime = new ElapsedTime();

    // Normal moveToPosition PID Coefficients
    private final double k_p = 0.01;
    private final double k_d = 0.003;
    private final double k_i = 0;
    private final double max_i = 0.01;
    private double initAngle;

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
        container = new T2_Container(frontBlocker, sideBlocker);
        arm = new T2_Outtake(new Motor(hardwareMap, "arm", true), container, this,5.0);
        sweeper = new T2_Intake(new Motor(hardwareMap,  "sweeper"));

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

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backrightDrive.setPower(0);
        backleftDrive.setPower(0);

        initServos();

        // Odometry
        resetAngle();
        if(matchType == 1){
            initAngle = -179;
            odometry = new T2_T265Odometry(0, 0, -179, hardwareMap);
        }
        else{
            initAngle = 0;
            odometry = new T2_T265Odometry(0, 0, 0, hardwareMap);
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
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public double getAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle + initAngle;
    }

    public double[] scalePowers (double bLeftPow, double fLeftPow, double bRightPow, double fRightPow){
        double maxPow = Math.max(Math.max(Math.abs(fLeftPow), Math.abs(bLeftPow)), Math.max(Math.abs(fRightPow), Math.abs(bRightPow)));
        if (maxPow > 1) {
            fLeftPow /= maxPow;
            bLeftPow /= maxPow;
            fRightPow /= maxPow;
            bRightPow /= maxPow;
        }

        return new double[] {fLeftPow, bLeftPow, fRightPow, bRightPow};
    }


    public void driveFieldCentric(double baseAngle, double drive){
        double fRightPow, bRightPow, fLeftPow, bLeftPow;

        double bLeftAngle = Math.toRadians(baseAngle + 135);
        double fLeftAngle = Math.toRadians(baseAngle + 45);
        double bRightAngle = Math.toRadians(baseAngle + 225);
        double fRightAngle = Math.toRadians(baseAngle + 315);

        fRightPow = (drive * Math.sin(fRightAngle));
        bRightPow = (drive * Math.sin(bRightAngle));
        fLeftPow = (drive * Math.sin(fLeftAngle));
        bLeftPow = (drive * Math.sin(bLeftAngle));

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];


//        fLeftPow = Math.tanh(fLeftPow);
//        bLeftPow = Math.tanh(bLeftPow);
//        fRightPow = Math.tanh(fRightPow);
//        bRightPow = Math.tanh(bRightPow);


        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
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
            odometry.updatePosition();
            currTime = time.milliseconds() + 0.00001; // avoids divide by 0 error

            // error from input
            angleDiff = Angle.angleDifference(getAngle(), targetAngle);

            // update
            dAng = (angleDiff - prevAngleDiff) / (currTime - prevTime);
            iAng +=  (angleDiff * (currTime - prevTime));

            if(max_i < iAng){
                iAng = max_i;
            }else if(max_i * -1 > iAng){
                iAng = -max_i;
            }

            prevTime = currTime;
            prevAngleDiff = angleDiff;

            // 0.1 = f, tanh = makes the values approach 1 to -1
            double power = Range.clip(0.1 * Math.signum(angleDiff)
                    + 0.9 * Math.tanh(k_p * angleDiff + k_d * dAng), -powerCap, powerCap);

            setDrivePowers(-power, power, -power, power);

            // Teleop Breakout
            if(gamepad1.a && gamepad2.a){
                break;
            }
        }
        // stop when pos is reached
        stopBot();
    }

    public void turnToV2(double targetAngle, double timeout, double powerCap, double minDiff, LinearOpMode opMode)  {
        double angleDiff = 100, currTime = 0;
        double prevAngleDiff = 100;
        double dAng, iAng = 0;

        ElapsedTime time = new ElapsedTime(), cycleTime = new ElapsedTime();
        double prevTime = 0;

        while (time.milliseconds() < timeout && Math.abs(targetAngle - getAngle()) > minDiff && opMode.opModeIsActive())  {
            cycleTime.reset();
            resetCache();
            odometry.updatePosition();
            currTime = time.milliseconds() + 0.00001; // avoids divide by 0 error

            // error from input
            angleDiff = Angle.angleDifference(getAngle(), targetAngle);

            // update
            dAng = (angleDiff - prevAngleDiff) / (currTime - prevTime);
            iAng +=  (angleDiff * (currTime - prevTime));

            if(max_i < iAng){
                iAng = max_i;
            }else if(max_i * -1 > iAng){
                iAng = -max_i;
            }

            prevTime = currTime;
            prevAngleDiff = angleDiff;

            // 0.1 = f, tanh = makes the values approach 1 to -1
            double power = Range.clip(0.1 * Math.signum(angleDiff)
                    + 0.9 * Math.tanh(k_p * angleDiff + k_d * dAng), -powerCap, powerCap);

            setDrivePowers(-power, power, -power, power);

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
        odometry.updatePosition();
        double currX = odometry.getX(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(currX - targetX) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            currX = odometry.getX();
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
        }
        stopBot();
    }

    public void yTo(double targetY, double timeout, double powerCap, double minDifference, LinearOpMode opMode, boolean negate){
        odometry.updatePosition();
        double Y = odometry.getY(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(Y - targetY) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            Y = odometry.getY();
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
        }
        stopBot();
    }

    public void xTo(double targetX, double timeout, double powerCap, double minDifference, LinearOpMode opMode){
        odometry.updatePosition();
        double currX = odometry.getX(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(currX - targetX) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            currX = odometry.getX();
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

            driveFieldCentric(getAngle(), drive) ;
        }
        stopBot();
    }

    public void yTo(double targetY, double timeout, double powerCap, double minDifference, LinearOpMode opMode){
        odometry.updatePosition();
        double Y = odometry.getY(); // replace as needed
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(Y - targetY) > minDifference && time.milliseconds() < timeout && opMode.opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            Y = odometry.getY();
            double yDiff = targetY - Y;
            double drive = Range.clip(yDiff * 0.055, -powerCap, powerCap) * -1;

            driveFieldCentric(getAngle(), drive) ;
        }
        stopBot();
    }


    public double getAutoAimAngle(Point p){
        double xDiff = p.xP - odometry.getX();
        double yDiff = p.yP - odometry.getY();
        return Math.toDegrees(Math.atan2(yDiff, xDiff));
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

