package org.firstinspires.ftc.teamcode.T4_2022;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Motor;
import org.firstinspires.ftc.teamcode.Utils.Point;
import org.firstinspires.ftc.teamcode.Utils.SplineGenerator;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.List;
import java.util.Locale;

public abstract class T3_Base extends LinearOpMode {
    // Lynx ModulesR
    List<LynxModule> allHubs;

    // Odometry
    T3_T265Odometry odometry;
    SplineGenerator splineGenerator = new SplineGenerator();

    // Motors
    protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;
    public Motor  leftDrive   = null;
    public Motor  rightDrive  = null;
    public Motor  backleftDrive   = null;
    public Motor  backrightDrive  = null;
    public Motor carousel = null;

    Servo frontBlocker, sideBlocker;

    public T3_Intake sweeper = null;
    public T3_Outtake arm = null;
    public T3_Container container;

    public static final double carouselPow = 0.4;

    // Testing Mode
    boolean testing = false;

    // Sleep Times
    ElapsedTime matchTime = new ElapsedTime();
    ElapsedTime pollTime, cTime;
    int testingSleep = 200, normalSleep = 10;

    // Gyro and Angles
    public BNO055IMU gyro;
    double startAngle;

    // Constants and Conversions
    protected double dpadTurnSpeed = 0.175, dpadDriveSpeed = 0.2;
    protected double autoAngleToPower = 0.01;
    protected double powerMult = 0;


    // Powers


    // Other
    boolean isRunning;

    public enum sweepStateOptions {
        FORWARD,
        REVERSE,
        STOP
    }

    public enum carsouselStateOptions {
        SPIN,
        STOP
    }

    public enum armStateOptions {
        HIGH,
        MIDDLE,
        LOW,
        SWEEP,
        REST
    }

    public void initHardware(int type){
        // Setting up Hubs (Manual Caching)
        allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Motors
        fLeftMotor = new Motor(hardwareMap, "fLeft");
        bLeftMotor = new Motor(hardwareMap, "bLeft");
        fRightMotor = new Motor(hardwareMap, "fRight");
        bRightMotor = new Motor(hardwareMap, "bRight");


        carousel = new Motor(hardwareMap, "carousel");

        frontBlocker = hardwareMap.servo.get("frontBlocker");
        sideBlocker = hardwareMap.servo.get("sideBlocker");
        container = new T3_Container(frontBlocker, sideBlocker);

        arm = new T3_Outtake(new Motor(hardwareMap, "arm"), container, this,5.0);
        sweeper = new T3_Intake(new Motor(hardwareMap,  "sweeper"));
        // Servos


//         Odometry
//         0 is Auto, 1 is TeleOp
        if(type == 0){
            odometry = new T3_T265Odometry(0, 19.9, -90, hardwareMap);
        }
        else{
            odometry = new T3_T265Odometry(0, 0, 0, hardwareMap);
        }


        // Misc
        isRunning = true;

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);


        odometry.initializeT265();
    }

    public void initServos(){
        frontBlocker.setPosition(container.frontBlockPos);
        sideBlocker.setPosition(container.sideBlockPos);
    }

    public void spinCarousel(){
        carousel.setPower(0.7);
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public double getAngle(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //ZYX is Original
        return angles.firstAngle;
    }

    // MULTI-USE FUNCTIONS
    public void driveFieldCentric(double baseAngle, double drive, double turn, double strafe){
        double fRightPow, bRightPow, fLeftPow, bLeftPow;

        double bLeftAngle = Math.toRadians(baseAngle + 135);
        double fLeftAngle = Math.toRadians(baseAngle + 45);
        double bRightAngle = Math.toRadians(baseAngle + 225);
        double fRightAngle = Math.toRadians(baseAngle + 315);

        fRightPow = (drive * Math.sin(fRightAngle) + strafe * Math.cos(fRightAngle)) / Math.sqrt(1) + turn;
        bRightPow = (drive * Math.sin(bRightAngle) + strafe * Math.cos(bRightAngle)) / Math.sqrt(1) + turn;
        fLeftPow = (drive * Math.sin(fLeftAngle) + strafe * Math.cos(fLeftAngle)) / Math.sqrt(1) + turn;
        bLeftPow = (drive * Math.sin(bLeftAngle) + strafe * Math.cos(bLeftAngle)) / Math.sqrt(1) + turn;

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];

        setDrivePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
    }

    public void driveRobotCentric(double drive, double turn, double strafe){
        double fRightPow = 0, bRightPow = 0, fLeftPow = 0, bLeftPow = 0;

        fLeftPow = -drive + turn - strafe;
        bLeftPow = -drive + turn + strafe;
        fRightPow = drive + turn - strafe;
        bRightPow = drive + turn + strafe;

        double[] calculatedPower = scalePowers(bLeftPow, fLeftPow, bRightPow, fRightPow);
        fLeftPow = calculatedPower[0];
        bLeftPow = calculatedPower[1];
        fRightPow = calculatedPower[2];
        bRightPow = calculatedPower[3];

        setDrivePowers(bLeftPow, fLeftPow, bRightPow,fRightPow);
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

    public void setDrivePowers(double bLeftPow, double fLeftPow, double bRightPow, double fRightPow){
        bLeftMotor.setPower(bLeftPow);
        fLeftMotor.setPower(fLeftPow);
        bRightMotor.setPower(bRightPow);
        fRightMotor.setPower(fRightPow);
    }

    // AUTONOMOUS/TESTING FUNCTIONS
    public void rest(){
        rest(testing ? testingSleep : normalSleep);
    }

    public void rest(int time){
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


    public void turnTo(double targetAngle, long timeout, double powerCap){ turnTo(targetAngle, timeout, powerCap, 2); }

    public void turnTo(double targetAngle, long timeout){
        turnTo(targetAngle, timeout, 0.7);
    }

    public void turnTo(double targetAngle, long timeout, double powerCap, double minDifference){
        double currAngle = T3_Readings.angle;
        ElapsedTime time = new ElapsedTime();
        while(Math.abs(currAngle - targetAngle) > minDifference && time.milliseconds() < timeout && isRunning){
            resetCache();
            setReadings();

            currAngle = T3_Readings.angle;
            double angleDiff = Angle.normalize(currAngle - targetAngle);
            double calcP = Range.clip(angleDiff * autoAngleToPower, -powerCap, powerCap);
            setDrivePowers(calcP, calcP, calcP, calcP);
        }

        setDrivePowers(0, 0, 0, 0);
    }

    // auto-aim
    public void aimTo(Point destPos){
        double xDiff = destPos.xP - odometry.getX(); double yDiff = destPos.yP - odometry.getY();
        double angle = odometry.normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)));
        turnTo(angle, 6000);
    }

//    public void followSplinePath(Point [] p) {
//        BigDecimal[] coefficients = splineGenerator.generateSplinePath(p);
//
//        for (int i = 0; i < coefficients.length; i += 4) {
//            for (double x = p[i / 4].xP; x <= p[(i / 4) + 1].xP; x += 5) {
//                BigDecimal a = coefficients[i].multiply(BigDecimal.valueOf(x).pow(3, MathContext.DECIMAL64));
//                BigDecimal b = coefficients[i + 1].multiply(BigDecimal.valueOf(x).pow(2, MathContext.DECIMAL64));
//                BigDecimal c = coefficients[i + 2].multiply(BigDecimal.valueOf(x));
//                BigDecimal d = coefficients[i + 3];
//                double y = (a.add(b).add(c).add(d)).doubleValue();
//
//                // odometry.x, y, and angle are robot's current values
//                double xDiff = x - odometry.getX(); double yDiff = y - odometry.getY();
//                double angle = odometry.normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)));
//
//                // todo: Pass in the next point and set drive powers at the end of the loop such that it starts traversing there
//                // todo: Set at power min so the spline doesn't slow down too much when traversing.
//                splineMove(x, y, angle, 2, 2, 2, 4000, this);
//            }
//
//            // Just in case the final pos is a little bit off
//            double destX = p[(i / 4) + 1].xP;
//            BigDecimal a = coefficients[i].multiply(BigDecimal.valueOf(destX).pow(3, MathContext.DECIMAL64));
//            BigDecimal b = coefficients[i + 1].multiply(BigDecimal.valueOf(destX).pow(2, MathContext.DECIMAL64));
//            BigDecimal c = coefficients[i + 2].multiply(BigDecimal.valueOf(destX));
//            BigDecimal d = coefficients[i + 3];
//
//            double destY = (a.add(b).add(c).add(d)).doubleValue();
//            double xDiff = destX - odometry.getX(); double yDiff = destY - odometry.getY();
//            double destAngle = odometry.normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)));
//            splineMove(p[(i / 4) + 1].xP, destY, destAngle, 2, 2, 2, 4000, this);
//        }
//        setDrivePowers(0, 0, 0, 0);
//    }
//


    // ODOMETRY FUNCTIONS
    private final double distanceToPowerScale = 0.03;
    private final double angleToPowerScale = 0.01;



    private final double max_i = 0.01;

    // Spline PID Coefficients
    private final double k_p_s = 0.075;
    private final double k_d_s = 0;
    private final double k_i_s = 0;

    // Normal moveToPosition PID Coefficients
    private final double k_p = 0.055;
    private final double k_d = 0;
    private final double k_i = 0;


    // No stop PID
    public void splineMove(double targetXPos, double targetYPos, double targetAngle, double xAccuracy, double yAccuracy, double angleAccuracy, double timeout, OpMode opMode){
        double xDiff = 100, yDiff = 100, angleDiff = 100, currTime = 0;
        double prevXDiff = 100, prevYDiff = 100;
        double pX = 0, pY = 0, dX, dY, iX = 0, iY = 0;

        ElapsedTime time = new ElapsedTime();
        double prevTime = 0;

        while (time.milliseconds() < timeout && (Math.abs(xDiff) > xAccuracy || Math.abs(yDiff) > yAccuracy || Math.abs(angleDiff) > angleAccuracy) && ((LinearOpMode)opMode).opModeIsActive())  {
            // timers
            resetCache();
            setReadings();
            odometry.updatePosition();
            currTime = time.milliseconds() + 0.001; // avoids divide by 0 error

            // error from input
            xDiff = targetXPos - odometry.getX();
            yDiff = targetYPos - odometry.getY();
            angleDiff = odometry.normalizeAngle(-targetAngle + odometry.getAngle());

            //
            if(xDiff > xAccuracy){
                pX = 0.2;
            }else if(xDiff < xAccuracy){
                pX = -0.2;
            }

            if(yDiff > yAccuracy){
                pY = 0.2;
            }else if(yDiff < yAccuracy){
                pY = -0.2;
            }

            // feeding error into motors
//            driveRobotCentric(pY + dY + iY, angleToPower(angleDiff), pX + dX + iX);
            driveFieldCentric(odometry.getAngle(), pY, angleToPower(angleDiff), pX);

            // prepare for next iteration
            prevTime = currTime;
            prevXDiff = xDiff;
            prevYDiff = yDiff;

            // Teleop Breakout
            if(gamepad1.a && gamepad2.a){
                break;
            }
        }
    }


    public void moveToPosition(double targetXPos, double targetYPos, double targetAngle, double xAccuracy, double yAccuracy, double angleAccuracy, double timeout, OpMode opMode){
        double xDiff = 100, yDiff = 100, angleDiff = 100, currTime = 0;
        double prevXDiff = 100, prevYDiff = 100;
        double pX, pY, dX, dY, iX = 0, iY = 0;

        ElapsedTime time = new ElapsedTime(), cycleTime = new ElapsedTime();
        double prevTime = 0;

        while (time.milliseconds() < timeout && (Math.abs(xDiff) > xAccuracy || Math.abs(yDiff) > yAccuracy || Math.abs(angleDiff) > angleAccuracy) && ((LinearOpMode)opMode).opModeIsActive())  {
            // timers
            cycleTime.reset();
            resetCache();
            setReadings();
            odometry.updatePosition();
            currTime = time.milliseconds() + 0.00001; // avoids divide by 0 error

            // error from input
            xDiff = targetXPos - odometry.getX();
            yDiff = targetYPos - odometry.getY();
            angleDiff = odometry.normalizeAngle(-targetAngle + odometry.getAngle());

            // updating pid components
            pX = k_p * xDiff;
            pY = k_p * yDiff;

            dX =  k_d * (xDiff - prevXDiff) / (currTime - prevTime);
            dY =  k_d * (yDiff - prevYDiff) / (currTime - prevTime);

            iX += k_i * (xDiff * (currTime - prevTime));
            iY += k_i * (yDiff * (currTime - prevTime));

            if(max_i < iX){
                iX = max_i;
            }else if(max_i * -1 > iX){
                iX = -max_i;
            }

            if(max_i < iY){
                iY = max_i;
            }else if(max_i * -1 > iY){
                iY = -max_i;
            }

            // feeding error into motors
            // driveRobotCentric(pY + dY + iY, angleToPower(angleDiff), pX + dX + iX);
            driveFieldCentric(odometry.getAngle(),pY + dY + iY, angleToPower(angleDiff), pX + dX + iX);

            prevTime = currTime;
            prevXDiff = xDiff;
            prevYDiff = yDiff;

            // Teleop Breakout
            if(gamepad1.a && gamepad2.a){
                break;
            }
        }
        // stop when pos is reached
        driveRobotCentric(0, 0, 0);
    }

    public void moveToPosition(double targetXPos, double targetYPos, double targetAngle, double posAccuracy, double angleAccuracy, double timeout, OpMode opMode) {
        moveToPosition(targetXPos, targetYPos, targetAngle, posAccuracy, posAccuracy, angleAccuracy, timeout, opMode);
    }

    public void moveToPosition(double targetXPos, double targetYPos, double targetAngle, double timeout, OpMode opMode) {
        moveToPosition(targetXPos, targetYPos, targetAngle, 1, 2, timeout, opMode);
    }

    public void moveToPosition(double targetXPos, double targetYPos, double targetAngle, double posAccuracy, double timeout, OpMode opMode) {
        moveToPosition(targetXPos, targetYPos, targetAngle, posAccuracy, 2, timeout, opMode);
    }

    // Function implementing Points
    public void moveToPosition(Point p, double xAccuracy, double yAccuracy, double angleAccuracy, double timeout, OpMode opMode){
        moveToPosition(p.xP, p.yP, p.ang, xAccuracy, yAccuracy, angleAccuracy, timeout, opMode);
    }

    public void moveToPosition(Point p, double posAccuracy, double angleAccuracy, double timeout, OpMode opMode) {
        moveToPosition(p.xP, p.yP, p.ang, posAccuracy, posAccuracy, angleAccuracy, timeout, opMode);
    }

    public void moveToPosition(Point p, double timeout, OpMode opMode) {
        moveToPosition(p.xP, p.yP, p.ang, 1, 2, timeout, opMode);
    }

    public void moveToPosition(Point p, double posAccuracy, double timeout, OpMode opMode) {
        moveToPosition(p.xP, p.yP, p.ang, posAccuracy, 2, timeout, opMode);
    }

    public void autoAimToWobble(OpMode opMode){
        double xDiff = T3_OdometryConstants.goal.xP - odometry.getX();
        double yDiff = T3_OdometryConstants.goal.yP - odometry.getY();

        double ang = Math.toDegrees(Math.atan2(yDiff, xDiff));

        moveToPosition(odometry.getX(), odometry.getY(), ang, 2, 1, 2000, opMode);
    }

    public double getAutoAimAngle(Point p){
        double xDiff = p.xP - odometry.getX();
        double yDiff = p.yP - odometry.getY();
        return Math.toDegrees(Math.atan2(yDiff, xDiff));
    }

    public void autoPositionToWobble(OpMode opMode){
        // TODO: Switch to Calculus Optimization; make use of the derivative and constraint point to find in O(1)
        double x = odometry.getX();
        double minDist = Double.MAX_VALUE;
        double testY;

        Point robotPose = new Point(odometry.getX(), odometry.getY(), odometry.getAngle());
        Point closestPoint = new Point(0, 0, 0);

        for (int i = 10; i <= 20; i++) { // find the limits of the circle... 46 inches to the y, 57.5 to the x

            // test optimization along the circle (x-46)^2 + (y-57.5)^2 = 841
            for (int j = 0; j < 2; j++) {
                double testPoint = Math.pow(i * -1, 2) + (92 * i) - 1275;
                if(j == 0){
                    testY = -1 * Math.sqrt(testPoint) + 57.5;
                }else{
                    testY = Math.sqrt(testPoint) + 57.5;
                }

                Point testP = new Point(x, testY, 0);
                double currDist = testP.getDistance(robotPose);
                if(minDist > currDist){
                    minDist =  currDist;
                    closestPoint = new Point(i, testY);
                }
            }
        }

        closestPoint.ang = getAutoAimAngle(closestPoint);
        moveToPosition(closestPoint, 2, 2, 2, 3000, opMode);
    }

    public void autoPositionToWobblev2(OpMode opMode){
        double x = odometry.getX();
        double y = odometry.getY();
        double r = 29; // 29 inch wobble radius

    }

    public double distanceToPower(double dist) {
        return dist * distanceToPowerScale;
    }

    public double angleToPower(double angle) {
        return angle * angleToPowerScale;
    }

    // BULK-READING FUNCTIONS
    public void resetCache(){
        // Clears cache of all hubs
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }

    public void setReadings(){
        // Function Calls
        setVoltage();

        // Direct Storage
        T3_Readings.angle = getAngle();
        T3_Readings.x = odometry.getX();
        T3_Readings.y = odometry.getY();
    }

    private void setVoltage(){
        double voltage = Double.MIN_VALUE;
        for(LynxModule hub : allHubs){
            voltage = Math.max(voltage, hub.getInputVoltage(VoltageUnit.VOLTS));
        }

        T3_Readings.voltage = voltage;
    }

    public void adjustToPower(){
        double high = 14, low = 12;
        powerMult = Range.clip(1 - ((T3_Readings.voltage - low) / (high - low)), 0, 1);
    }

    private double scale(double highPow, double lowPow){
        return lowPow + (highPow - lowPow) * powerMult;
    }

    // Other Functions
    public double normalizeThreeDigits(double d){
        return (int)(d * 1000) / 1000.;
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;

}
