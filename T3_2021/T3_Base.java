package org.firstinspires.ftc.teamcode.T3_2021;

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

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public abstract class T3_Base extends LinearOpMode {
    // Lynx Modules
    List<LynxModule> allHubs;

    // Odometry
    T3_T265Odometry odometry;

    // Motors
    protected Motor fLeftMotor, bLeftMotor, fRightMotor, bRightMotor;
    protected Motor shooter1, shooter2, sweeper1, sweeper2;

    protected T3_MotorPair shooter, sweeper;

    // Servos
    protected Servo wobbleServo1, wobbleServo2, ringRotator, kicker, shooterAdjustor, wobbleGoalGrabber, stackServo;

    protected T3_WobbleArm wobbleArm;

    // Servo Positions
    double ringRotatorDown = 0, ringRotatorUp = 0.75;
    double kickerEngPos = 0.375, kickerOpenPos = 0.7;
    double shootAdjUp = 0.51, shootAdjDown = 0.7, shootAdjAuto = 0.545;
    double wobbleGoalGrabberClose = 0.05, wobbleGoalGrabberOpen = 0.5;
    double[] stackServoPositions = {0, 0.3, 0.7};
    int stackServoPos = 2;

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
    protected double kickerCycleTime = 420;

    // Powers
    protected double setShooterPow = 1;
    protected double minSweepPow = 0.8, maxSweepPow = 1, reverseSweeperPow = -0.7;

    // Other
    boolean isRunning;

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

        shooter1 = new Motor(hardwareMap, "shooter1", true);
        shooter2 = new Motor(hardwareMap, "shooter2", true);
        shooter = new T3_MotorPair(shooter1, shooter2);


        sweeper1 = new Motor(hardwareMap, "sweeper1", false);
        sweeper2 = new Motor(hardwareMap, "sweeper2", false);
        sweeper = new T3_MotorPair(sweeper1, sweeper2);

        // Servos
        kicker = hardwareMap.servo.get("kicker");
        shooterAdjustor = hardwareMap.servo.get("shooterAdjustor");
        ringRotator = hardwareMap.servo.get("ringRotator");
        wobbleGoalGrabber = hardwareMap.servo.get("wobbleGoalGrabber");
        stackServo = hardwareMap.servo.get("stackServo");

        wobbleServo1 = hardwareMap.servo.get("wobbleServo1");
        wobbleServo2 = hardwareMap.servo.get("wobbleServo2");
        wobbleArm = new T3_WobbleArm(wobbleServo1, wobbleServo2);

        // Odometry
        // 0 is Auto, 1 is TeleOp
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
        wobbleArm.in();
        ringRotator.setPosition(ringRotatorUp);
        kicker.setPosition(kickerOpenPos);
        shooterAdjustor.setPosition(shootAdjDown);
        wobbleGoalGrabber.setPosition(wobbleGoalGrabberClose);
        stackServo.setPosition(stackServoPositions[2]);
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

    public void fireRings(int numRings){
        driveRobotCentric(0, 0, 0);
        sweeper.setPower(0);
        ringRotator.setPosition(ringRotatorUp);
        rest(500);

        for(int i = 0; i < numRings; i++){
            kicker.setPosition(kickerEngPos);
            rest((int) (kickerCycleTime / 2));
            kicker.setPosition(kickerOpenPos);
            rest((int) (kickerCycleTime / 2));
        }

        shooter.setPower(0);
        ringRotator.setPosition(ringRotatorDown);
        cTime.reset();
    }

    public void dropWobble(){
        wobbleArm.out();
        rest(700);

        wobbleGoalGrabber.setPosition(wobbleGoalGrabberOpen);
        rest(500);

        wobbleArm.in();
        rest(50);
        wobbleGoalGrabber.setPosition(wobbleGoalGrabberClose);
        cTime.reset();
    }

    public void lowerWobbleArm(){
        wobbleArm.out();
        rest(500);

        wobbleGoalGrabber.setPosition(wobbleGoalGrabberOpen);
        rest(500);
        cTime.reset();
    }

    public void grabWobble(){
        wobbleGoalGrabber.setPosition(wobbleGoalGrabberClose);
        rest(700);

        wobbleArm.in();
        rest(500);
        cTime.reset();
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


    public void splineMove(double targetXPos, double targetYPos, double targetAngle, double xAccuracy, double yAccuracy, double angleAccuracy, double timeout, OpMode opMode){
        double xDiff = 100, yDiff = 100, angleDiff = 100, currTime = 0;
        double prevXDiff = 100, prevYDiff = 100;
        double pX, pY, dX, dY, iX = 0, iY = 0;

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

            // updating pid components
            pX = k_p_s * xDiff;
            pY = k_p_s * yDiff;

            dX =  k_d_s * (xDiff - prevXDiff) / (currTime - prevTime);
            dY =  k_d_s * (yDiff - prevYDiff) / (currTime - prevTime);

            iX += k_i_s * (xDiff * (currTime - prevTime));
            iY += k_i_s * (yDiff * (currTime - prevTime));

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
//            driveRobotCentric(pY + dY + iY, angleToPower(angleDiff), pX + dX + iX);
            driveFieldCentric(odometry.getAngle(),pY + dY + iY, angleToPower(angleDiff), pX + dX + iX);

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


    public void cubicSplineInterpolation(Point [] p) {
        int row = 0;
        int solutionIndex = (p.length - 1) * 4;
        Arrays.sort(p);
        // initialize matrix
        BigDecimal[][] m = new BigDecimal[(p.length - 1) * 4][(p.length - 1) * 4 + 1]; // rows
        for (int  i = 0; i < (p.length - 1) * 4; i++) {
            for (int j = 0; j <= (p.length - 1) * 4; j++) {
                m[i][j] = BigDecimal.ZERO; // fill with zeros
            }
        }

        // n - 1 splines
        for (int functionNr = 0; functionNr < p.length - 1; functionNr++, row++) {
            Point p0 = p[functionNr], p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(p0.xP, MathContext.DECIMAL64).pow(3, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 1] = new BigDecimal(p0.xP, MathContext.DECIMAL64).pow(2, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 2] = new BigDecimal(p0.xP, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 3] = new BigDecimal(1, MathContext.DECIMAL64);

            m[row][solutionIndex] = new BigDecimal(p0.yP, MathContext.DECIMAL64);

            ++row;

            m[row][functionNr * 4] = new BigDecimal(p1.xP, MathContext.DECIMAL64).pow(3, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 1] = new BigDecimal(p1.xP, MathContext.DECIMAL64).pow(2, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 2] = new BigDecimal(p1.xP, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 3] = new BigDecimal(1, MathContext.DECIMAL64);

            m[row][solutionIndex] = new BigDecimal(p1.yP, MathContext.DECIMAL64);

        }

        // first derivative
        for (int functionNr = 0; functionNr < p.length - 2; functionNr++, row++) {
            Point p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(3, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP).pow(2, MathContext.DECIMAL64));

            m[row][functionNr * 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP), MathContext.DECIMAL64);

            m[row][functionNr * 4 + 2] = new BigDecimal(1, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 4] = new BigDecimal(-3).multiply(new BigDecimal(p1.xP).pow(2, MathContext.DECIMAL64));

            m[row][functionNr * 4 + 5] = new BigDecimal(-2, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP), MathContext.DECIMAL64);

            m[row][functionNr * 4 + 6] = new BigDecimal(-1, MathContext.DECIMAL64);

        }


        // second derivative
        for (int functionNr = 0; functionNr < p.length - 2; functionNr++, row++) {
            Point p1 = p[functionNr + 1];
            m[row][functionNr * 4] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP, MathContext.DECIMAL64), MathContext.DECIMAL64);

            m[row][functionNr * 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64);

            m[row][functionNr * 4 + 4] = new BigDecimal(-6, MathContext.DECIMAL64).multiply(new BigDecimal(p1.xP, MathContext.DECIMAL64), MathContext.DECIMAL64);

            m[row][functionNr * 4 + 5] = new BigDecimal(-2, MathContext.DECIMAL64);
        }


        // check these calculations later
        m[row][0] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p[0].xP, MathContext.DECIMAL64), MathContext.DECIMAL64);

        m[row++][1] = new BigDecimal(2, MathContext.DECIMAL64);

        m[row][solutionIndex - 4] = new BigDecimal(6, MathContext.DECIMAL64).multiply(new BigDecimal(p[p.length - 1].xP, MathContext.DECIMAL64), MathContext.DECIMAL64);

        m[row][solutionIndex - 4 + 1] = new BigDecimal(2, MathContext.DECIMAL64);


        BigDecimal[][] reducedRowEchelonForm = rref(m);
        BigDecimal[] coefficients = new BigDecimal[reducedRowEchelonForm.length];
        for (int i = 0; i < reducedRowEchelonForm.length; i++) {
            coefficients[i] = reducedRowEchelonForm[i][reducedRowEchelonForm[i].length - 1];
        }

        for (int i = 0; i < coefficients.length; i += 4) {
            for (double x = p[i / 4].xP; x <= p[(i / 4) + 1].xP; x += 5) {
                BigDecimal a = coefficients[i].multiply(BigDecimal.valueOf(x).pow(3, MathContext.DECIMAL64));
                BigDecimal b = coefficients[i + 1].multiply(BigDecimal.valueOf(x).pow(2, MathContext.DECIMAL64));
                BigDecimal c = coefficients[i + 2].multiply(BigDecimal.valueOf(x));
                BigDecimal d = coefficients[i + 3];
                double y = (a.add(b).add(c).add(d)).doubleValue();

                // odometry.x, y, and angle are robot's current values
                double xDiff = x - odometry.getX(); double yDiff = y - odometry.getY();
                double angle = odometry.normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)));

                splineMove(x, y, angle, 2, 2, 2, 4000, this);
            }

            double destX = p[(i / 4) + 1].xP;
            BigDecimal a = coefficients[i].multiply(BigDecimal.valueOf(destX).pow(3, MathContext.DECIMAL64));
            BigDecimal b = coefficients[i + 1].multiply(BigDecimal.valueOf(destX).pow(2, MathContext.DECIMAL64));
            BigDecimal c = coefficients[i + 2].multiply(BigDecimal.valueOf(destX));
            BigDecimal d = coefficients[i + 3];

            double destY = (a.add(b).add(c).add(d)).doubleValue();
            double xDiff = destX - odometry.getX(); double yDiff = destY - odometry.getY();
            double destAngle = odometry.normalizeAngle(Math.toDegrees(Math.atan2(yDiff, xDiff)));
            splineMove(p[(i / 4) + 1].xP, destY, destAngle, 2, 2, 2, 4000, this);
        }
    }

    public static BigDecimal [][] rref(BigDecimal[][] mat) {
        int lead = 0;
        for (int r = 0; r < mat.length; r++) {
            int i = r;
            while (mat[i][lead].compareTo(BigDecimal.ZERO) == 0) {
                i++;
                if (mat.length == i) {
                    i = r;
                    lead++;
                }
            }

            BigDecimal [] tmp = mat[i];
            mat[i] = mat[r];
            mat[r] = tmp;

            BigDecimal val = mat[r][lead];
            for (int j = 0; j < mat[0].length; j++) {
                mat[r][j] = mat[r][j].divide(val, MathContext.DECIMAL64);
            }

            for (i = 0; i < mat.length; i++) {
                if (i == r) continue;
                val = mat[i][lead];
                for (int j = 0; j < mat[0].length; j++) {
                    mat[i][j] = mat[i][j].subtract(val.multiply(mat[r][j], MathContext.DECIMAL64), MathContext.DECIMAL64);
                }
            }
            lead++;
        }
        return mat;
    }

    public void autoAim(OpMode opMode){
        double xDiff = T3_OdometryConstants.goal.xP - odometry.getX();
        double yDiff = T3_OdometryConstants.goal.yP - odometry.getY();

        double ang = Math.toDegrees(Math.atan2(yDiff, xDiff));

        moveToPosition(odometry.getX(), odometry.getY(), ang, 2, 1, 2000, opMode);
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
