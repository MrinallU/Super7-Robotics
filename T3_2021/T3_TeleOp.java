package org.firstinspires.ftc.teamcode.T3_2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utils.Angle;

@Disabled
@TeleOp(name = "T3_TeleOp", group = "T3")
public class T3_TeleOp extends T3_Base{
    // Drive Variables
    double targetAngle, currAngle, drive, turn, strafe, multiplier = 1;

    // TeleOp Variables
    boolean basicDrive = true, slowDrive = false, fastDrive = false;
    ElapsedTime kTime = new ElapsedTime();
    boolean shoot = false, bShoot = false, shootOn = false;
    boolean pushDone = false, retDone = false;
    boolean wobbleClosed = true;
    boolean shootFlapUp = false;
    boolean stackUp = true;

    // Button Variables
    boolean yP = false, yLP = false;
    boolean rP2 = false, rLP2 = false;
    boolean lP2 = false, lLP2 = false;
    boolean dDP2 = false, dDLP2 = false;
    boolean dUP2 = false, dULP2 = false;
    boolean aP = false, aLP = false;
    boolean aP2 = false, aLP2 = false;
    boolean bP2 = false, bLP2 = false;
    boolean yP2 = false, yLP2 = false;
    boolean stackServoLast = false, stackServoCurr = false;

    // TeleOp Constants / Variables
    double setShooterPow = 1, reverseShooterPow = -1, shooterChangePower = 0.025;
    double minSweepPow = 0.6, maxSweepPow = 1, reverseSweeperPow = -0.7;
    double [] stackPos = {0, 0.3, 0.5};

    // TeleOp Powers
    double shooterPow = 0, sweeperPow = 0;

    // Drive Type
    String driveType;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(0);
        initServos();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        matchTime.reset();
        resetCache();
        setReadings();

        adjustToPower();

        targetAngle = -T3_Readings.angle - 90;
        shooter.setPID(0.055, 0, 0.01);

        while(opModeIsActive()){
            // Reset Hub Cache
            resetCache();


            // Update Variables
            setReadings();
            odometry.updatePosition();

            // Reset Angle
            currAngle = T3_Readings.angle;
            if (gamepad1.x) {
                targetAngle = -currAngle - 180;
            }

            // Change Drive Mode
            yLP = yP;
            yP = gamepad1.y;
            if(!yLP && yP){
                basicDrive = !basicDrive;
            }

            // Drive
            if(gamepad1.left_bumper){
                slowDrive = true;
            }
            else{
                slowDrive = false;
            }

            if(gamepad1.left_trigger > 0.05){
                fastDrive = true;
            }
            else{
                fastDrive = false;
            }

            drive = floor(gamepad1.right_stick_y) * multiplier;
            strafe = floor(-gamepad1.right_stick_x) * multiplier;
            turn = turnFloor(gamepad1.left_stick_x) * multiplier;

            if(basicDrive){
                driveType = "Robot Centric";

                if(gamepad1.dpad_right){
                    driveRobotCentric(0, dpadTurnSpeed, 0);
                }
                else if(gamepad1.dpad_left){
                    driveRobotCentric( 0, -dpadTurnSpeed, 0);
                }
                else if(gamepad1.dpad_up){
                    driveFieldCentric(targetAngle + currAngle,0, autoAngleToPower * Angle.angleDifference(currAngle, targetAngle + 90), 0);
                }
                else{
                    driveRobotCentric(drive, turn, strafe);
                }
            }
            else{
                driveType = "Field Centric";

                if(gamepad1.dpad_right){
                    driveFieldCentric(targetAngle + currAngle, 0, dpadTurnSpeed, 0);
                }
                else if(gamepad1.dpad_left){
                    driveFieldCentric(targetAngle + currAngle, 0, -dpadTurnSpeed, 0);
                }
                else if(gamepad1.dpad_up){
                    driveFieldCentric(targetAngle + currAngle, -dpadDriveSpeed, 0, 0);
                }
                else if(gamepad1.dpad_down){
                    driveFieldCentric(targetAngle + currAngle, dpadDriveSpeed, 0, 0);
                }
                else{
                    driveFieldCentric(targetAngle + currAngle, drive, turn, strafe);
                }
            }

            // Adjusting Shooter Power
            rLP2 = rP2;
            rP2 = gamepad2.dpad_right;

            lLP2 = lP2;
            lP2 = gamepad2.dpad_left;
            if(rP2 && !rLP2){
                setShooterPow = normalizeThreeDigits(setShooterPow + shooterChangePower);
                setShooterPow = Math.min(1, setShooterPow);
            }
            else if(lP2 && !lLP2){
                setShooterPow = normalizeThreeDigits(setShooterPow - shooterChangePower);
                setShooterPow = Math.max(0, setShooterPow);
            }

            // Wobble Goal Grabber
            aLP = aP;
//            aP = gamepad1.a;
            if(aP && !aLP){
                wobbleClosed = !wobbleClosed;

                if(wobbleClosed){
                    wobbleGoalGrabber.setPosition(wobbleGoalGrabberClose);
                }
                else{
                    wobbleGoalGrabber.setPosition(wobbleGoalGrabberOpen);
                }
            }

            if (matchTime.seconds() > 90 || gamepad2.a) {
                if (gamepad2.right_stick_button) {
                    adjustToPower();
                    moveToPosition(T3_OdometryConstants.splineTest[0], 1000, this);
                    cubicSplineInterpolation(T3_OdometryConstants.splineTest);
                }
            }

            dDLP2 = dDP2;
            dDP2 = gamepad2.dpad_down;

            dULP2 = dUP2;
            dUP2 = gamepad2.dpad_up;

            if(dDP2 && !dDLP2){
                wobbleArm.out();
            }
            else if(dUP2 && !dULP2){
                wobbleArm.in();
            }

            // Kicker
            if(!gamepad2.x){
                kTime.reset();
                if(pushDone && !retDone){
                    kicker.setPosition(kickerOpenPos);
                }

                retDone = false;
                pushDone = false;
            }
            if(ringRotator.getPosition() == ringRotatorUp) {
                if (kTime.milliseconds() % kickerCycleTime > 10 && kTime.milliseconds() % kickerCycleTime < kickerCycleTime / 2 && !pushDone) {
                    pushDone = true;
                    retDone = false;

                    kicker.setPosition(kickerEngPos);
                } else if(kTime.milliseconds() % kickerCycleTime > kickerCycleTime / 2 && !retDone){
                    pushDone = false;
                    retDone = true;
                    kicker.setPosition(kickerOpenPos);
                }
            }


            // Ring Rotator
            aLP2 = aP2;
            aP2 = gamepad2.a;

            bLP2 = bP2;
            bP2 = gamepad2.b;

            if(aP2 && !aLP2){
                ringRotator.setPosition(ringRotatorUp);
            }
            else if(bP2 && !bLP2){
                ringRotator.setPosition(ringRotatorDown);
            }

            // Shooter
            shoot = bShoot;
            bShoot = gamepad2.left_trigger > 0.05;

            if(shoot && !bShoot){
                shootOn = !shootOn;
            }

            if(shootOn){
                shooterPow = setShooterPow;
                ringRotator.setPosition(ringRotatorUp);
            }
            else if(gamepad2.left_bumper){
                shooterPow = reverseShooterPow;
            }
            else{
                shooterPow = 0;
            }

            // Shooter Adjustor
            yLP2 = yP2;
            yP2 = gamepad2.y;

            if(yP2 && !yLP2){
                shootFlapUp = !shootFlapUp;

                if(shootFlapUp){
                    shooterAdjustor.setPosition(shootAdjUp);
                }
                else{
                    shooterAdjustor.setPosition(shootAdjDown);
                }
            }

            // Sweeper
            if (gamepad1.right_trigger > 0.05){
                ringRotator.setPosition(ringRotatorDown);
                sweeperPow = gamepad1.right_trigger * (maxSweepPow - minSweepPow) + minSweepPow;
            }
            else if(gamepad1.right_bumper){
                sweeperPow = reverseSweeperPow;
            }
            else{
                sweeperPow = 0;
            }

            //Stack Servo
            stackServoLast = stackServoCurr;
            stackServoCurr = gamepad1.right_stick_button;
            if(stackServoCurr && !stackServoLast){
                if(stackServo.getPosition() == 0){
                    stackServo.setPosition(stackPos[1]);
                }else if(stackServo.getPosition() == 0.3){
                    stackServo.setPosition(stackPos[2]);
                }else{
                    stackServo.setPosition(stackPos[0]);
                }
            }

            // Set Powers
            shooter.setPower(shooterPow);
            sweeper.setPower(sweeperPow);


            // Display Values
            telemetry.addData("Drive Type", driveType);
            telemetry.addData("Shooter Power", setShooterPow);
            telemetry.addLine(odometry.outStr);
            telemetry.update();
        }
    }

    // Drive Floor Functions
    public double floor(double rawInput) {
        if(slowDrive){
            return ((int) (rawInput * 5.5)) / 11.0;
        }
        else if(fastDrive){
            return ((int) (rawInput * 11)) / 11.0;
        }
        return ((int) (rawInput * 9)) / 11.0;
    }

    // Turning Floor
    public double turnFloor(double rawInput) {
        return ((int) (rawInput * 15)) / 20.0;
    }
}