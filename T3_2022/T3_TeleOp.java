package org.firstinspires.ftc.teamcode.T3_2022;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.T3_2021.T3_Readings;
import org.firstinspires.ftc.teamcode.Utils.Angle;


@TeleOp(name = "T3-TeleOp", group = "Test")
public class T3_TeleOp extends T3_Base {
    // Drive Variables
    double targetAngle, currAngle, drive, turn, strafe, multiplier = 1, left = 0, right = 0, max;
    String driveType;
    // TeleOp Variables
    boolean basicDrive = true, slowDrive = false, fastDrive = false, carouselOn = false;


    // Button Variables
    boolean carouselIsOn = false;
    boolean sweeperIsOn = false;
    boolean armIsOn = false;
    boolean yP;
    boolean yLP;
    boolean y2P;
    boolean yL2P;
    boolean dPadRight2 = false;
    boolean dPadRightLast2 = false;
    boolean dPadUp2 = false;
    boolean dPadUpLast2 = false;
    boolean dPadDown2 = false;
    boolean dPadDownLast2 = false;

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

        while(opModeIsActive()){
            // Reset Hub Cache
            resetCache();


            // Update Variables
            odometry.updatePosition();
            setReadings();


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

    public void startCarousel(){
        carousel.setPower(carouselPow);
    }

    public void stopCarousel(){
        carousel.setPower(0);
    }
}