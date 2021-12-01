package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.Vision.T2_Camera;

@Autonomous(name="T1_Primary_Red_Autonomous", group="Autonomous")
public class T2_Primary_Red_Autonomous extends T2_Base {
    int pos =  0;

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T2_Camera camera = new T2_Camera(hardwareMap);
        pos = camera.readBarcode("redPrimary");
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        odometry.updatePosition();
        arm.moveToPosition(300);

        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // move forward
        xTo(-10, 5000, 0.5, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        //turn
        turnToV2(88, 6000, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // move forward to carousel
        yTo(-15, 5000, 0.5, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // turn to carousel
        turnToV2(178, 10000, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // move forward to carousel
        xTo(-23.5, 2000, 0.4, 1, this, false);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // carousel
        startCarousel();
        sleep(3000);
        stopCarousel();

        // move a few inches back
        xTo(-31, 5000, 0.5, 1, this, false);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);


        // turn 90 degrees
        turnToV2(92, 10000, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);


        // move to wobble a few inches behind to drop the arm
        yTo(5, 5000, 0.5, 2, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // position arm to appropriate level
        if(pos == 0){
            arm.moveTop();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveBottom();
        }
        sleep(1000);

        // move arm inside the wobble
        yTo(18, 5000, 0.3, 2, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);


        // dump freight
        arm.dump();
        sleep(1500);
        arm.container.dumpBlock();

        // move back
        yTo(0, 5000, 0.4, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // reset arm to 135
        arm.moveToPosition(135);
        sleep(500);

        // move back and park
        yTo(-19, 5000, 0.5, 2, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // turn to hub
        turnToV2(0, 10000, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // park inside hub
        xTo(-25, 5000, 0.6, 2, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);


        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getAngle());
        }

        odometry.stopT265();
     }
}
