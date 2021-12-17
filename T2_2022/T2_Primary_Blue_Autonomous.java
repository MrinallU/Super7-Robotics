package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;

@Autonomous(name="T2_Primary_Blue_Autonomous", group="Autonomous")
public class T2_Primary_Blue_Autonomous extends T2_Base {
    @Override
    public void runOpMode() throws InterruptedException {
        init(1);
        initServosAuto();
        T2_Camera camera = new T2_Camera(hardwareMap);
       int pos = camera.readBarcode("bluePrimary");
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        odometry.updatePosition();
        arm.moveToPosition(300);



        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // move a little bit foward
        xTo(10, 5000, 0.4, 1, this, false);

        //turn
        turnToV2(86, 3000, 1, 3, this);
        sleep(500);

        // move forward to carousel
        yTo(-16.5, 2500, 0.2, 1,this, true);


        sleep(500);



        // start carousel
        startBlueCarousel();
        sleep(3000);
        stopCarousel();

        // turn to wobble
        turnToV2(-180, 3000, 1, 3, this);

        // move back
        xTo(14, 5000, 0.5, 1, this, false);
        sleep(500);

        // turn to wobble
        turnToV2(-88, 10000, this);

        // apporach wobble
        yTo(-5, 4000, 0.4, 1,this, false);

        if(pos == 0){
            arm.moveBottom();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveTop();
        }
        sleep(1500);

        yTo(10, 4000, 0.4, 1,this, false);

        arm.dump();
        yTo(13, 4000, 0.1, 1,this, false);
        sleep(500);
        arm.container.dumpBlock();

        yTo(-30, 4000, 0.4, 1,this, false);


        arm.moveToPosition(135);
        sleep(500);

        // turn to wobble
        turnToV2(-180, 3000, 1, 3, this);
        sleep(500);

        // move back
        xTo(23, 5000, 0.5, 1, this, false);





        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getRelativeAngle());
        }

        odometry.stopT265();
    }
}
