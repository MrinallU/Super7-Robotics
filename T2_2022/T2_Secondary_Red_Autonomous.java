package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;

@Autonomous(name="T2_Secondary_Red_Autonomous", group="Autonomous")
@Disabled

public class T2_Secondary_Red_Autonomous extends T2_Base {
    int pos =  0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T2_Camera camera = new T2_Camera(hardwareMap);

        initOdometry();



        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        pos = camera.readBarcode("redSecondary");

        if(pos == 0){
            telemetry.addData("Wobble Level: ", "Bottom");
            telemetry.addData("Shipping Element Placement: ", "☒ ☐ ☐");
            elementDiagram = "☒ ☐ ☐";
            telemetry.update();
        }else if(pos == 1){
            telemetry.addData("Wobble Level: ", "Middle");
            telemetry.addData("Shipping Element Placement: ", "☐ ☒ ☐");
            elementDiagram = "☐ ☒ ☐";
            telemetry.update();
        }else if(pos == 2){
            telemetry.addData("Wobble Level: ", "Top");
            telemetry.addData("Shipping Element Placement: ", "☐ ☐ ☒");
            elementDiagram = "☐ ☐ ☒";
            telemetry.update();
        }


        odometry.updatePosition();
        arm.moveToPosition(300);

        // drive a little bit forward
        xTo(-11, 5000, 0.8, 1, this, true);
        sleep(500);

        // turn to wobble
        turnToV2(88, 6000, this);
        sleep(500);

//        // drop arm to desired pos
//        if(pos == 0){
//            arm.moveBottom();
//        }else if(pos == 1){
//            arm.moveMid();
//        }else{
//            arm.moveTop();
//        }
        arm.moveTop();
        sleep(1000);

        //align with shipping hub
        turnToV2(92, 6000, this);
        sleep(500);

        // place arm inside freight
        yTo(-11, 5000, 0.5, 1, this, true);
        sleep(500);

        // dump freight
        arm.dump();
        sleep(500);
        arm.container.dumpBlock();
        sleep(500);


        //align with barrier
        turnToV2(88, 4000, this);

        //approach barrier
        yTo(-5, 5000, 0.8, 1, this, true);
        arm.moveToPosition(500);

        //move into barrier
        yTo(50, 5000, 0.3, 1, this, true);
        sleep(500);
//        arm.sweepPos();
//
//
//        sleep(1000);
//
//        // turn to freight stack
//        sweeper.sweep();
//        turnToV2(45, 6000, this);
//        sleep(500);
//
//
//
//
//        // move into stack
//        xTo(-12, 2000, 0.3, 1, this, true);
//        sleep(2000);
//        sweeper.stop();
//        container.sweepBlock();
//
//
//        // move back
//        xTo(-14, 5000, 0.4, 1, this, true);
//
//        // move arm to cycle pos
//        arm.moveTop();
//
//        // turn to Barrier
//        turnToV2(88, 6000, this);
//        sleep(500);
//
//        // move to wobble
//        yTo(-5, 5000, 0.3, 1, this, true);
//
//
//        //turn to wobble
//        turnToV2(88, 6000, this);
//        sleep(500);
//
//        // place arm inside freight
//        yTo(-11, 5000, 0.6, 1, this, true);
//        sleep(500);
//
//        // dump freight
//        arm.dump();
//        sleep(500);
//        arm.container.dumpBlock();
//        sleep(500);
//
//        //approach barrier
//        yTo(-5, 5000, 0.8, 1, this, true);
//
//        //move into barrier
//        yTo(45, 5000, 0.3, 1, this, true);
//        sleep(500);
//
//        arm.sweepPosReset();


        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getAngle());
        }

        odometry.stopT265();
    }
}
