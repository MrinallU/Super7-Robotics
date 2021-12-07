package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;

@Autonomous(name="T1_Secondary_Red_Autonomous", group="Autonomous")
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
        xTo(-11, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // turn to wobble
        turnToV2(88, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // drop arm to desired pos
        if(pos == 0){
            arm.moveBottom();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveTop();
        }
        sleep(1000);

        turnToV2(88, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // place arm inside freight
        yTo(-12, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // dump freight
        arm.dump();
        sleep(1000);
        arm.container.dumpBlock();
        sleep(500);

        //approach barrier
        yTo(-2, 5000, 0.5, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        //move into barrier
        yTo(50, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // turn to freight stack
        turnToV2(45, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        arm.sweepPos();


        // move into stack
        xTo(-8, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);


        // sweep
        sweeper.sweep();
        sleep(2000);
        sweeper.stop();
        container.sweepBlock();
        arm.moveTop();

        // move back
        xTo(-13, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // turn to Barrier
        turnToV2(88, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // move to wobble
        yTo(0, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // turn to wobble
        turnToV2(88, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        yTo(-12, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // drop arm to desired pos
        arm.moveTop();

        turnToV2(88, 6000, this);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // place arm inside freight
        yTo(-12, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // dump freight
        arm.dump();
        sleep(1000);
        arm.container.dumpBlock();
        sleep(500);

        //approach barrier
        yTo(-2, 5000, 0.5, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        //move into barrier
        yTo(50, 5000, 0.3, 1, this, true);
        telemetry.addData("Shipping Element Placement: ", elementDiagram);
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
