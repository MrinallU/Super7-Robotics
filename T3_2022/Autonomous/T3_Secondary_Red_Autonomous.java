package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Red_Autonomous", group="Autonomous")
public class T3_Secondary_Red_Autonomous extends T3_Base {
    int pos =  0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(3000);

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

        arm.moveToPosition(300);

        telemetry.addData("Shipping Element Placement: ", elementDiagram);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();


        moveTicksBack(300, 4000, 0.5, 20,this);
        sleep(500);


        turnToV2(86, 4000, this);
        sleep(500);


        if(pos == 0){
            arm.moveBottomRedSecond();
            sleep(1500);
            arm.moveToPosition(1230);
            sleep(500);
            moveTicksBack(730, 4000, 0.4, 20, this); //bottom deposit
        }else if(pos == 1){
            arm.moveMid();
            sleep(1500);
            moveTicksBack(800, 4000, 0.4, 20, this); //mid and top deposit
        }else{
            arm.moveTop();
            sleep(1500);
            moveTicksBack(900, 4000, 0.4, 20, this); //mid and top deposit
        }

        sleep(500);
        arm.dump();
        sleep(500);


        moveTicksFront(2500, 4000, 0.4, 20, this);
        sleep(500);
        container.dumpBlock();
        arm.sweepPos();
        sleep(1000);
        turnToV2(25, 2000, this);
        sleep(250);


        sleep(250);
        sweeper.sweep();
        sleep(250);
        moveTicksFront(440, 4000, 0.4, 20, this);
        sleep(2000);
        sweeper.stop();
        container.sweepBlock();

        sweeper.dump();
        sleep(1000);
        sweeper.stop();


        moveTicksBack(300, 4000, 0.4, 20, this);

        turnToV2(90, 4000, this);
        sleep(250);

        moveTicksBack(500, 4000, 0.5, 20,this);
        sleep(250);
        arm.moveToPosition(500);


        moveTicksBack(2000, 4000, 0.5, 20,this);
        sleep(250);


        arm.moveTop();
        sleep(1500);

        arm.dump();
        sleep(500);
        container.dumpBlock();

        moveTicksFront(2450, 6000, 0.4, 20, this);
        arm.sweepPos();
    }
}
