package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;

@Autonomous(name="T2_Secondary_Blue_Autonomous", group="Autonomous")

public class T2_Secondary_Blue_Autonomous extends T2_Base{
    @Override
    public void runOpMode() throws InterruptedException {
        init(1);
        initServosAuto();
        T2_Camera camera = new T2_Camera(hardwareMap);
        int pos = camera.readBarcode("blueSecondary");
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        odometry.updatePosition();
        arm.moveToPosition(300);

        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // move a little bit forward
        xTo(12, 5000, 0.4, 1, this, false);

        //turn to wobble
        turnToV2(-86, 10000, this);

        if(pos == 0){
            arm.moveBottomBlue();
        }else if(pos == 1){
            arm.moveMidBlue();
        }else{
            arm.moveTop();
        }
        sleep(2000);



        // move to wobble
        yTo(-19, 4000, 0.4, 1,this, false);
        sleep(500);

        arm.dump();
        sleep(500);
        arm.container.dumpBlock();
        sleep(500);

        turnToV2(-88, 10000, this);


        yTo( 25, 4000, 0.2, 1,this, false);
        sleep(500);

        sweeper.sweep();
        arm.sweepPos();
        sleep(1000);

        turnToV2(128, 3000, this);
        sleep(500);

        xTo(6, 5000, 0.4, 1, this, false);
        sleep(1000);

        sweeper.stop();
        container.sweepBlock();
        arm.moveTop();

        xTo(12, 5000, 0.4, 1, this, false);
        sleep(500);

        // turn to wobble
        turnToV2(-88, 10000, this);

        // move to wobble
        yTo(-19, 4000, 0.2, 1,this, false);

        // dump
        arm.dump();
        sleep(1000);
        arm.container.dumpBlock();
        sleep(500);

        // realign with barrier (prevents robot from getting stuck by approaching at a slightly off angle)
        turnToV2(-88, 10000, this);
        sleep(500);

        // move inside the barrier
        yTo( 25, 4000, 0.2, 1,this, false);
        sleep(500);

        arm.sweepPos();
        sleep(1000);
    }
}
