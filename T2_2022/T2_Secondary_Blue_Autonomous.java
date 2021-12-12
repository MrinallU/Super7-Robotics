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
        xTo(10, 5000, 0.4, 1, this, false);

        //turn to wobble
        turnToV2(-86, 10000, this);

        if(pos == 0){
            arm.moveBottom();
        }else if(pos == 1){
            arm.moveMid();
        }else{
            arm.moveTop();
        }

        // move to wobble
        yTo(-5, 4000, 0.4, 1,this, false);

        arm.dump();
        yTo(-7, 4000, 0.1, 1,this, false);
        sleep(500);
        arm.container.dumpBlock();

        yTo(0, 4000, 0.4, 1,this, false);

        container.dumpBlock();
        arm.moveToPosition(300);



    }
}
