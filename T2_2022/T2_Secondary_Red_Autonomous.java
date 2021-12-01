package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.Vision.T2_Camera;

@Autonomous(name="T1_Secondary_Red_Autonomous", group="Autonomous")
public class T2_Secondary_Red_Autonomous extends T2_Base {
    int pos =  0;

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T2_Camera camera = new T2_Camera(hardwareMap);
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        odometry.updatePosition();
        arm.moveToPosition(300);

        // drive a little bit forward
        xTo(-11, 5000, 0.3, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // turn to wobble
        turnToV2(88, 6000, 0.4, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();
        sleep(500);

        // drop arm to desired pos
        if(pos == 0){
            arm.moveTop();
        }else if(pos == 1){
            arm.moveMidBlue();
        }else{
            arm.moveBottomBlue();
        }
        sleep(1000);

        // place arm inside freight
        yTo(-10, 5000, 0.3, 1, this, true);
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
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        //reset arm
        arm.moveToPosition(135);

        //move into barrier
        yTo(25, 5000, 0.6, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        // turn to freight stack
        turnToV2(0, 6000, this);
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
