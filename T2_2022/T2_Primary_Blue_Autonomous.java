package org.firstinspires.ftc.teamcode.T2_2022;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;

@Autonomous(name="T1_Primary_Blue_Autonomous", group="Autonomous")
public class T2_Primary_Blue_Autonomous extends T2_Base {
    int armPos =  1000;
    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServos();
        T2_Camera camera = new T2_Camera(hardwareMap);
        camera.readBarcode("bluePrimary");
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        odometry.updatePosition();
        arm.moveToPosition(300);
        int pos = 0;

        if (pos == 0){

        }else if(pos == 1){

        }else{

        }


        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        xTo(-15, 5000, 0.2, 1, this, true);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();


        turnToV2(-115, 10000, this);
        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        sleep(500);

        telemetry.addLine("Ang: " + getAngle());
        telemetry.addLine("Pos: " + odometry.outStr);
        telemetry.update();

        sleep(500); // park

        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getRelativeAngle());
        }

        odometry.stopT265();
    }
}
