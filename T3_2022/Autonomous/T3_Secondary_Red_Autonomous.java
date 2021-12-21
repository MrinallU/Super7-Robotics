package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Red_Autonomous", group="Autonomous")
public class T3_Secondary_Red_Autonomous extends T3_Base {
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



        odometry.updatePosition();

        pos = camera.readBarcode("redPrimary");


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


        moveTicksBack(200, 4000, 0.5, 20,this);
        sleep(500);

        turnToV2(90, 4000, this);
        sleep(500);

        pos = 1;


        while(opModeIsActive()){
            resetCache();
            odometry.updatePosition();
            telemetry.addLine("cam pos " + odometry.outStr);
            telemetry.addLine("imu angle " + getAngle());
        }

        odometry.stopT265();
    }
}
