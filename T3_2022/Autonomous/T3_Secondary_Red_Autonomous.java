package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.spartronics4915.lib.T265Camera;

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

        odometry.updatePosition();

        // check camera confidence before proceeding
        if(odometry.poseConfidence == T265Camera.PoseConfidence.Failed || odometry.poseConfidence == T265Camera.PoseConfidence.Low){
            // camera is not confident in position or init failed
            // enter barrier and terminate

            telemetry.addLine("pose conf: LOW; Entering barrier");
            telemetry.update();

        }else{
            // camera is fairly confident abt it's pose estimation so use the values
            // cycle
            telemetry.addLine("pose conf: HIGH; Cycling");
            telemetry.update();
        }

        odometry.stopT265();
    }
}
