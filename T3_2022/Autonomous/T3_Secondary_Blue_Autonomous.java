package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.spartronics4915.lib.T265Camera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;

@Autonomous(name="T3_Secondary_Blue_Autonomous", group="Autonomous")
public class T3_Secondary_Blue_Autonomous extends T3_Base {
    int pos = 0;
    String elementDiagram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();
        sleep(2000);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        arm.moveToPosition(300);
        moveTicksBack(570, 3000, 0.3, 20, this);
        sleep(500);

        arm.moveMid();
        sleep(1000);

        turnToV2(45, 4000, this);
        sleep(500);

        arm.dump();
        sleep(500);

        turnToV2(90, 2000, this);
        sleep(500);
        container.dumpBlock();
        arm.moveToPosition(300);
        sleep(500);

        moveTicksBack(1850, 4500, 0.4, 20, this);
        sleep(500);

        turnToV2(-45, 3000, this);
        arm.sweepPos();
        sleep(500);
        container.sweepRelease();
        sweeper.sweep();
        sleep(250);
        moveTicksFront(200, 2000, 0.4, 20, this);
        sleep(1500);
        sweeper.stop();

        container.sweepBlock();
        sweeper.dump();
        moveTicksBack(300, 2000, 0.4, 20, this);
        turnToV2(90, 2000, this);
        sweeper.stop();
        sleep(500);
        arm.moveToPosition(300);

        moveTicksFront(2000, 3000, 0.4, 20, this);
        sleep(500);
        arm.moveTop();

        turnToV2(45, 2000, this);
        sleep(500);
        arm.dump();

        sleep(500);
        container.dumpBlock();
        turnToV2(90, 3000, this);
        sleep(500);
        moveTicksBack(2000, 4000, 0.4, 20, this);
        sleep(500);
        arm.moveToPosition(300);

    }
}
