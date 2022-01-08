package org.firstinspires.ftc.teamcode.T3_2022.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.T2_2022.Modules.T2_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.Modules.T3_Camera;
import org.firstinspires.ftc.teamcode.T3_2022.T3_Base;
import org.firstinspires.ftc.teamcode.Utils.Point;
@Disabled
@Autonomous(name="T3_Splinetest", group="Autonomous")
public class T3_Splinetest extends T3_Base {
    int pos = 0;
    String elementDiagram = "";
    Point [] splineTest = {new Point(0, 0), new Point(23, 17), new Point(51, 29), new Point(75, 2)};

    @Override
    public void runOpMode() throws InterruptedException {
        init(0);
        initServosAuto();
        T3_Camera camera = new T3_Camera(hardwareMap);
        initOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        wheelOdometry.updatePosition(
                leftDrive.encoderReading(),
                rightDrive.encoderReading(),
                getAngle());
        arm.moveToPosition(300);

        sleep(1000);
        traverseSpline(splineTest, 0.4, 2, 8,false);

        sleep(500);
    }
}