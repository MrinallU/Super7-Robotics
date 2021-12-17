package org.firstinspires.ftc.teamcode.T2_2022.Modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Motor;

public class T2_Outtake {
    public Motor motor1, motor2;
    public T2_Container container;
    LinearOpMode opMode;

    public T2_Outtake(Motor motor1, T2_Container container, LinearOpMode opmode, double p){
        this.motor1 = motor1;
        this.container = container;
        this.opMode = opmode;
    }

    public void moveToPosition(double pos) {
        motor1.setTarget(pos);
        motor1.retMotorEx().setTargetPositionTolerance(3);
        motor1.toPosition();
        motor1.setPower(0.3);
    }

    public void moveTop(){
        container.sweepBlock();
        moveToPosition(930);
    }

    public void moveMid(){
        container.sweepBlock();
        moveToPosition(1140);
    }

    public void moveBottom(){
        container.sweepBlock();
        moveToPosition(1235);
    }

    public void moveBottomBlue(){
        container.sweepBlock();
        moveToPosition(1250);
    }

    public void sweepPos(){
        container.sweepBlock();
        moveToPosition(0);
        container.sweepRelease();
    }

    public void sweepPosReset(){
        container.sweepBlock();
        moveToPosition(0);
    }


    public void moveMidBlue(){
        container.sweepBlock();
        moveToPosition(1130);
    }



    public void autoInitPos(){
        moveToPosition(135);
    }

    public void dump(){
        container.dumpRelease();
    }
}
