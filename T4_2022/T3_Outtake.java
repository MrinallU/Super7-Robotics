package org.firstinspires.ftc.teamcode.T4_2022;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Motor;

public class T3_Outtake {
    Motor motor1, motor2;
    T3_Container container;
    LinearOpMode opMode;

    public T3_Outtake(Motor motor1, T3_Container container, LinearOpMode opmode, double p){
        this.motor1 = motor1;
        this.container = container;
        this.opMode = opmode;
    }

    public void moveToPosition(double pos) {
        motor1.setTarget(pos);
        motor1.retMotorEx().setTargetPositionTolerance(3);
        motor1.toPosition();
        motor1.setPower(0.15);
    }

    public void moveTop(){
        container.sweepBlock();
        moveToPosition(850);
    }

    public void moveMid(){
        container.sweepBlock();
        moveToPosition(1150);
    }

    public void moveBottom(){
        container.sweepBlock();
        moveToPosition(1350);
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

    public void dump(){
        container.dumpRelease();
    }
}
