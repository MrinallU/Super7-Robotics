package org.firstinspires.ftc.teamcode.T2_2022.Modules;

import com.qualcomm.robotcore.hardware.Servo;

public class T2_Container {
    Servo frontBlocker, sideBlocker;
    double frontBlockPos = 0.20, sideBlockPos = 0.8, frontReleasePos = 1, sideReleasePos = 0;

    public T2_Container(Servo frontBlocker, Servo sideBlocker){
        this.frontBlocker = frontBlocker;
        this.sideBlocker = sideBlocker;
    }

    public void sweepBlock(){ // after sweeping close to go to the hub
        frontBlocker.setPosition(frontBlockPos);
    }

    public void dumpBlock(){ // after sweeping close to go to the hub
        sideBlocker.setPosition(sideBlockPos);
    }


    public void dumpRelease(){ // release
        sideBlocker.setPosition(sideReleasePos);
    } // to dump

    public void sweepRelease(){ // get ready for sweeping
        frontBlocker.setPosition(frontReleasePos);
    }

    public void init(){
        frontBlocker.setPosition(frontReleasePos);
        sideBlocker.setPosition(sideBlockPos);
    }

    public void initAuto(){
        frontBlocker.setPosition(frontBlockPos);
        sideBlocker.setPosition(sideBlockPos);
    }
}
