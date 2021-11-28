package org.firstinspires.ftc.teamcode.T3_2021;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Utils.Motor;

public class T3_MotorPair {
    Motor motor1, motor2;
    
    public T3_MotorPair(Motor motor1, Motor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
    }
    
    public void setPower(double power){
        motor1.setPower(-power);
        motor2.setPower(power);
    }

    public void setPID(double p, double i, double d){
        motor1.retMotorEx().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, 0));
        motor2.retMotorEx().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, 0));
    }

}
