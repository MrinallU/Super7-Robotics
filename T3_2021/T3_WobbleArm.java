package org.firstinspires.ftc.teamcode.T3_2021;
import com.qualcomm.robotcore.hardware.Servo;

public class T3_WobbleArm {

    private Servo servo1, servo2;
    private double servo1In = 0.5, servo1Out = 1;
    private double servo2In = 0.5, servo2Out = 1;

    public T3_WobbleArm(Servo servo1, Servo servo2){
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public void in(){
        servo1.setPosition(servo1In);
        servo2.setPosition(servo2In);
    }

    public void out(){
        servo1.setPosition(servo1Out);
        servo2.setPosition(servo2Out);
    }
}
