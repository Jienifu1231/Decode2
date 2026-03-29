package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Angle {

    public enum State {
        CLOSE,
        FAR,
        MANUAL;
    }

    public State target_state;
    public Servo outtake_angle;

    public double manual_pos = 0;
    //THESE
    public double close_pos = 0.169;//0.167
    public double far_pos = 0.22;//0.24
    //change these two

    public Angle(HardwareMap hardwareMap){
        outtake_angle = hardwareMap.get(Servo.class, "angle");

        target_state = State.MANUAL;
    }

    public void close(){target_state = State.CLOSE;}
    public void far(){target_state = State.FAR;}
    public void manual(double pos){target_state = State.MANUAL; manual_pos = pos;}

    public void update(){
        switch(target_state){
            case CLOSE:
                outtake_angle.setPosition(close_pos);
                break;
            case FAR:
                outtake_angle.setPosition(far_pos);
                break;
            case MANUAL:
                outtake_angle.setPosition(manual_pos);
                break;
        }
    }

}
//make a real outtake state to control everything when done tuning
