package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    public enum State{
        CLOSE,
        OPEN,
        MANUAL,
    }

    public State target_state, last_state;

    public double close_pos = 0.4;//0.4
    public double open_pos = 0;//0
    public double manual_pos = 0;

    public Servo Gate1;

    public boolean GateOpen = false;

    public Gate(HardwareMap hardwareMap){
        Gate1 = hardwareMap.servo.get("Gate1");

        target_state = State.CLOSE;
        last_state = State.CLOSE;
    }

    public void close(){target_state = State.CLOSE;}
    public void open(){target_state = State.OPEN;}
    public void manual(double pos){target_state = State.MANUAL; manual_pos = pos;}

    public void update(){
        switch(target_state){
            case CLOSE:
                Gate1.setPosition(open_pos);
                break;

            case OPEN:
                Gate1.setPosition(close_pos);
                break;

            case MANUAL:
                Gate1.setPosition(manual_pos);
                break;
        }


    }



}
