package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    public enum State {
        DOWN,
        LIFTING,
        MANUAL
    }

    public State target_state;

    public Servo lift1;
    public Servo lift2;
    public Servo lift3;

    public double downpos = 0;
    public double liftpos = 1;//0.4
    public double manual_pos = 0;

    public Lift(HardwareMap hardwareMap) {

        lift1 = hardwareMap.get(Servo.class, "lift1");
        lift1.setDirection(Servo.Direction.FORWARD);
        //reverse this in axon programmer
        //lift1.setPosition(0.5);
        //change

        lift2 = hardwareMap.get(Servo.class, "lift2");
        lift2.setDirection(Servo.Direction.FORWARD);

        lift3 = hardwareMap.get(Servo.class, "lift3");
        lift3.setDirection(Servo.Direction.FORWARD);

    }

    public void down() {
        target_state = State.DOWN;
    }

    public void lift() {
        target_state = State.LIFTING;
    }

    public void manual(double pos) {
        target_state = State.MANUAL;
        manual_pos = pos;
    }


    public void update() {

        /*switch (target_state) {
             case DOWN:
                 lift1.setPosition(downpos);
                 lift2.setPosition(downpos);
                 lift3.setPosition(downpos);

                 break;

             case LIFTING:
                 lift1.setPosition(liftpos);
                 lift2.setPosition(liftpos);
                 lift3.setPosition(liftpos);
                 break;

             case MANUAL:
                 lift1.setPosition(manual_pos);
                 lift2.setPosition(manual_pos);//used to be just manual power
                 lift3.setPosition(manual_pos);
                 break;
         }*/

    }
}






