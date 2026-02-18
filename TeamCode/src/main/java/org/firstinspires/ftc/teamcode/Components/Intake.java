package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {


    //1 motor spins all wheels on track
    // 2 servos to close "gate"
    //different component to control outtake - separate for turret or combine

    public enum State{
        STOP,
        MANUAL,
        REVERSE,
        EXPRELEASE,
    }

    public State target_state, last_state;
    public double power = 0.85;

    public double manualP = 0;

    //experiment


    public DcMotor Intake;

    //public Servo Gate2;

    public Intake(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        target_state = State.STOP;
        last_state = State.STOP;
    }
    public boolean gateOpen;

    public void stop(){target_state = State.STOP;}
    public void reverse(){target_state = State.REVERSE;}
    public void manual(double p){target_state = State.MANUAL; manualP = p;}
    public void exprelease(boolean gate){target_state = State.EXPRELEASE; gateOpen = gate;}
 //another state for manual gate or just button in tele?



    public void update(){



        switch (target_state){

            case STOP:
              // Gate1.setPosition(gate1closed);
                Intake.setPower(0);
                break;
            case REVERSE:
                Intake.setPower(-0.6);
                break;

            case MANUAL:
                Intake.setPower(manualP);
                break;


            case EXPRELEASE:
                Intake.setPower(power);
                break;


        }

            last_state = target_state;




    }




}
