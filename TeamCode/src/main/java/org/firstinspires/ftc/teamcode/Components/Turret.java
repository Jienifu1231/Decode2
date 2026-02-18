package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PID;

@Config
public class Turret {

    public enum State{
        STOP,
        LIMERED,
        LIMEBLUE,
        MANUAL
    }

    public Limelight3A limelight;
    public LLResult limeResult;

    public State target_state, last_state;
    public double turret_power = 0;

    public DcMotor turret;

   public  PID lime_pid;
   public double tx =0;
    public double output = 0;
   public boolean llresult = false;




    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        target_state = State.STOP;
        last_state = State.STOP;
        lime_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);

        //limelight.pipelineSwitch(0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();


    }

    public static double kp = 0.03;//0.06
    public static double ki = 0.15;
    public static double kd = 0;
    public static double sensitivity = 3;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.5;
    public static double max = 0.5;


    public void stop(){target_state = State.STOP;}
    public void limeRed(){target_state = State.LIMERED;}
    public void limeBlue(){target_state = State.LIMEBLUE;}
    public void manual(double p){turret_power = p; target_state = State.MANUAL;}

    public void update(){

        switch (target_state){
            case STOP:
                turret_power = 0;
                turret.setPower(turret_power);
                break;
            case LIMERED:
                limeResult = limelight.getLatestResult();
                if(limeResult.isValid() && limeResult != null){
                    tx = limeResult.getTx();
                    turret_power = lime_pid.update(0, tx);
                    llresult = true;
                }else{
                    llresult = false;
                }
                turret.setPower(-turret_power);
                break;

            case LIMEBLUE:
                limeResult = limelight.getLatestResult();
                if(limeResult.isValid() && limeResult != null){
                    tx = limeResult.getTx();
                    turret_power = lime_pid.update(0, tx);
                    llresult = true;
                }else{
                    llresult = false;
                }
                turret.setPower(-turret_power);
                break;

            case MANUAL:
                turret.setPower(turret_power);
                break;

        }

        last_state = target_state;
    }
}