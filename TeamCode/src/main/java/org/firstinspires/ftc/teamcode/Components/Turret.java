package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PID;

@Config
public class Turret {

    public enum State{
        STOP,
        LIMERED,
        LIMEBLUE,
        PINPOINT,
        MANUAL
    }

    public Limelight3A limelight;
    public LLResult limeResult;

    public State target_state, last_state;
    public double turret_power = 0;

    public DcMotorEx turret;

   public  PID lime_pid;
   public PID pp_pid;
   public double tx =0;
    public double output = 0;
   public boolean llresult = false;




    public Turret(HardwareMap hardwareMap){
        turret = hardwareMap.get(DcMotorEx.class,"turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //turret = hardwareMap.get(DcMotorEx.class, "turret");
        //turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //LflyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        target_state = State.STOP;
        last_state = State.STOP;
        lime_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(5);
        //pipe 5 auto red
        //pipe 3 tele red
        //red close is 1
        //blue close is 0
        //blue far is 2
        //blue auto 4
        limelight.setPollRateHz(100);
        limelight.start();             
                      

    }

    public static double kp = 0.036;//0.039                                                                             
    public static double ki = 0.15;//0.15
    public static double kd = 0;
    public static double sensitivity = 2;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.5;
    public static double max = 0.78;
/*
    public static double kp = 0.03;//0.06
    public static double ki = 0.15;
    public static double kd = 0;
    public static double sensitivity = 1;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.6;
    public static double max = 0.9;

 */

    public void stop(){target_state = State.STOP;}
    public void limeRed(){target_state = State.LIMERED;}
    public void limeBlue(){target_state = State.LIMEBLUE;}
    public void pinpoint(){target_state = State.PINPOINT;}
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
                turret.setPower(turret_power);
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
                turret.setPower(turret_power);
                break;

            case PINPOINT:
//using pinpoint to check the heading
                break;

            case MANUAL:
                turret.setPower(turret_power);
                break;

        }

        last_state = target_state;
    }
}