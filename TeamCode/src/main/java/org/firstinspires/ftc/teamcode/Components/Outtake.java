
package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.PIDF;

@Config
public class Outtake {

    public enum State {
        WAITING,
        LAUNCH_CLOSE,
        LAUNCH_FAR,
        LAUNCH_FAR_AUTO,
        BANG_PID,
        MANUAL
    }

    public State target_state, last_state;



    public double power = 0;

    public double total_time = 0;
    public boolean state_reached = true;
    public double vel =0;
    public double curvel = 0;
    public boolean bangbang = false;
    public boolean pid_active = false;

    public double manual_vel;

    public DcMotorEx LflyWheel;
    public DcMotorEx RflyWheel;
    public VoltageSensor voltageSensor;
    public double voltage;

    public static double kP = 0.00001;//0.000009
    public static double kI = 0;//0.1
    public static double kD = 0;//0.00007
    public static double kV = 0.00034;//0.00034
    public static double sensitivity = 10;
    public static double integral_sum_limit = 10;
    public static double norm_vel = 1000;
    public static double max_speed = 0.9;
    //close angle tuning


    public static double far_kP = 0.000014;//0.000006'
    public static double far_kI = 0.03;
    public static double far_kD = 0;
    public static double far_kV = 0.00035;//0.00038
    //tune kV first and make sure it doesnt collide with max speed

    public static double far_auto_kV = 0.000335;//0.00034

    public static double far_sensitivity = 0;
    public static double far_integral_sum_limit = 0;
    public static double far_norm_vel = 0.9;
    public static double far_max_speed = 1;

    public PIDF close_pid, far_pid, far_auto_pid;
    //switched to PIDF method instead of just PID


    public Outtake(HardwareMap hardwareMap) {
        LflyWheel = hardwareMap.get(DcMotorEx.class,"lfw");
       LflyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       LflyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LflyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //LflyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

       RflyWheel = hardwareMap.get(DcMotorEx.class,"rfw");
       RflyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RflyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RflyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //RflyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        target_state = State.WAITING;
        last_state = State.WAITING;

        close_pid = new PIDF(kP, kI, kD, kV, sensitivity, integral_sum_limit, norm_vel, max_speed, false);
        far_pid = new PIDF(far_kP, far_kI, far_kD, far_kV, far_sensitivity, far_integral_sum_limit, far_norm_vel, far_max_speed, false);
        far_auto_pid = new PIDF(far_kP, far_kI, far_kD, far_auto_kV, far_sensitivity, far_integral_sum_limit, far_norm_vel, far_max_speed, false);
    }


    public void stop() {
        target_state = State.WAITING;
    }

    public void launch_close() {
        target_state = State.LAUNCH_CLOSE;
    }

    public void launch_far(){target_state = State.LAUNCH_FAR;}
    public void launch_far_auto(){target_state = State.LAUNCH_FAR_AUTO;}
    public void bang_pid(){target_state = State.BANG_PID;}

    public void manual(double vel){target_state = State.MANUAL; manual_vel = vel; }


    public void update() {

        if (last_state != target_state) {
            total_time = 0;
            state_reached = false;
        }

        switch (target_state){
            case WAITING:
                // launch_angle = close_angle;
                power = 0;
                RflyWheel.setPower(power);
                LflyWheel.setPower(power);

                break;


            case LAUNCH_CLOSE:
                vel = 1600;//change this based on calculation
                voltage = voltageSensor.getVoltage();

                curvel = RflyWheel.getVelocity();
                power = close_pid.update(vel, curvel, voltage);
               RflyWheel.setPower(power);
                LflyWheel.setPower(power);


                //PID applied -- Test this

                //RflyWheel.setVelocity(vel);
               // LflyWheel.setVelocity(vel);

                //servo stuff

                break;

            case LAUNCH_FAR:
                // launch_angle = close_angle;
                vel = 1800;//1000
                curvel = RflyWheel.getVelocity();
                voltage = voltageSensor.getVoltage();

                power = far_pid.update(vel, curvel, voltage);//experiment
               RflyWheel.setPower(power);
               LflyWheel.setPower(power);

               voltage = voltageSensor.getVoltage();

               // RflyWheel.setVelocity(vel);
                //LflyWheel.setVelocity(vel);

                break;

            case LAUNCH_FAR_AUTO:
                // launch_angle = close_angle;
                vel = 1800;//1000
                curvel = RflyWheel.getVelocity();
                voltage = voltageSensor.getVoltage();

                power = far_auto_pid.update(vel, curvel, voltage);//experiment
                RflyWheel.setPower(power);
                LflyWheel.setPower(power);

                voltage = voltageSensor.getVoltage();

                // RflyWheel.setVelocity(vel);
                //LflyWheel.setVelocity(vel);

                break;

            case BANG_PID:
                vel = 1500; //later use a linear regression to model based on where we are on the field
                voltage = voltageSensor.getVoltage();

                curvel = RflyWheel.getVelocity();
                if(curvel <= (vel-200) || curvel >= (vel + 200)){
                    bangbang = true;
                    if(curvel <= (vel -200)){
                        power = 1;
                    }
                    if(curvel >= (vel +200)){
                        power = 0;
                    }
                }else{
                    power = far_pid.update(vel, curvel, voltage);
                    pid_active = true;
                }

                RflyWheel.setPower(power);
                LflyWheel.setPower(power);
                break;


            case MANUAL:
                RflyWheel.setVelocity(manual_vel);
                LflyWheel.setVelocity(manual_vel);
                break;



        }


    }

}
