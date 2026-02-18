
package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.PIDF;

@Config
public class Outtake {

    public enum State {
        WAITING,
        LAUNCH_CLOSE,
        LAUNCH_FAR,
        LAUNCH_TELE,
        LAUNCH_BANG
    }

    public State target_state, last_state;



    public double power = 0;

    public double total_time = 0;
    public boolean state_reached = true;
    public double vel =0;
    public double curvel = 0;

    public DcMotorEx LflyWheel;
    public DcMotorEx RflyWheel;

    public static double kP = 0.00001;//0.000009
    public static double kI = 0;//0.1
    public static double kD = 0;//0.00007
    public static double kV = 0.000355;//0.000389
    public static double sensitivity = 10;
    public static double integral_sum_limit = 10;
    public static double norm_vel = 1000;
    public static double max_speed = 0.9;
    //close angle tuning


    public static double far_kP = 0.000014;//0.000006'
    public static double far_kI = 0.003;
    public static double far_kD = 0;
    public static double far_kV = 0.00034;//0.00039
    //tune kV first and make sure it doesnt collide with max speed
    public static double far_sensitivity = 0;
    public static double far_integral_sum_limit = 0;
    public static double far_norm_vel = 0.9;
    public static double far_max_speed = 1;

    public PIDF close_pid, far_pid;
    //switched to PIDF method instead of just PID


    public Outtake(HardwareMap hardwareMap) {LflyWheel = hardwareMap.get(DcMotorEx.class,"lfw");
       LflyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       LflyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       LflyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //LflyWheel.setDirection(DcMotorSimple.Direction.REVERSE);

       RflyWheel = hardwareMap.get(DcMotorEx.class,"rfw");
       RflyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RflyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       RflyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       //RflyWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        target_state = State.WAITING;
        last_state = State.WAITING;

        close_pid = new PIDF(kP, kI, kD, kV, sensitivity, integral_sum_limit, norm_vel, max_speed, false);
        far_pid = new PIDF(far_kP, far_kI, far_kD, far_kV, far_sensitivity, far_integral_sum_limit, far_norm_vel, far_max_speed, false);

    }


    public void stop() {
        target_state = State.WAITING;
    }

    public void launch_close() {
        target_state = State.LAUNCH_CLOSE;
    }

    public void launch_far(){target_state = State.LAUNCH_FAR;}
    public void launch_tele(){target_state = State.LAUNCH_TELE;}
    public void launch_bang(){target_state = State.LAUNCH_BANG;}


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
                curvel = RflyWheel.getVelocity();
                power = close_pid.update(vel, curvel);
               RflyWheel.setPower(power);
                LflyWheel.setPower(power);
                //PID applied -- Test this

                //RflyWheel.setVelocity(vel);
               // LflyWheel.setVelocity(vel);

                //servo stuff

                break;

            case LAUNCH_FAR:
                // launch_angle = close_angle;
                vel = 1900;//1000
                curvel = RflyWheel.getVelocity();
                power = far_pid.update(vel, curvel);//experiment
               RflyWheel.setPower(power);
               LflyWheel.setPower(power);

               // RflyWheel.setVelocity(vel);
                //LflyWheel.setVelocity(vel);

                break;
            case LAUNCH_BANG:
                vel = 1900;//1000
                curvel = RflyWheel.getVelocity();
                power = far_pid.update(vel, curvel);//experiment
                if(curvel <= vel){
                    power = 1;
                }
                RflyWheel.setPower(power);
                LflyWheel.setPower(power);
                break;

            case LAUNCH_TELE:
                vel = 1800;
                curvel = RflyWheel.getVelocity();
                power = far_pid.update(vel, curvel);//experiment
                RflyWheel.setPower(power);
                LflyWheel.setPower(power);
                break;



        }


    }

}
