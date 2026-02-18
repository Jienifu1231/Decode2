package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class PID {

    public double kP, kI, kD;
    public double sensitivity;
    public double integral_sum_limit;
    public double norm_vel;
    public double max;
    public boolean angleMode;

    public PID(double kP, double kI, double kD, double sensitivity, double integral_sum_limit, double norm_vel, double max, boolean angleMode){
        //PID tuning constants
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        //how close to the target the robot has to be
        this.sensitivity = sensitivity;

        //maximum for the integral term
        this.integral_sum_limit = integral_sum_limit;

        //typical top speed of the system
        this.norm_vel = norm_vel;

        //maximum possible output
        this.max = max;

        //whether or not the PID is being used for the angle
        this.angleMode = angleMode;
    }

    //whether the robot has reached the target position
    public boolean pos_reached = false;

    //extent to which you smooth out the change in error by weighing the previously calculated
    //change with the current calculated change
    private final double weight = 0.1;

    //variables for storing previous values, i.e. to calculate change
    private double last_error = 0.0;
    private double last_estimate = 0.0;
    private double last_target = 0.0;
    private double last_pos = 0.0;
    private double last_time = 0.0;

    //calculated integral sum
    public double integral_sum = 0.0;

    //calculated velocity
    public double vel = 0.0;

    //output
    public double out = 0.0;

    //function to set max power
    public void setMaxPower(double new_max){
        this.max = new_max;
    }

    //function to set sensitivity
    public void setSensitivity(double new_sensitivity) { this.sensitivity = new_sensitivity; }

    //function to set tuning coefficients
    public void setCoefficients(double new_p, double new_i, double new_d) {
        this.kP = new_p;
        this.kI = new_i;
        this.kD = new_d;
    }

    //function to perform calculation in a loop; inputs desired target and current position
    public double update(double target, double pos){

        double currTime = System.nanoTime();
        double time_elapsed = (currTime - last_time)/1.0E9;

        if (last_time == 0) { //added
            last_time = currTime;
            last_pos = pos;
            return 0;
        } //added

        last_time = currTime;

        double error;
        if(this.angleMode){
            //calculate the error for heading pid
            double angleNeutral = target - pos;
            double angleMinus = target - (pos - 2*Math.PI);
            double anglePlus = target - (pos + 2*Math.PI);

            double angleShortest = Math.min(Math.min(Math.abs(angleNeutral), Math.abs(angleMinus)), Math.abs(anglePlus));

            if(angleShortest == Math.abs(angleNeutral)){
                error = angleNeutral;
            } else if(angleShortest == Math.abs(angleMinus)){
                error = angleMinus;
            } else {
                error = anglePlus;
            }
        } else {
            //calculate error normally
            error = target - pos;
        }

        //calculate velocity
        vel = (pos - last_pos) / time_elapsed;

        //calculate weighted change in error
        double cur_estimate = (weight * last_estimate) + (1 - weight) * (error - last_error);

        //calculate derivative
        double derivative = cur_estimate / time_elapsed;

        //calculate integral sum
        integral_sum += (cur_estimate * time_elapsed / 2);

        // ensure the integral sum cannot exceed the limit
        integral_sum = Range.clip(this.integral_sum, -this.integral_sum_limit, this.integral_sum_limit);

        //calculate the output using weighted p, i and d
        out = ((this.kP * error) + (this.kI * integral_sum) + (this.kD * derivative));

        //ensure that the output is never more than the max value (1)
        out = Range.clip(out, -this.max, this.max);

        //determine whether the target has been reached
        pos_reached = Math.abs(error) <= this.sensitivity;

        //integral clamping - turns off the integral term when the target changes or when the kP term is already enough
        if (target != last_target || (vel > this.norm_vel || vel < -this.norm_vel)){
            integral_sum = 0;
        }

        //redefine the stored values
        last_error = error;
        last_estimate = cur_estimate;
        last_target = target;
        last_pos = pos;

        return out;
    }

}
