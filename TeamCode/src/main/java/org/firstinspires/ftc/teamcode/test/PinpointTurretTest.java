package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
@Config
public class PinpointTurretTest extends GorillabotCentral {
    public PID pp_pid, heading_pid;
    public static double kp = 0.03;//0.06
    public static double ki = 0.15;
    public static double kd = 0;
    public static double sensitivity = 0.1;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.5;
    public static double max = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        double heading = 0;
        double output = 0;
        double ticksPerDegree = 5.7;//in degrees

        Pose2d init_pos = new Pose2d(0,0, Math.toRadians(180));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));
        Pose2d curpos = new Pose2d(0,0, Math.toRadians(0));

        pp_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,true);

        heading_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);
        //should be doing the same thing js for testing

        updateControllers();

        waitForStart();
        while(!isStopRequested()){
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());

            double PPHeading = Math.toDegrees(curpos.getHeading());//in degress
            double TurretHeading = (Turret.turret.getCurrentPosition() / ticksPerDegree);//degree the turret rotated -- in degrees
            double FinalHeading = Math.toRadians(PPHeading + TurretHeading);//total degrees to radians
            double TargetHeading = Math.toRadians(225);//radians
            //6 ticks per degree



            if(g2.a.isPressed()){
                output = pp_pid.update(TargetHeading, FinalHeading);
                //maybe???
                Turret.manual(output);
            } else if(g2.rightBumper.isPressed()){
                Turret.manual(-0.5);
            }else if(g2.leftBumper.isPressed()){
                Turret.manual(0.5);//tune
            }

            drive.setDrivePower(g1.getDrivePower().scale(1).scaleHeading(1).scaleX(1));
            drive.pinpoint.update();
            updateComponents();

            telemetry.addData("robot heading", PPHeading);
            telemetry.addData("turret heading", TurretHeading);
            telemetry.addData("final heading", FinalHeading);
            telemetry.addData("power", output);
            telemetry.addData("turret power", Turret.turret.getPower());
            //telemetry.addData("pid Active", pidActive);
            telemetry.addData("pos reached", pp_pid.pos_reached);
            telemetry.update();

            dashboardTelemetry.addData("robot heading", heading);
            dashboardTelemetry.addData("0", 0);
            dashboardTelemetry.update();
        }

    }
}
