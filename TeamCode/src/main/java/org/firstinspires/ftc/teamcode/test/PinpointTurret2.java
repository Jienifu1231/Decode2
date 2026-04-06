package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
@Config
public class PinpointTurret2 extends GorillabotCentral {
    public PID pp_pid;
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
        double output = 0;
        double heading = 0;
        double ticksPerDegree = 5.7;//in degrees

        Pose2d init_pos = new Pose2d(0,0, Math.toRadians(180));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));
        Pose2d curpos = zero;


        pp_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);//angle mode?

        updateControllers();

        waitForStart();
        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());

            double TurretHeading = Math.toRadians(Turret.turret.getCurrentPosition() / ticksPerDegree);
            double TurretAngle = Math.atan2(curpos.getY(), curpos.getX()) - curpos.getHeading();

            output = pp_pid.update(TurretAngle, TurretHeading);
            //tune

            if(g1.a.isPressed()){
                Turret.manual(output);
            } else if(g2.rightBumper.isPressed()){
                Turret.manual(-0.5);
            }else if(g2.leftBumper.isPressed()){
                Turret.manual(0.5);//tune
            }

            drive.setDrivePower(g1.getDrivePower().scale(1).scaleHeading(1).scaleX(1));
            updateComponents();

            //what is ts

            telemetry.addData("turret heading", TurretHeading);//all in radians
            telemetry.addData("Target angle", TurretAngle);
            telemetry.addData("power", output);
            telemetry.addData("turret power", Turret.turret.getPower());
            telemetry.addData("Turret pos", Turret.turret.getCurrentPosition());
            //telemetry.addData("pid Active", pidActive);
            telemetry.addData("pos reached", pp_pid.pos_reached);
            telemetry.update();

            dashboardTelemetry.addData("turret heading", TurretHeading);
            dashboardTelemetry.addData("target heading", TurretAngle);
            dashboardTelemetry.update();



        }

    }
}
