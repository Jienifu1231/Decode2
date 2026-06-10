package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
@Config
public class ComponentTestRed extends GorillabotCentral {
    public PID lime_pid;
    public static double kp = 0.03;//0.06
    public static double ki = 0.15;
    public static double kd = 0;
    public static double sensitivity = 1;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.6;
    public static double max = 0.9;

    public double AngleOffset = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();
        Pose2d curpos = zero;

        Pose2d reloc = zero;

        Pose2d init_pos = new Pose2d(0,0, Math.toRadians(180));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        double TurretTicks =0 ;
        double tx = 0;
        double output = 0;
        boolean pidActive = false;
        double CurPep = 1;
        initLime(1);
        lime_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);
        double pos = 0;
        updateControllers();

        while(!isStarted()){
            drive.pinpoint.recalibrateIMU();
        }

        waitForStart();

        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());

            TurretTicks = Turret.turret.getCurrentPosition();

            drive.setDrivePower(g1.getDrivePower().scale(1));

            if(g1.a.isPressed()){
                Intake.reverse();
            }else if(g1.rightTrigger.moved()) {
                Intake.exprelease(false);
            //}else if(g1.leftTrigger.moved()){
                //Intake.exprelease(true);
            }else{
                Intake.stop();
            }


           if(g1.leftBumper.wasJustPressed()){
                Outtake.launch_close();
                   Angle.close();

            }else if(g1.rightBumper.wasJustPressed()){
                Outtake.launch_far();
                    Angle.far();

            }else if(g2.x.wasJustPressed()){
               Outtake.stop();
           }



            if(g1.touchpad.wasJustPressed()){
                reloc = new Pose2d(-10.25,54, Math.toRadians(90));//65.5,-64.4, Math.toRadians(180)
                drive.pinpoint.setPosition(Pose2d.dtoD(reloc));
                drive.pinpoint.recalibrateIMU();//robot has to be still?8
                drive.pinpoint.update();
                //TurretTicks = Turret.turret.getCurrentPosition();
            }//expe

            if(g2.dpadLeft.wasJustPressed()){
                AngleOffset += 0.06;
            }else if(g2.dpadRight.wasJustPressed()){
                AngleOffset -= 0.06;
            }

            //if(g2.a.isPressed(){
            if(g2.a.isPressed()){
                Turret.pinpointRed(curpos, TurretTicks, AngleOffset);
            } else if(g2.rightBumper.isPressed()){
                Turret.manual(-0.5);
            }else if(g2.leftBumper.isPressed()){
                Turret.manual(0.5);//tune
            }else{
                Turret.stop();
            }


            if(g1.leftTrigger.moved()){
                Intake.manual(1);
                Gate.open();
            }else{
                Gate.close();
            }

            if(g1.dpadDown.wasJustPressed() && g1.dpadRight.wasJustPressed()){
                Lift.manual(0.5);
                if(Lift.lift1.getPosition() == 0.5){
                    Lift.manual(1);
                }
                pos = 1;
            }else if(g1.dpadLeft.wasJustPressed()){
                pos -= 0.1;
                if(pos <= 0){
                    pos = 0;
                }
                Lift.manual(pos);
            }



            updateComponents();

            telemetry.addData("current pos", curpos);
            telemetry.addData("reloc pos", reloc);
            telemetry.addData("turret ticks", TurretTicks);
            telemetry.addData("Turret Angle Offset", AngleOffset);
            telemetry.addData("turret power", Turret.turret.getPower());
            telemetry.addData("outtake state", Outtake.target_state);
            telemetry.addData("outtake power right", Outtake.RflyWheel.getPower());
            telemetry.addData("outtake power left", Outtake.LflyWheel.getPower());
            telemetry.addData("fly wheel vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("fly wheel vel left", Outtake.LflyWheel.getVelocity());
            telemetry.addData("rfly wheel pos", Outtake.RflyWheel.getCurrentPosition());
            telemetry.addData("lfly wheel pos", Outtake.LflyWheel.getCurrentPosition());
            telemetry.addData("wheel vel just for testing", drive.backLeft.getVelocity());
            telemetry.addData("turret state", Turret.target_state);
            telemetry.addData("angle servo", Angle.outtake_angle.getPosition());
            telemetry.addData("output of far_pid", Outtake.far_pid.out);
            telemetry.update();

            dashboardTelemetry.addData("target vel", -Outtake.vel);
            dashboardTelemetry.addData("cur vel right", Outtake.RflyWheel.getVelocity());
            dashboardTelemetry.addData("cur vel left", Outtake.LflyWheel.getVelocity());
            dashboardTelemetry.addData("tx", tx);
            dashboardTelemetry.addData("lime target", 0);
            dashboardTelemetry.addData("turret target ang", Turret.TurretAngle);
            dashboardTelemetry.addData("turret current heading", Turret.TurretHeading);
            dashboardTelemetry.addData("power", -1000 * Outtake.RflyWheel.getPower());
            dashboardTelemetry.addData("error", Outtake.error);
            dashboardTelemetry.update();
        }



    }
}
