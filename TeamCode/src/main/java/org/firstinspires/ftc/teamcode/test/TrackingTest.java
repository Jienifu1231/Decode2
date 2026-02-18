package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.PID;

@TeleOp
@Config
public class TrackingTest extends GorillabotCentral {

    public PID lime_pid;
    public static double kp = 0.03;//0.06
    public static double ki = 0.15;
    public static double kd = 0;
    public static double sensitivity = 3;
    public static double integral_sum_limit = 10;
    public static double normn_vel = 0.5;
    public static double max = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        double tx = 0;
        double output = 0;
        boolean pidActive = false;



        initLime(0);

        lime_pid = new PID(kp, ki, kd, sensitivity, integral_sum_limit, normn_vel, max,false);
        updateControllers();


        waitForStart();
        while(!isStopRequested()){

            limeResult = limelight.getLatestResult();

            if(limeResult != null && limeResult.isValid()){
               pidActive = true;
            }else{
                pidActive = false;
            }



            if(g2.a.isPressed()&& pidActive == true) {//&& pidActive == true
                //initLime(1);
                tx = limeResult.getTx();
                output = lime_pid.update(0, tx);//could be negativ
                Turret.manual(-output);
                //Turret.limeRed()
                // }else if (g2.y.isPressed()&& pidActive == true){
                //initLime(0);
                //tx = limeResult.getTx();
                //output = lime_pid.update(0, tx);//could be negativ
                //Turret.manual(-output);
            } else if(g2.rightBumper.isPressed()){
                Turret.manual(0.5);
            }else if(g2.leftBumper.isPressed()){
                Turret.manual(-0.5);//tune
            }else{
                Turret.stop();
            }

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
            //Outtake.stop();
            //}



            if(g2.b.isPressed()){
                Gate.open();
            }else{
                Gate.close();
            }

            updateComponents();


            telemetry.addData("tx", Turret.tx);
            telemetry.addData("power", output);
            telemetry.addData("turret power", Turret.turret.getPower());
            telemetry.addData("lime result", limeResult);
            telemetry.addData("pid Active", pidActive);
            telemetry.addData("pos reached", lime_pid.pos_reached);
            telemetry.update();

            dashboardTelemetry.addData("tx", tx);
            dashboardTelemetry.addData("0", 0);
            dashboardTelemetry.update();


        }
    }
}