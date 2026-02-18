package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.GorillabotCentral;

public class AngleAdjust extends GorillabotCentral {


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();
        updateControllers();

        waitForStart();
        while(!isStopRequested()){

            if(g1.a.wasJustPressed()){
                Turret.limeRed();
                Outtake.launch_far();
                Angle.far();
                if(Outtake.curvel <= (Outtake.vel-100)){
                    Angle.manual(0.2);
                }
           }else{
                Outtake.stop();
                Turret.stop();
            }

            updateComponents();

            telemetry.addData("Outtake current vel", Outtake.curvel);
            telemetry.addData("angle pos", Angle.outtake_angle.getPosition());
            telemetry.update();
        }

    }
}
