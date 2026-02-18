package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
@TeleOp
public class ServoTest extends GorillabotCentral {


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();
        updateControllers();

        double pos = 0;//just to test // used to be 0

        waitForStart();
        while(!isStopRequested()) {

            if (g1.a.wasJustPressed()) {
                pos += 0.1;
                if(pos >= 1){
                    pos = 1;
                }

            }

            if(g1.b.wasJustPressed()){
                pos -= 0.1;
                if(pos <= 0){
                    pos = 0;
                }

            }

           //Gate.manual(pos);
            //Lift.manual(pos);
            Angle.manual(pos);

               updateComponents();



                    telemetry.addData("servo position ", pos);
                    telemetry.addData("acutall pos Lift 1", Lift.lift1.getPosition());
                    telemetry.addData("acutaul pos lift 2", Lift.lift2.getPosition());
                    telemetry.addData("actual pos lift 3", Lift.lift3.getPosition());
                    //telemetry.addData("actual pos gate", Gate.Gate1.getPosition());
                    telemetry.addData("actuall pos angle", Angle.outtake_angle.getPosition());
                    telemetry.update();


                }

            }
        }

