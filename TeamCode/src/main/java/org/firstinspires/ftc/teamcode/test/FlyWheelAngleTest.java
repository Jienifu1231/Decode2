package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;

@TeleOp
public class FlyWheelAngleTest extends GorillabotCentral {

    @Override
    public void runOpMode() throws InterruptedException {

            initializeComponents();
            updateControllers();

            waitForStart();
            ElapsedTime gateTimer = new ElapsedTime();
            double gateWait = 2;
            double pos = 0;

            while(!isStopRequested()) {

                drive.setDrivePower(g1.getDrivePower().scale(1));

                if (g1.a.isPressed()) {
                    Intake.reverse();
                } else if (g1.rightTrigger.moved()) {
                    Intake.exprelease(false);
                    //}else if(g1.leftTrigger.moved()){
                    //Intake.exprelease(true);
                } else {
                    Intake.stop();
                }


                if (g1.leftBumper.wasJustPressed()) {
                    Outtake.launch_close();
                    Angle.close();

                } else if (g1.rightBumper.wasJustPressed()) {
                    Outtake.launch_far();
                    Angle.far();

                }else if(g2.rightBumper.wasJustPressed()){
                    Outtake.launch_far();
                    if(Outtake.curvel <= (Outtake.vel - 300)){
                        Angle.manual(0.15);
                    }

                }else if(g2.leftBumper.wasJustPressed()) {
                    Outtake.launch_bang();
                    Angle.manual(0.28);
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
                //intake update in updateComponents()

                telemetry.addData("outtake state", Outtake.target_state);
                telemetry.addData("outtake power right", Outtake.RflyWheel.getPower());
                telemetry.addData("outtake power left", Outtake.LflyWheel.getPower());
                telemetry.addData("outtake power variable", Outtake.power);
                telemetry.addData("fly wheel vel", Outtake.RflyWheel.getVelocity());
                telemetry.addData("fly wheel vel left", Outtake.LflyWheel.getVelocity());
                telemetry.addData("rfly wheel pos", Outtake.RflyWheel.getCurrentPosition());
                telemetry.addData("lfly wheel pos", Outtake.LflyWheel.getCurrentPosition());
                telemetry.addData("gate position", Gate.Gate1.getPosition());
                telemetry.addData("gateOpen boolean", Intake.gateOpen);
                telemetry.addData("angle servo", Angle.outtake_angle.getPosition());
                telemetry.addData("output of far_pid", Outtake.far_pid.out);
                telemetry.update();

                dashboardTelemetry.addData("target vel", -Outtake.vel);
                dashboardTelemetry.addData("cur vel right", Outtake.RflyWheel.getVelocity());
                dashboardTelemetry.addData("cur vel left", Outtake.LflyWheel.getVelocity());
                dashboardTelemetry.update();
            }



        }
    }

