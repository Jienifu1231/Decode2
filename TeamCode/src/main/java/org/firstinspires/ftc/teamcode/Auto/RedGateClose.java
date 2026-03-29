package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.OldAuto.CloseRedAuto;
import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class RedGateClose extends GorillabotCentral {
    enum State{
        INIT,
        PATH,
        SHOOT,
        ROW2,
        INTAKE2,
        GATEOPEN,
        GATEINTAKE,
        ROW1,
        INTAKE1
    }

    public static Pose2d ShootPath = new Pose2d(-13.7,17.5, Math.toRadians(90));
    public static Pose2d Row2 = new Pose2d (9.5,18.6, Math.toRadians(90));
    public static Pose2d Intake2 = new Pose2d (9.5,59, Math.toRadians(90));
    public static Pose2d GateOpen = new Pose2d (8,50, Math.toRadians(90));
    public static Pose2d IntakeGate = new Pose2d (8.5 ,59.5, Math.toRadians(128));//chang heading bsed on acc pos
//x: (8 ,58.5, Math.toRadians(123))
    public static Pose2d Row1= new Pose2d (-14,18.6, Math.toRadians(90));
    public static Pose2d Intake1 = new Pose2d (-14,55.5,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,41.375,Math.toRadians(0)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        boolean GateIntakeIndex = false;
        boolean goBack = false;

        int intakeIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime PathTimer = new ElapsedTime();
        double Path_wait = 2.5;

        ElapsedTime ShooterTimer = new ElapsedTime();
        double shooting = 1.5; //change this if too much time/too less

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 0.8;

        ElapsedTime IntakeTimer = new ElapsedTime();
        double Intaking = 1.8;

        ElapsedTime GatePathTimer = new ElapsedTime();
        double GatePathWait = 0.8;

        ElapsedTime GateIntake = new ElapsedTime();
        double GateIntakeWait = 3; // 2.7  this accordingly

        ElapsedTime Row1Timer = new ElapsedTime();
        double Row1Wait = 0.6;

        ElapsedTime Row1Intake = new ElapsedTime();
        double Row1IntakeWait = 1.3;

        waitForStart();
        while (!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state) {
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    Turret.stop();
                    if (timer.seconds() > init_wait) {
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                    output = drive.goToPosition(ShootPath, 0.6, 3, 0.6, 0.2);
                    //sens 3
                    if (intakeIndex == 0) {
                        Outtake.launch_far();
                    } else {
                        Outtake.launch_close();
                    }
                    Gate.close();
                    Intake.exprelease(true);
                    Turret.limeBlue();
                    if (drive.drivePID.pos_reached == true && PathTimer.seconds() > Path_wait) {
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        IntakeTimer.reset();
                        state = State.SHOOT;

                    }
                    break;

                case SHOOT:
                    Turret.limeRed();
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    if (intakeIndex == 0) {
                        Angle.manual(0.1735);
                    } else {
                        Angle.manual(0.175);//0.1745
                    }
                    Intake.exprelease(true);
                    if (ShooterTimer.seconds() > shooting) {
                        drive.resetPath();
                        Turret.stop();
                        Gate.close();
                        Intake.stop();
                        IntakePathTimer.reset();
                        if (intakeIndex == 0) {
                            state = State.ROW2;
                        } else if (GateIntakeIndex) {
                            Row1Timer.reset();
                            state = State.ROW1;
                        } else if (intakeIndex == 1 || intakeIndex == 2 || intakeIndex == 3 || intakeIndex == 4) {
                            GatePathTimer.reset();
                            intakeIndex = 2;
                            state = State.GATEOPEN;
                        }
                    }
                    break;

                case ROW2:
                    output = drive.goToPosition(Row2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (IntakePathTimer.seconds() > IntakePathWait && drive.drivePID.pos_reached == true) {
                        drive.resetPath();
                        IntakeTimer.reset();
                        if(goBack == false) {
                            state = State.INTAKE2;
                        }else{
                            PathTimer.reset();
                            state = State.PATH;
                        }
                    }
                    break;


                case INTAKE2:
                    output = drive.goToPosition(Intake2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if (IntakeTimer.seconds() > Intaking) {
                        //&& drive.drivePID.pos_reached == true
                        drive.resetPath();
                        intakeIndex = 1;
                        //PathTimer.reset();
                       GatePathTimer.reset();
                        state = State.GATEOPEN;

                    }
                    break;

                case GATEOPEN:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (drive.drivePID.pos_reached == true || GatePathTimer.seconds() > GatePathWait) {
                        drive.resetPath();
                        if (goBack == false) {
                            goBack = true;
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }
                        if (intakeIndex == 1 && goBack == true) {
                            PathTimer.reset();
                            drive.resetPath();
                            state = State.PATH;
                        } else {
                            GateIntake.reset();
                            state = State.GATEINTAKE;
                        }
                    }
                    break;

                case GATEINTAKE:
                    output = drive.goToPosition(IntakeGate, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if (drive.drivePID.pos_reached == true && GateIntake.seconds() > GateIntakeWait) {
                        if (intakeIndex == 2) {
                            intakeIndex = 3;
                        }else if(intakeIndex == 3){
                            intakeIndex = 4;
                            //GateIntakeIndex = true;
                        }else if(intakeIndex == 3){
                            intakeIndex = 4;
                            //GateIntakeIndex = true;
                        }


                        if(intakeIndex == 4){
                            GateIntakeIndex = true;
                        }
                        drive.resetPath();
                        PathTimer.reset();
                        //Intake.manual(0.5);
                        state = State.PATH;
                    }
                    break;

                case ROW1:
                    output = drive.goToPosition(Row1, 0.6, 1, 0.45, 0.1);
                    Gate.close();
                    if (Row1Timer.seconds() > Row1Wait || drive.drivePID.pos_reached) {
                        drive.resetPath();
                        Row1Intake.reset();
                        state = State.INTAKE1;
                    }
                    break;

                case INTAKE1:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.4, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if (Row1Intake.seconds() > Row1IntakeWait) {
                        //&& drive.drivePID.pos_reached == true
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;


                    }
            }


            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("intake Index", intakeIndex);
            telemetry.addData("outtake vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("Gate Intake Index", GateIntakeIndex);
            telemetry.addData("Gate Intake Timer", GateIntake.seconds());
            telemetry.update();


        }
    }
}
