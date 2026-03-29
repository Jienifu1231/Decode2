package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class BlueGateLINEAR extends GorillabotCentral {
    enum State{
        INIT,
        PATH,
        SHOOT,
        ROW2,
        INTAKE2,
        PREGATEOPEN,
        BACKUP,
        PREGATE1,
        GATEOPEN1,
        GATEINTAKE1,
        BACKUP1,
        GATEOPEN2,
        GATEINTAKE2,
        BACKUP2,
        GATEOPEN3,
        GATEINTAKE3,
        BACKUP3,
        ROW1,
        INTAKE1,
        STOP


    }
    public Pose2d ShootPath = new Pose2d (-13.7,-17.5, Math.toRadians(-90));
    public Pose2d Row2 = new Pose2d (9.75,-18.6, Math.toRadians(-90));
    public Pose2d Intake2 = new Pose2d(9.75,-62.6, Math.toRadians(-90));
    public Pose2d BackUp = new Pose2d (9.75,-18.6, Math.toRadians(-90));
    public Pose2d PreGate = new Pose2d(6, -18.6, Math.toRadians(-90));
    public Pose2d GateOpen = new Pose2d(6,-57, Math.toRadians(-90));//change this
    public Pose2d GateIntake = new Pose2d(9 ,-58.5, Math.toRadians(-128));
    public Pose2d Row1 = new Pose2d(-14,-18.6, Math.toRadians(-90));
    public Pose2d Intake1 = new Pose2d(-14,-55.6,Math.toRadians(-90));
    public Pose2d Stop = new Pose2d (10, -17.5, Math.toRadians(90));



    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,-41.375,Math.toRadians(0));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        double GateIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime PathTimer = new ElapsedTime();
        double PathWait = 2.3;

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 1;

        ElapsedTime IntakeTimer = new ElapsedTime();
        double Intaking = 1;

        ElapsedTime ShooterTimer = new ElapsedTime();
        double Shooting = 1.5;

        ElapsedTime PreGateTimer = new ElapsedTime();
        double PreGateWait = 1;

        ElapsedTime GateOpenTimer = new ElapsedTime();
        double GateOpenWait = 0.5;

        ElapsedTime GateIntakeTimer = new ElapsedTime();
        double GateIntakeWait = 2.5;

        waitForStart();
        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state){
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
                    output = drive.goToPosition(ShootPath, 0.6, 2, 0.6, 0.2);
                    if (GateIndex == 0) {
                        Outtake.launch_far();
                    } else {
                        Outtake.launch_close();
                    }
                    Gate.close();
                    Intake.manual(0.6);//make sure balls dont fall off
                    Turret.limeBlue();
                    if ( PathTimer.seconds() > PathWait) {
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        state = State.SHOOT;
                    }
                    break;

                case SHOOT:
                    Turret.limeRed();
                    output = zero;
                    Outtake.launch_close();
                    Gate.open();
                    if (GateIndex == 0) {
                        Angle.close();//1735
                    } else {
                        Angle.manual(0.176);//0.175
                    }
                    Intake.exprelease(true);
                    if (ShooterTimer.seconds() > Shooting) {
                        drive.resetPath();
                        drive.drivePID.pos_reached = false;
                        Turret.stop();
                        Gate.close();
                        Intake.stop();

                        if(GateIndex == 0){
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }else if(GateIndex == 1 ){
                            PreGateTimer.reset();
                            state = State.PREGATE1;
                        }else if (GateIndex == 2){
                            //GateOpenTimer.reset();
                            //state = State.GATEOPEN2;
                            IntakePathTimer.reset();
                            state = State.ROW1;
                        }else if (GateIndex == 3){
                            //GateOpenTimer.reset();
                            IntakePathTimer.reset();
                            state = State.ROW1;
                        }else if (GateIndex == 4){
                            IntakePathTimer.reset();
                            state = State.ROW1;
                        }else if(GateIndex == 5){
                            state = State.STOP;
                        }

                    }
                    break;

                case ROW2:
                    output = drive.goToPosition(Row2, 0.6, 1.5,0.6, 0.1);
                    Gate.close();
                    if(drive.drivePID.pos_reached == true && IntakePathTimer.seconds() > IntakePathWait ){
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE2;
                    }

                    break;

                case INTAKE2:
                    output = drive.goToPosition(Intake2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if (IntakeTimer.seconds() > Intaking) {
                        //&& drive.drivePID.pos_reached == true
                        drive.resetPath();
                        GateIndex = 1;
                        //PathTimer.reset();
                        GateOpenTimer.reset();
                        state = State.PREGATEOPEN;

                    }
                    break;

                case PREGATEOPEN:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if( GateOpenTimer.seconds() > GateOpenWait){
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        IntakePathTimer.reset();
                        state = State.BACKUP;
                    }
                    break;

                case BACKUP:
                    output = drive.goToPosition(BackUp, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait && drive.drivePID.pos_reached == true){
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PREGATE1:
                    output = drive.goToPosition(PreGate, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(drive.drivePID.pos_reached == true && PreGateTimer.seconds() > PreGateWait){
                        drive.resetPath();
                        GateOpenTimer.reset();
                        state = State.GATEOPEN1;
                    }

                case GATEOPEN1:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if( GateOpenTimer.seconds() > GateOpenWait){
                        //drive.drivePID.pos_reached == true ||
                        drive.resetPath();
                        GateIntakeTimer.reset();
                        state = State.GATEINTAKE1;
                    }
                    break;

                case GATEINTAKE1:
                    output = drive.goToPosition(GateIntake, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if( GateIntakeTimer.seconds() > GateIntakeWait){
                        //drive.drivePID.pos_reached  == true ||
                        GateIndex = 2;
                        drive.resetPath();
                        IntakePathTimer.reset();
                        state = State.BACKUP1;
                    }
                    break;

                case BACKUP1:
                    output = drive.goToPosition(BackUp, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait ){
                        //drive.drivePID.pos_reached == true
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case GATEOPEN2:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if(drive.drivePID.pos_reached == true || GateOpenTimer.seconds() > GateOpenWait){
                        drive.resetPath();
                        GateIntakeTimer.reset();
                        state = State.GATEINTAKE2;
                    }
                    break;

                case GATEINTAKE2:
                    output = drive.goToPosition(GateIntake, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if( GateIntakeTimer.seconds() > GateIntakeWait){
                        //drive.drivePID.pos_reached  == true ||
                        GateIndex = 3;
                        drive.resetPath();
                        IntakePathTimer.reset();
                        state = State.BACKUP2;
                    }
                    break;

                case BACKUP2:
                    output = drive.goToPosition(BackUp, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait ){
                        //|| drive.drivePID.pos_reached == true
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case GATEOPEN3:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if(drive.drivePID.pos_reached == true || GateOpenTimer.seconds() > GateOpenWait){
                        drive.resetPath();
                        GateIntakeTimer.reset();
                        state = State.GATEINTAKE3;
                    }
                    break;

                case GATEINTAKE3:
                    output = drive.goToPosition(GateIntake, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if( GateIntakeTimer.seconds() > GateIntakeWait){
                        //drive.drivePID.pos_reached  == true ||
                        GateIndex = 4;
                        drive.resetPath();
                        IntakePathTimer.reset();
                        state = State.BACKUP3;
                    }
                    break;

                case BACKUP3:
                    output = drive.goToPosition(BackUp, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakePathTimer.seconds() > IntakePathWait || drive.drivePID.pos_reached == true){
                        drive.resetPath();
                        PathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case ROW1:
                    output = drive.goToPosition(Row1, 0.6, 1,0.6, 0.1);
                    Gate.close();
                    if( IntakePathTimer.seconds() > IntakePathWait ){
                        //drive.drivePID.pos_reached == true ||
                        drive.resetPath();
                        //PathTimer.reset();
                        IntakeTimer.reset();
                        state = State.INTAKE1;
                    }
                    break;

                case INTAKE1:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if (IntakeTimer.seconds() > Intaking) {
                        //&& drive.drivePID.pos_reached == true
                        drive.resetPath();
                        GateIndex = 5;
                        //PathTimer.reset();
                        GateOpenTimer.reset();
                        PathTimer.reset();
                        state = State.PATH;

                    }
                    break;

                case STOP:
                    output = drive.goToPosition(Stop, 0.6, 1, 0.6, 0.1);

                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("outtake vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("Gate Intake Timer", GateIntakeTimer.seconds());
            telemetry.addData("GateINdex", GateIndex);
            // telemetry.addData("")
            telemetry.update();


        }



    }
}
