package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class RedGateCloseNEW extends GorillabotCentral {
    enum State{
        INIT,
        PATH,
        SHOOT,
        ROW2,
        INTAKE2,
        GATEOPEN,
        GATEINTAKE,
        ROW1,
        INTAKE1,
        PATH2,
        SHOOT2,
        STOP
    }

    public static Pose2d ShootPath = new Pose2d(-14.5,17.5, Math.toRadians(90));
    public static Pose2d ShootPath2 = new Pose2d (-13.7,17.5, Math.toRadians(90));
    // CHANGE
    public static Pose2d Row2 = new Pose2d (7,18.6, Math.toRadians(90));//x9.5
    public static Pose2d Intake2 = new Pose2d (7,59, Math.toRadians(90));
    public static Pose2d GateOpen = new Pose2d (5,56, Math.toRadians(90));
    public static Pose2d IntakeGate = new Pose2d (9 ,58.5, Math.toRadians(128));//chang heading bsed on acc pos
    //8.5 ,56, Math.toRadians(128))
    public static Pose2d Row1= new Pose2d (-14,18.6, Math.toRadians(90));
    public static Pose2d Intake1 = new Pose2d (-14,55.5,Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,41.375,Math.toRadians(0));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        double IntakeIndex = 0;
        boolean goBack = false;
        boolean timerReset = false;
        boolean IndexUpdate = false;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime PathTimer = new ElapsedTime();
        double Path_wait = 2.5;

        ElapsedTime ShooterTimer = new ElapsedTime();
        double shooting = 1.5;

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double IntakePathWait = 0.9;//row 1 and 2

        ElapsedTime IntakeTimer = new ElapsedTime();
        double Intaking = 1.5; // 1.8 actual intaking

        ElapsedTime GatePathTimer = new ElapsedTime();
        double GatePathWait = 3;//Path going to gate//1.5

        ElapsedTime GatePrePath = new ElapsedTime();
        double PrePath = 2;

        ElapsedTime GateIntake = new ElapsedTime();
        double GateIntakeWait = 2.7; // 2 this accordingly-- GATEPATH + GATE Intake

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
                    if (IntakeIndex == 0) {
                        Outtake.launch_far();
                    } else {
                        Outtake.launch_close();
                        goBack = true;
                    }
                    Gate.close();
                    Intake.manual(0.6);//make sure balls dont fall off
                    Turret.limeBlue();
                    if ( PathTimer.seconds() > Path_wait) {
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
                    if (IntakeIndex == 0) {
                        Angle.close();//1735
                    } else {
                        Angle.manual(0.176);//0.175
                    }
                    Intake.exprelease(true);
                    if (ShooterTimer.seconds() > shooting) {
                        drive.resetPath();
                        drive.drivePID.pos_reached = false;
                        Turret.stop();
                        Gate.close();
                        Intake.stop();

                        if(IntakeIndex == 0){
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }else if(IntakeIndex == 1 || IntakeIndex == 2|| IntakeIndex == 3){
                            GatePrePath.reset();
                            timerReset = false;
                            IndexUpdate = true;
                            state = State.GATEOPEN;

                        }

                    }

                    break;

                case ROW2:
                    output = drive.goToPosition(Row2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(IntakeIndex != 0){
                        goBack = true;
                    }
                    if(drive.drivePID.pos_reached == true || IntakePathTimer.seconds() > IntakePathWait ){
                        drive.resetPath();
                        drive.drivePID.pos_reached = false;
                        if(goBack == false){
                            IntakeTimer.reset();
                            state = State.INTAKE2;
                        }else if(goBack == true && IntakePathTimer.seconds() > IntakePathWait){
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
                        drive.drivePID.pos_reached = false;
                        IntakeIndex = 1;
                        //PathTimer.reset();
                        GatePathTimer.reset();
                        state = State.GATEOPEN;

                    }
                    break;

                case GATEOPEN:
                    output = drive.goToPosition(GateOpen, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if ( GatePathTimer.seconds() > GatePathWait) {
                        //drive.drivePID.pos_reached == true ||
                        drive.resetPath();
                        if(goBack == false){
                            IntakePathTimer.reset();
                            state = State.ROW2;
                        }else{
                            PathTimer.reset();
                            GateIntake.reset();
                            GatePrePath.reset();
                            state = State.GATEINTAKE;//or gate intake
                        }
                    }
                    break;

                case GATEINTAKE:
                    output = drive.goToPosition(IntakeGate, 0.6, 1, 0.6, 0.1);
                    //IndexUpdate = true;
                    //fix logic here
                    if(drive.drivePID.pos_reached == true || GatePrePath.seconds() > PrePath){
                        if(IntakeIndex == 1 && IndexUpdate == true){
                            IndexUpdate = false;
                            IntakeIndex = 2 ;
                        }else if(IntakeIndex == 2 && IndexUpdate == true){
                            IndexUpdate = false;
                            IntakeIndex = 3;
                        }else if(IntakeIndex == 3 && IndexUpdate == true){
                            IntakeIndex = 4;
                        }
                        Gate.close();
                        Intake.exprelease(true);
                        if(GateIntake.seconds() > GateIntakeWait) {
                            drive.resetPath();
                            if(IntakeIndex == 4){
                                IntakePathTimer.reset();
                                state = State.ROW1;
                            }else {
                                GatePathTimer.reset();
                                state = State.PATH;
                            }
                        }
                    }
                    break;

                case ROW1:
                    output = drive.goToPosition(Row1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if( IntakePathTimer.seconds() > IntakePathWait ){
                        drive.resetPath();
                        IntakeTimer.reset();
                        state = State.INTAKE1;
                    }
                    break;

                case INTAKE1:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Intake.exprelease(true);
                    if(drive.drivePID.pos_reached == true || IntakeTimer.seconds() > Intaking){
                        drive.resetPath();
                        PathTimer.reset();
                        state = state.PATH2;
                    }
                    break;

                case PATH2:
                    output = drive.goToPosition(ShootPath2, 0.6, 1, 0.6, 0.1);
                    Outtake.launch_close();
                    Gate.close();
                    Intake.manual(0.4);
                    Turret.limeRed();
                    if(drive.drivePID.pos_reached == true || PathTimer.seconds() > Path_wait){
                        ShooterTimer.reset();
                        drive.resetPath();
                        Intake.stop();
                        state = State.SHOOT2;
                    }
                    break;

                case SHOOT2:
                    output = zero;
                    Outtake.launch_close();
                    Turret.limeRed();
                    Intake.exprelease(true);
                    Gate.open();
                    Angle.manual(0.175);
                    if(ShooterTimer.seconds() > shooting){
                       state = State.STOP;
                    }

                    break;


            }



            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("intake Index", IntakeIndex);
            telemetry.addData("outtake vel", Outtake.RflyWheel.getVelocity());
            telemetry.addData("update INdex", IndexUpdate);
            telemetry.addData("Back Up", goBack);
            telemetry.addData("Gate Intake Timer", GateIntake.seconds());
            telemetry.addData("Time reset", timerReset);
           // telemetry.addData("")
            telemetry.update();




        }

    }
}
