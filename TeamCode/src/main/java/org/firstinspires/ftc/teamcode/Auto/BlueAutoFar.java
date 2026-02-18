package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.BlueAutoFar.State.INIT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class BlueAutoFar extends GorillabotCentral {
enum State{
    INIT,
    PATH,
    SHOOT,
    INTAKEPATH,
    INTAKE,
    INTAKEPATH2,
    INTAKE2,
    INTAKEPATH3,
    INTAKE3,
    STOP,
}

   public static Pose2d ShootPath1 = new Pose2d(58.75, -16.6, Math.toRadians(201));
public static Pose2d ShootPath1Turn = new Pose2d(58.75, -16.6, Math.toRadians(201));
// change this later according to the field
    public static Pose2d IntakePath = new Pose2d(35.75,-18.6, Math.toRadians(180));//36.75
    public static Pose2d IntakePath1 = new Pose2d(37.75,-62.6,Math.toRadians(-90));//acutal intaking

    public static Pose2d IntakePath2 = new Pose2d (14.75,-18.6, Math.toRadians(180));
    //change this
    public static Pose2d Intake2 = new Pose2d (14.75,-62.6, Math.toRadians(-90));
    public static Pose2d IntakePath3 = new Pose2d(-8,-18.6, Math.toRadians(180));
    public static Pose2d Intake3 = new Pose2d(-8,-62.6,Math.toRadians(-90));
    public static Pose2d stop = new Pose2d(0, -22, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(64.75,-18.6,Math.toRadians(180));
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos)); // set it to the field centric
        //drive.pinpoint.resetPosAndIMU();


        Pose2d output = zero;
        Pose2d curpos = new Pose2d(0, 0, 0);

        int shootIndex = 0;// how much balls we shot
        int IntakeIndex = 0;


        ElapsedTime timer = new ElapsedTime();//timer for init state
        double init_wait = 0.2;//time in init state

        ElapsedTime TotalShootTimer = new ElapsedTime();//timer for one shot
        double Shooting = 1;//1.1     --- total time taking for shooting
        double first_shot = 0.7;//1  ---- the time in between shots -- bad name
        double intake_Wait = 0.7;//1.5  --- time waiting for the first shot -- should be longer
        ElapsedTime IntakeWati = new ElapsedTime();//traking how much time the intake is moving/transfering

        ElapsedTime ShootPathTimer = new ElapsedTime();//shoot path timer
        double path_wait = 2;//time taken to go to the spot for shooting
        //CHANGE THIS TO POS_REACHED
        double intake_path = 1;//1.5 --- time intaking/going forward

        ElapsedTime pathTimer = new ElapsedTime();//timer for finishing intake path -- CHECK AND PRELACE WITH POS_REACHED
        //check use this if got stucked in a path


        ElapsedTime intakeTimer = new ElapsedTime();//timer for actuall intaking
        double intake_time = 2;//2.8


        boolean finished = false;



        boolean shootReached = false;
        boolean intakeReached = false;
        boolean intake_reached = false;


        waitForStart();
        timer.reset();

        while (!isStopRequested()) {
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;

            switch(state){
                case INIT:
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    if(timer.seconds() > init_wait){
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                   output = drive.goToPosition(ShootPath1, 0.47, 1, 0.4, 0.1);
                    Outtake.launch_far();
                   Gate.close();
                   Intake.exprelease(true);

                    if(ShootPathTimer.seconds() > path_wait){//change stuff here to make sure PID is right -- ki
                        //drive.drivePID.pos_reached == true &&
                        //ShootPathTimer.seconds() > path_wait
                        drive.resetPath();
                        Intake.stop();
                        TotalShootTimer.reset();
                        IntakeWati.reset();
                        state = State.SHOOT;
                        //check I term for not being able to reach the state
                    }
                    break;

                case SHOOT:
                    Turret.limeBlue();
                    output = zero;
                    Outtake.launch_far();
                        if(TotalShootTimer.seconds() < Shooting) {
                            //Outtake.launch_far();
                            Angle.far();
                            Gate.open();
                            if (IntakeWati.seconds() > first_shot) {
                               Intake.exprelease(true);
                           } else{
                               Intake.stop();
                           }
                           //first shot -- wait longer
                            // between second and third shot, wait

                        } else {
                        shootIndex++;
                        TotalShootTimer.reset();
                        IntakeWati.reset();
                         }

                      if(shootIndex == 3) {
                            //Outtake.stop();
                            Intake.stop();
                            Turret.stop();
                            pathTimer.reset();
                            if(IntakeIndex == 0) {
                                drive.resetPath();
                                drive.drivePID.pos_reached = false;
                                state = State.INTAKEPATH;
                            }
                            if(IntakeIndex ==1 ){
                                drive.resetPath();
                                state = State.INTAKEPATH2;
                            }
                            if(IntakeIndex == 2){
                                drive.resetPath();
                                state = State.INTAKEPATH3;
                            }
                        }

                    break;

                case INTAKEPATH:
                    output = drive.goToPosition(IntakePath, 0.4, 1, 0.4, 0.05);
                    Gate.close();
                    if(pathTimer.seconds() > intake_path){
                        //changed it here to see if it skip state if it did then only timer
                        drive.resetPath();
                        intakeTimer.reset();
                        state = State.INTAKE;
                    }
                    break;

                case INTAKE:
                    Gate.close();
                    output = drive.goToPosition(IntakePath1, 0.4, 1, 0.4, 0.05);
                    if(intakeTimer.seconds() <= intake_time){
                        Intake.exprelease(false);
                    }
                    if(intakeTimer.seconds() > intake_time){
                        //Intake.stop();
                        intake_reached = true;
                        shootIndex = 0;
                        IntakeIndex = 1;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                        // drive.drivePID.pos_reached
                    }
                    break;

                case INTAKEPATH2:
                    Gate.close();
                    output = drive.goToPosition(IntakePath2, 0.4, 1, 0.4, 0.1);
                    if(pathTimer.seconds() > path_wait){
                        drive.resetPath();
                        intakeTimer.reset();
                        state = State.INTAKE2;
                    }
                    break;

                case INTAKE2:
                    Gate.close();
                    output = drive.goToPosition(Intake2, 0.4, 1, 0.4, 0.1);
                    if(intakeTimer.seconds() <= intake_time){
                        Intake.exprelease(false);
                    }else{
                        //Intake.stop();
                        shootIndex = 0;
                        IntakeIndex = 2;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.PATH;
                    }
                    //add from here if everything before that works well
                    break;
                case INTAKEPATH3:
                    Gate.close();
                    output = drive.goToPosition(IntakePath3, 0.4, 1, 0.4, 0.1);
                    if(drive.drivePID.pos_reached && pathTimer.seconds() > path_wait){
                        drive.resetPath();
                        intakeTimer.reset();
                        state = State.INTAKE3;
                    }
                    break;
                case INTAKE3:
                    Gate.close();
                    output = drive.goToPosition(Intake3, 0.4, 1, 0.4, 0.1);
                    if(intakeTimer.seconds() <= intake_time){
                        Intake.exprelease(false);
                    }else{
                        //Intake.stop();
                        shootIndex = 0;
                        IntakeIndex =3;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.STOP;
                    }
                    break;

                case STOP:
                    output = drive.goToPosition(stop, 0.4, 1, 0.4, 0.1);

                    break;
            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("shoot Index", shootIndex);
            telemetry.addData("shootReached?", shootReached);
            telemetry.addData("intake reached?", intakeReached);
            telemetry.addData("total shooter timer", TotalShootTimer.seconds());
            telemetry.addData("intake time", IntakeWati.seconds());
            telemetry.addData("pid Pos reached", drive.drivePID.pos_reached);
            telemetry.addData("Intake index ", IntakeIndex );
            telemetry.addData("power", Outtake.RflyWheel.getPower());
            telemetry.addData("limelight", Turret.limeResult);

            telemetry.update();

            dashboardTelemetry.addData("state", state);
            dashboardTelemetry.addData("current pos", curpos);
            dashboardTelemetry.addData("Outtake state", Outtake.target_state);
            dashboardTelemetry.addData("Intake state", Intake.target_state);
            dashboardTelemetry.addData("shoot Index", shootIndex);
            dashboardTelemetry.addData("shootReached?", shootReached);
            dashboardTelemetry.addData("intake reached?", intakeReached);
            dashboardTelemetry.update();

        }


    }
}
