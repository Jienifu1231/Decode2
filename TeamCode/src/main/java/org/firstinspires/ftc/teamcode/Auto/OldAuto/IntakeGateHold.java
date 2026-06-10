package org.firstinspires.ftc.teamcode.Auto.OldAuto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@Autonomous
@Config
public class IntakeGateHold extends GorillabotCentral {
    enum State{
        INIT,
        PATH,
        SHOOT,
        INTAKEPATH2,
        INTAKE2,
        BACKUP,
        GATEHOLD,
        GATEINTAKE,
        INTAKEPATH1,
        INTAKE1,
        STOP
    }

    public static Pose2d ShootPath = new Pose2d(-13,17.5, Math.toRadians(90));
    public static Pose2d IntakePath1 = new Pose2d(-14,18.6, Math.toRadians(90));
    public static Pose2d Intake1 = new Pose2d (-14,54,Math.toRadians(90));
    public static Pose2d IntakePath2 = new Pose2d(9.75,18.6, Math.toRadians(90));
    public static Pose2d Intake2 = new Pose2d(9.75,62.6, Math.toRadians(90));
    public static Pose2d BackUp =  new Pose2d(-14,40, Math.toRadians(90));
    public static Pose2d GateHold = new Pose2d (-4,58, Math.toRadians(90));

    public static Pose2d GateInake = new Pose2d (-2, 62, Math.toRadians(120));
    public static Pose2d Stop =  new Pose2d (3, 18.6, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        State state = State.INIT;
        initializeComponents();

        Pose2d init_pos = new Pose2d(-65,41.375,Math.toRadians(0)); // change this
        drive.pinpoint.setPosition(Pose2d.dtoD(init_pos));

        Pose2d output = zero;
        Pose2d curpos = zero;

        int IntakeIndex = 0;

        ElapsedTime timer = new ElapsedTime();
        double init_wait = 0.3;

        ElapsedTime ShootPathTimer = new ElapsedTime();
        double shootPathWait = 2;

        ElapsedTime ShooterTimer  = new ElapsedTime();
        double shooting = 1.2;

        ElapsedTime IntakePathTimer = new ElapsedTime();
        double intakePathWait = 1;

        ElapsedTime IntakingTimer = new ElapsedTime();
        double Intaking = 2;

        ElapsedTime GateHoldTimer = new ElapsedTime();
        double GateHolding = 3;

        double TurretTicks = 0;

        double AngleOffset = 0;

        waitForStart();
        while(!isStopRequested()){
            drive.pinpoint.update();
            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = curpos;
            TurretTicks = Turret.turret.getCurrentPosition();

            switch(state) {
                case INIT:
                    output = zero;
                    Intake.stop();
                    Outtake.stop();
                    Gate.close();
                    Turret.stop();
                    if (timer.seconds() > init_wait) {
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;

                case PATH:
                    output = drive.goToPosition(ShootPath, 0.6, 1, 0.6, 0.1);//0.4
                    Turret.pinpointRed(curpos, TurretTicks, AngleOffset);
                    Outtake.launch_close();
                    Gate.close();
                    Intake.exprelease(true);

                    if (IntakeIndex == 0) {
                        shootPathWait = 2.8;
                    }
                    if (ShootPathTimer.seconds() > shootPathWait) {
                        //drive.drivePID.pos_reached == true ||
                        drive.resetPath();
                        ShooterTimer.reset();
                        Intake.stop();
                        state = State.SHOOT;

                    }
                    break;

                case SHOOT:
                    output = zero;

                    Gate.open();
                    Intake.exprelease(true);

                    if (IntakeIndex == 0) {
                        Outtake.launch_far();//far
                        Angle.close();
                    } else {
                        Angle.manual(0.1745);
                        Outtake.launch_close();
                    }

                    if (ShooterTimer.seconds() > shooting) {
                        //Outtake.stop();
                        Intake.stop();
                        Turret.stop();
                        IntakePathTimer.reset();
                        GateHoldTimer.reset();
                        if (IntakeIndex == 0) {
                            drive.resetPath();
                            state = State.INTAKEPATH2;
                        }

                        if (IntakeIndex == 1) {
                            drive.resetPath();
                            state = State.GATEHOLD;
                        }
                        /*if(IntakeIndex == 2){
                            drive.resetPath();
                            state = State.INTAKEPATH3;
                        }

                         */
                        if (IntakeIndex == 2) {
                            drive.resetPath();
                            state = State.GATEHOLD;
                        }
                        if(IntakeIndex ==3 ){
                            drive.resetPath();
                            state = State.INTAKEPATH1;
                        }

                        if(IntakeIndex == 4){
                            drive.resetPath();
                            state = State.STOP;
                        }
                    }
                    break;

                case INTAKEPATH2:
                    output = drive.goToPosition(IntakePath2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (IntakePathTimer.seconds() > intakePathWait) {
                        //drive.drivePID.pos_reached == true &&
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE2;
                    }
                    break;

                case INTAKE2:
                    output = drive.goToPosition(Intake2, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (IntakingTimer.seconds() <= Intaking) {
                        Intake.exprelease(true);
                        IntakePathTimer.reset();
                    } else {
                        output = drive.goToPosition(IntakePath2, 0.4, 1, 0.4, 0.1);
                        if (IntakePathTimer.seconds() > intakePathWait) {
                            Intake.stop();
                            IntakeIndex = 1;
                            AngleOffset = -0.06;
                            ShootPathTimer.reset();
                            drive.resetPath();
                            state = State.BACKUP;
                        }
                    }
                    break;

                case BACKUP:
                    output = drive.goToPosition(BackUp, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if (ShootPathTimer.seconds() >= 1) {
                        drive.resetPath();
                        GateHoldTimer.reset();
                        state = State.GATEHOLD;
                    }
                    break;

                case GATEHOLD:
                    output = drive.goToPosition(GateHold, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_close();
                    if (GateHoldTimer.seconds() >= GateHolding) {
                        drive.resetPath();
                        ShootPathTimer.reset();
                        state = State.GATEINTAKE;
                    }
                    break;

                case GATEINTAKE:
                    output = drive.goToPosition(GateInake, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    if(ShootPathTimer.seconds() >= 2){
                        if(IntakeIndex == 2){
                            IntakeIndex = 3;
                        }else {
                            IntakeIndex = 2;
                        }
                        drive.resetPath();
                        ShootPathTimer.reset();
                        state = State.PATH;
                    }
                    break;



                case INTAKEPATH1:
                    output = drive.goToPosition(IntakePath1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_close();
                    if (IntakePathTimer.seconds() > intakePathWait) {
                        //drive.drivePID.pos_reached == true
                        drive.resetPath();
                        IntakingTimer.reset();
                        state = State.INTAKE1;
                    }
                    break;

                case INTAKE1:
                    output = drive.goToPosition(Intake1, 0.6, 1, 0.6, 0.1);
                    Gate.close();
                    Outtake.launch_close();
                    if (IntakingTimer.seconds() <= Intaking) {
                        Intake.exprelease(true);
                    } else {
                        Intake.stop();
                        //shootIndex = 0;
                        IntakeIndex = 4;
                        AngleOffset = -0.06;
                        ShootPathTimer.reset();
                        drive.resetPath();
                        state = State.BACKUP;
                    }
                    break;





                case STOP:
                    output = drive.goToPosition(Stop, 0.6, 1, 0.6, 0.1);
                    Turret.reset(Turret.turret.getCurrentPosition());
                    break;

            }

            drive.setDrivePower(output);
            updateComponents();

            telemetry.addData("state", state);
            telemetry.addData("current pos", curpos);
            telemetry.addData("Intake state", Intake.target_state);
            telemetry.addData("Outtake state", Outtake.target_state);
            telemetry.addData("Outtake ticks", Outtake.RflyWheel.getVelocity());
            telemetry.addData("intake index ", IntakeIndex );
            telemetry.addData("drivePID pos reached?", drive.drivePID.pos_reached);
            telemetry.addData("Turret power", Turret.turret.getPower());
            telemetry.addData("Turret Ticks", TurretTicks);
            telemetry.addData("Turret State", Turret.target_state);
            telemetry.update();


            dashboardTelemetry.addData("target vel", -Outtake.vel);
            dashboardTelemetry.addData("cur vel right", Outtake.RflyWheel.getVelocity());
            dashboardTelemetry.addData("cur vel left", Outtake.LflyWheel.getVelocity());
            dashboardTelemetry.addData("Turret target", Turret.TurretAngle);
            dashboardTelemetry.addData("Turret current angle", Turret.TurretHeading);
            dashboardTelemetry.update();



        }

    }





}

