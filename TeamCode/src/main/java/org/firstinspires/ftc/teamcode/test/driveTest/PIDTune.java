package org.firstinspires.ftc.teamcode.test.driveTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.drive.DrivePID;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
@Config
public class PIDTune extends GorillabotCentral {
    public static Pose2d TestPath = new Pose2d(20, 30, Math.toRadians(0));
    public Pose2d currentdis = new Pose2d(0,0,0);
    public Pose2d RealTestPath = new Pose2d(0,0,0);

//change value here

    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        updateComponents();
        drive.pinpoint.recalibrateIMU();



        drive.init_pos = new Pose2d(0,0,Math.toRadians(0));
        Pose2D ppPose = Pose2d.dtoD(drive.init_pos);
        drive.pinpoint.setPosition(ppPose);
        //drive.pinpoint.resetPosAndIMU();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        Pose2d output = new Pose2d (0,0,0);

        waitForStart();
        while(!isStopRequested()){
            drive.pinpoint.update();
            currentdis = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.pose = currentdis;

            if (g1.x.isPressed()) {
                RealTestPath = TestPath;//new Pose2d (TestX, TestY, 0);
            }
            if(g1.y.isPressed()){
                RealTestPath = new Pose2d (0,0,0);
            }

            output = drive.goToPosition(RealTestPath, 0.6, 1, 0.6, 0.05);
            drive.setDrivePower(output);


            updateComponents();

            telemetry.addData("Position", currentdis);
            telemetry.addData("target", RealTestPath);
            telemetry.addData("forward P", DrivePID.forward_kP);
            telemetry.addData("strafe P", DrivePID.strafe_kP);
            telemetry.addData("rotation P", DrivePID.r_kP);
            telemetry.addData("rotation I", DrivePID.r_kI);
            //telemetry.addData("pose", drive.pose);
            //telemetry.addData("pose x", drive.pose.getX());
            // telemetry.addData("pose y", drive.pose.getY());
            //telemetry.addData("Pose H", drive.pose.getHeading());
            telemetry.addData("target", TestPath);
            telemetry.addData("Output", output);
            telemetry.addData("pos reached", drive.drivePID.pos_reached);

            telemetry.update();

            dashboardTelemetry.addData("target x", RealTestPath.getX());
            dashboardTelemetry.addData("tatrget y", RealTestPath.getY());
            dashboardTelemetry.addData("traget r", RealTestPath.getHeading());
            dashboardTelemetry.addData("current position X", currentdis.getX());
            dashboardTelemetry.addData("current pos Y", currentdis.getY());
            dashboardTelemetry.addData("current pos r", currentdis.getHeading());
            dashboardTelemetry.update();



        }





    }
}
