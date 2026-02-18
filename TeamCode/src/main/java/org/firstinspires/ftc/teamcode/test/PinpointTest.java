package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GorillabotCentral;
import org.firstinspires.ftc.teamcode.util.Pose2d;

@TeleOp
public class PinpointTest extends GorillabotCentral {
    double drive_factor = 1;
    double heading_factor = 1;
    double strafe_factor = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeComponents();
        Pose2d curpos = new Pose2d(0,0,0);

        drive.pinpoint.resetPosAndIMU();
        waitForStart();
        while(!isStopRequested()){
            //curpos = new Pose2d( curpos.getX() + pinpoint.getPosX(DistanceUnit.INCH),  curpos.getY() + pinpoint.getPosY(DistanceUnit.INCH), curpos.getHeading() + pinpoint.getHeading(AngleUnit.RADIANS));

            curpos = Pose2d.Dtod(drive.pinpoint.getPosition());
            drive.setDrivePower(g1.getDrivePower().scale(drive_factor).scaleHeading(heading_factor).scaleX(strafe_factor));
            //drive.setDrivePower(g1.getDrivePower().scale(1));
            drive.pinpoint.update();

            updateControllers();



            telemetry.addData("current Position", curpos);
            telemetry.update();

        }




    }
}
