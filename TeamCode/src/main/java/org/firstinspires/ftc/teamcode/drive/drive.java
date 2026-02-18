package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.util.MathFunctions.lineCircleIntersection;
import static org.firstinspires.ftc.teamcode.util.MathFunctions.linePointDistance;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.ConstantAccelMath;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.WayPoint;

import java.util.ArrayList;
import java.util.List;

public class drive {

    //Make sure you change lots of things

    public enum State {
        IDLE,
        MOVING,
        MIDPOINT,
    }





    public State state = State.IDLE;

    public DcMotorEx frontLeft, frontRight, backLeft, backRight, RflyWheel, LflyWheel;
    public GoBildaPinpointDriver pinpoint;
    private final ConstantAccelMath constantAccelMath = new ConstantAccelMath();
    public DrivePID drivePID;
    public IMU imu;

    private Pose2d origin = new Pose2d(0, 0, 0);

    public Pose2d pose, vel, acc;
    public Pose2d init_pos = origin;
    private Pose2d lastPose = origin;
    private Pose2d lastVel = origin;

    public drive(HardwareMap hardwareMap){
        frontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        frontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        backLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        backRight = (DcMotorEx) hardwareMap.dcMotor.get("br");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        drivePID = new DrivePID();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-4.81495, 0.04, DistanceUnit.INCH);//0.04
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,//xencoder -- traking foward+
                GoBildaPinpointDriver.EncoderDirection.REVERSED//yencoder -- traking strafe -- left +
        );

        initialize();
       // initIMU(hardwareMap);
        //change
    }

    //new
    public double getRVelocity(){
        return RflyWheel.getVelocity();
    }

    public double getLVelocity(){
        return LflyWheel.getVelocity();
    }

    public void initialize(){

        lastTime = 0;
        last_left_encoder = 0;
        last_right_encoder = 0;
        last_center_encoder = 0;
        //if pinpoint then no

        pose = new Pose2d(0, 0, 0);
        vel = new Pose2d(0, 0, 0);
        acc = new Pose2d(0, 0, 0);
    }

    public void initIMU(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();
        imu_offset = 0;
        //not sure what that is
    }

    //TODO: Adjust these values for the driver
    double x_drive_weight = 1.1;
    double y_drive_weight = -1;
    double r_drive_weight = 0.7;
    //change these this year as well

    boolean useIMU = false;
    public double imu_offset = 0;

    double lastTime = 0;
    double last_left_encoder, last_right_encoder, last_center_encoder;
    Pose2d lastFollowPoint = new Pose2d(0, 0, 0);
    double lastCurrDist = 1000;
    boolean justInRange = false;

    public ElapsedTime waitTimer = new ElapsedTime();

    public double strafePolynomial(double x){ return 0.6*Math.pow(x, 5)-1.6*Math.pow(x, 3)+2*x; }
    public double slowFunction(double x){ return (x > 0) ? 0.4003056 * Math.atan(6 * x - 3) + 0.5 : -0.4003056 * Math.atan(- 6 * x - 3) - 0.5; }
    public boolean polynomialMode = false;
    public boolean slowTanMode = false;
    double x_power, y_power;

    public void checkIMU(){
        useIMU = true;
    }

    //public void setIMUYaw(double new_yaw){ imu_offset = new_yaw - getIMUYaw(); }

    //public double getIMUYaw(){ return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }

    public void setPose(Pose2d newPos){
        pose = newPos;
        //setIMUYaw(newPos.getHeading());
    }

    public void setDrivePower(Pose2d power){
        x_power = polynomialMode ? strafePolynomial(power.getX()) : power.getX();

        x_power = slowTanMode ? slowFunction(power.getX()) : power.getX();

        y_power = slowTanMode ? slowFunction(power.getY()) : power.getY();

        double fl = x_drive_weight * x_power - y_drive_weight * y_power + r_drive_weight * power.getHeading();
        double fr = x_drive_weight * x_power + y_drive_weight * y_power + r_drive_weight * power.getHeading();
        double bl = -x_drive_weight * x_power - y_drive_weight * y_power + r_drive_weight * power.getHeading();
        double br = -x_drive_weight * x_power + y_drive_weight * y_power + r_drive_weight * power.getHeading();

        //double fl = x_drive_weight * x_power + y_drive_weight * y_power + r_drive_weight * power.getHeading();//+x
        //double fr = -x_drive_weight * x_power + y_drive_weight * y_power - r_drive_weight * power.getHeading();//-x
        //double bl = -x_drive_weight * x_power + y_drive_weight * y_power + r_drive_weight * power.getHeading();//+x
        //double br = x_drive_weight * x_power + y_drive_weight * y_power - r_drive_weight * power.getHeading();//-

        double d = Math.max(1, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl/d);
        frontRight.setPower(fr/d);
        backLeft.setPower(bl/d);
        backRight.setPower(br/d);
        //mechanum wheel code
    }

    public void updatePose(){
        long currTime = System.nanoTime();
        double t = (currTime-lastTime)/1.0E9;
        lastTime = currTime;

        List<Double> EncoderPos = OdometryPods.getWheelPositions();
        // change if its really pinpoint

        double delta_left_encoder = EncoderPos.get(0) - last_left_encoder;
        double delta_right_encoder = EncoderPos.get(1) - last_right_encoder;
        double delta_center_encoder = EncoderPos.get(2) - last_center_encoder;

        last_left_encoder = EncoderPos.get(0);
        last_right_encoder = EncoderPos.get(1);
        last_center_encoder = EncoderPos.get(2);

        //This is the heading because the heading is proportional to the difference between the left_hang and right_hang wheel.
        double delta_theta = (delta_right_encoder - delta_left_encoder)/OdometryPods.LATERAL_DISTANCE;
        //This gives us deltaY because the back minus theta*R is the amount moved to the left_hang minus the amount of movement in the back encoder due to change in heading
        double delta_y = delta_center_encoder - delta_theta*OdometryPods.FORWARD_OFFSET;
        //This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double delta_x = -(delta_right_encoder*OdometryPods.CENTER_LEFT_OFFSET + delta_left_encoder*OdometryPods.CENTER_RIGHT_OFFSET)/(OdometryPods.LATERAL_DISTANCE);

        // constant accel
        Pose2d relDelta = new Pose2d(delta_x, delta_y, delta_theta);
        pose = constantAccelMath.calculate(t, relDelta, pose.clone());

        //if(useIMU){ pose.heading = (imu_offset + getIMUYaw()); }

        vel = pose.minus(lastPose).scale(1/t);
        acc = vel.minus(lastVel).scale(1/t);

        lastPose = pose;
        lastVel = vel;
        useIMU = false;
        //math about locolization
    }

    public Pose2d goToPosition(Pose2d target, double xy_speed, double xy_sensitivity, double r_speed, double r_sensitivity){
        state = State.MOVING;

        drivePID.setMaxPower(new Pose2d(xy_speed, xy_speed, r_speed));
        drivePID.setSensitivity(new Pose2d(xy_sensitivity, xy_sensitivity, r_sensitivity));

        drivePID.update(target, pose);

        return drivePID.out_power;
    }

    public boolean finalPath;
    public Pose2d followPoint;

    public int path_index = 0;
    public int last_index = 0;
    public boolean new_index = false;

    public boolean pathComplete;
    public double robotDist;
    public boolean inRange;
    public double decel_factor = 1;

    public double velLimit = 0;

    public ArrayList<Pose2d> intersections = new ArrayList<>();

    public void resetPath(){
        path_index = 0;
        last_index = 0;
        new_index = false;
        init_pos = pose;
        pathComplete = false;
    }

    public void resetLastPath(){
        path_index = 2;
        last_index = 2;
        new_index = false;
        init_pos = pose;
        pathComplete = false;
    }

    public Pose2d purePursuit(ArrayList<WayPoint> wayPoints, double default_radius){

        finalPath = (path_index == wayPoints.size() - 1);

        WayPoint currWayPoint = wayPoints.get(path_index);

        Pose2d lastTarget = (path_index == 0) ? init_pos : wayPoints.get(path_index - 1).toPose();
        Pose2d currTarget = currWayPoint.toPose();

        followPoint = lastFollowPoint;

        double lineDist = linePointDistance(lastTarget, currTarget, pose);
        double radius = Math.max(lineDist+0.5, default_radius);

        intersections = lineCircleIntersection(pose, radius, lastTarget, currTarget);
        boolean pathLost = intersections.isEmpty();

        double currDist = 1000;
        for (int i = 0; i < intersections.size(); i++){
            double newDist = Math.hypot(currTarget.getX() - intersections.get(i).getX(), currTarget.getY() - intersections.get(i).getY());
            if (newDist < currDist){
                followPoint = intersections.get(i);
                currDist = newDist;
            }
        }

        if(pathLost){ currDist = lastCurrDist; }

        robotDist = Math.hypot(currTarget.getX() - pose.getX(), currTarget.getY() - pose.getY());

        inRange = (currDist < currWayPoint.getCornerSensitivity() || robotDist < currWayPoint.getCornerSensitivity())
                && (!currWayPoint.getAngleWait() || Math.abs(pose.getHeading() - currTarget.getHeading()) < 0.15);

        if(inRange){
            if(finalPath){
                if(!justInRange){ waitTimer.reset(); }
                followPoint = currTarget;
            } else if (currWayPoint.hasWait()){
                if(!justInRange){ waitTimer.reset(); }
                followPoint = currTarget;
                if(waitTimer.seconds() > currWayPoint.getWaitTime()){ path_index ++; }
            } else {
                path_index ++;
            }
        }

        new_index = path_index != last_index;

        followPoint.heading = currWayPoint.getAngleMode() ? currWayPoint.getHeading() : pose.heading;

        velLimit = currWayPoint.getTransSpeed() * Math.min(Math.pow(robotDist / currWayPoint.getDecelerateDist(), decel_factor), 1);

        Pose2d output = goToPosition(followPoint, velLimit, currWayPoint.getRotSpeed(),
                currWayPoint.getSensitivity(), currWayPoint.getHeadingSensitivity());

        pathComplete = finalPath && drivePID.pos_reached && inRange && (waitTimer.seconds() > currWayPoint.getWaitTime());

        lastCurrDist = currDist;
        lastFollowPoint = followPoint;
        justInRange = inRange;
        last_index = path_index;

        return output;
    }

    public Pose2d purePursuitPose(ArrayList<Pose2d> poses, double default_radius, double sensitivity, double angle){

        boolean finalPath = (path_index == poses.size() - 2);

        Pose2d followPoint;

        if (!finalPath) {

            double lineDist = linePointDistance(poses.get(path_index), poses.get(path_index + 1), pose);
            double radius = Math.max(lineDist+0.5, default_radius);
            intersections = lineCircleIntersection(pose, radius, poses.get(path_index), poses.get(path_index + 1));

            boolean pathLost = (radius != default_radius) || intersections.isEmpty();

            followPoint = pathLost ? lastFollowPoint : new Pose2d(100, 100, 0);

            double currDist = 1000;
            for (int i = 0; i < intersections.size(); i++) {
                double newDist = Math.hypot(intersections.get(i).x - poses.get(path_index + 1).getX(), intersections.get(i).y - poses.get(path_index + 1).getY());

                if (newDist < currDist) {
                    followPoint = intersections.get(i);
                    currDist = newDist;
                }
            }

            robotDist = Math.hypot(pose.x - poses.get(path_index + 1).getX(), pose.y - poses.get(path_index + 1).getY());

            if (robotDist < sensitivity || currDist < sensitivity) {
                path_index++;
            }

            //radius = pathLost ? linePointDistance(poses.get(index).toPoint(), poses.get(index + 1).toPoint(), new Point(x_pos, y_pos)) + 0.5 : default_radius;
        } else {
            robotDist = Math.hypot(pose.x - poses.get(path_index + 1).getX(), pose.y - poses.get(path_index + 1).getY());

            followPoint = poses.get(poses.size() - 1);
        }

        double targetAngle = (angle == -1) ? poses.get(path_index + 1).getHeading() : angle; // if angle is -1, it's in Pose2d mode

        followPoint.heading = targetAngle;

        Pose2d output = goToPosition(followPoint, 0.6, 1, 0.4, 0.1);

        pathComplete = finalPath && robotDist < 2;

        lastFollowPoint = followPoint;

        return output;
    }
}
