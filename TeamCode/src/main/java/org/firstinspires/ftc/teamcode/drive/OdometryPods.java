package org.firstinspires.ftc.teamcode.drive;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Pose2d;

import java.util.Arrays;
import java.util.List;

//@Config
public class OdometryPods{
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //change the thing base on the real robot
    public static double LATERAL_DISTANCE = 9.6299; // 10.72158. in; distance between the left_hang and right_hang encoders
    //dont know if is correct change later
    public static double CENTER_LEFT_OFFSET = LATERAL_DISTANCE/2;
    public static double CENTER_RIGHT_OFFSET = LATERAL_DISTANCE/2;
    // if they are not right in the middle then not divided by 2 any more

    public static double FORWARD_OFFSET = 0.04; // in; offset of the lateral encoder 1.74

    //change change change

    public static Encoder leftEncoder, rightEncoder, frontEncoder;

    public OdometryPods(HardwareMap hardwareMap) {
        Arrays.asList(
                new Pose2d(0, -CENTER_LEFT_OFFSET, 0), //x-pod left_hang // used to be posigtive// right_hang //used to be negative
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // y-pod
        );

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bl"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));//leftrear
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "br"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
