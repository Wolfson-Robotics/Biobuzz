package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8) // unit = kg
            .forwardZeroPowerAcceleration(-40.443593609141494)
            .lateralZeroPowerAcceleration(-61.34546046201595)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.037, 0, 0.005, 0.025));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf_drive")
            .rightRearMotorName("rb_drive")
            .leftRearMotorName("lb_drive")
            .leftFrontMotorName("lf_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(96.63297782357284)
            .yVelocity(76.49313378521776);

    // Two-Wheel Odometry Constants (2 Deadwheels + Control Hub IMU)
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardPodY(1)  // TEMP! Y distance (inches) of forward pod from center of rotation
            .strafePodX(1)  // TEMP! X distance (inches) of strafe pod from center of rotation
            .forwardEncoder_HardwareMapName("leftFront") // Motor port for forward pod
            .strafeEncoder_HardwareMapName("rightRear")  // Motor port for strafe pod
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardTicksToInches(0.001989436789)        // For goBILDA 48mm pods: ~0.001989
            .strafeTicksToInches(0.001989436789)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants) // Use twoWheelLocalizer here
                .build();
    }
}