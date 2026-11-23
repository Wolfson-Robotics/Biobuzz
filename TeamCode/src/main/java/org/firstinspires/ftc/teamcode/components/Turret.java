package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.Rotation;

/**
 * Field-Centric Auto-Aiming Turret Component using RobotBase Position & Tipper Coordinates
 */
public class Turret extends RollerEx {

    public IMU imu = null;

    public double TICKS_PER_REV = 537.7;
    public double GEAR_RATIO = 200.0 / 92.0;
    public double FULL_TICKS_PER_REV = TICKS_PER_REV * GEAR_RATIO;

    public double MAX_TURRET_YAW = 70;  // Max turret angle relative to robot (degrees)
    public double MIN_TURRET_YAW = -70; // Min turret angle relative to robot (degrees)

    public double TARGET_TURRET_YAW = 0; // Target turret angle relative to robot (degrees)
    public double turretYaw = 0;         // Current turret angle relative to robot (degrees)
    public double encoderPos = 0;
    public double targetPos = 0;

    // References to arrays passed from RobotBase
    public double[] Position;      // [x, y, heading]
    public double[][] tipperBlue;  // {{36, 30}, {36, 96}}
    public double[][] tipperRed;   // {{84, 30}, {84, 96}}

    // Target selection flags
    public boolean isRedAlliance = false;
    public int tipperSideIndex = 0; // 0 = side facing origin, 1 = side away from origin

    // Compatibility fields for legacy OpModes
    public double TARGET_GLOBAL_YAW = 0;
    public double globalYaw = 0;
    public double robotYaw = 0;

    public enum TurretState {
        GO_TO_ZERO,
        GO_TO_GLOBAL_TARGET
    }
    public TurretState curTurretState = TurretState.GO_TO_ZERO;

    public Turret(DcMotorEx motor) {
        super(motor);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Turret(HardwareMap map, String motorName) {
        this((DcMotorEx) map.get(motorName));
    }

    public void switchTurretState(TurretState state) {
        this.curTurretState = state;
    }

    /**
     * Main loop method for the turret hardware.
     */
    public void loop() {
        // Read current robot pose from Pedro Pathing into RobotBase's Position array
        if (follower != null && follower.getPose() != null && Position != null) {
            Pose currentPose = follower.getPose();
            Position[0] = currentPose.getX();
            Position[1] = currentPose.getY();
            Position[2] = currentPose.getHeading(); // in radians
        }

        aimAtTargetTipper();
        updateMotor();
    }

    /**
     * Overloaded loop method to accept position array.
     */
    public void loop(double[] position) {
        if (position != null) {
            this.Position = position;
        }
        loop();
    }

    /**
     * Calculates the turret angle needed to point at the target tipper coordinate.
     */
    public void aimAtTargetTipper() {
        if (Position == null) return;

        double[][] currentTipper = isRedAlliance ? tipperRed : tipperBlue;
        if (currentTipper == null || currentTipper.length <= tipperSideIndex) return;

        double targetX = currentTipper[tipperSideIndex][0];
        double targetY = currentTipper[tipperSideIndex][1];

        double robotX = Position[0];
        double robotY = Position[1];
        double robotHeadingRad = Position[2];

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        double targetFieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        double relativeAngleDeg = Rotation.wrap180(targetFieldAngleDeg - robotHeadingDeg);
        TARGET_TURRET_YAW = Rotation.limitAndRound(relativeAngleDeg, MIN_TURRET_YAW, MAX_TURRET_YAW);
    }

    private void updateMotor() {
        encoderPos = motor.getCurrentPosition();
        turretYaw = (360.0 * encoderPos) / FULL_TICKS_PER_REV;

        double targetAbsPos = (TARGET_TURRET_YAW / 360.0) * FULL_TICKS_PER_REV;

        // Calculate shortest path delta to prevent wire tangling / over-rotation
        double deltaPos = targetAbsPos - encoderPos;
        deltaPos = ((deltaPos + (FULL_TICKS_PER_REV / 2)) % FULL_TICKS_PER_REV) - FULL_TICKS_PER_REV / 2;
        targetPos = encoderPos + deltaPos;

        motor.setTargetPosition((int) targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(MAX_POWER);
    }
}
