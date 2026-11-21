package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.camera.VisionPortalCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Locale;


// Instructions to connect: adb connect 192.168.43.1:5555
// cd %localappdata%/android/sdk/platform-tools
@TeleOp(name = "PlayerDrive")
public class PlayerDrive extends RobotBase {

    private static final Logger log = LoggerFactory.getLogger(PlayerDrive.class);

    int targetTag = BLUE_TAG;
    private ElapsedTime yellowPresenceTimer = null;
    private boolean hasRumbledForGreen = false;

    // Pattern for REV Blinkin LED Driver when target yellow is acquired (Forest Palette flashes/waves yellow & green)
    public static RevBlinkinLedDriver.BlinkinPattern GREEN_INDICATOR_PATTERN = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;


    @Override

    public void init() {
        super.init();
        // initCamera();
    }



    @Override
    public void init_loop() {
        super.init_loop();

        if (gamepad2.dpadLeftWasPressed()) {
            targetTag = BLUE_TAG;
        }
        if (gamepad2.dpadRightWasPressed()) {
            targetTag = RED_TAG;
        }

        commonPrintData();
    }

    @Override
    public void loop() {
        gamepad1Loop();
        gamepad2Loop();
        commonPrintData();
    }

    boolean settingVelocity = false;
    private void gamepad1Loop() {
        //Driving
        driveSystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //Slow down when holding left bumper
        if (gamepad1.left_bumper) { driveSystem.powerFactor = 0.2; }
        else { driveSystem.powerFactor = 1.; }

        // FlowerTaker tap toggle on Gamepad 1 Right Bumper
        if (gamepad1.rightBumperWasPressed()) {
            flowerTaker.switchPosition();
        }
    }

    /*
        Intake, Launcher, Transfer
       --------------------------
    */
    double curVelTarget = FAR_VELOCITY; // Default to min
    boolean stableVelocity = false;
    AprilTagDetection tag = null;
    boolean didFirst = false;
    boolean appliedVelocityDirectly = false;
    private void gamepad2Loop() {
        if (gamepad2.xWasPressed()) { launcher.MAX_POWER -= .05; }
        if (gamepad2.bWasPressed()) { launcher.MAX_POWER += .05; }
        
        // Updated dpad up/down for fixed velocity values
        if (gamepad2.dpadUpWasPressed()) { curVelTarget = FAR_VELOCITY; }
        if (gamepad2.dpadDownWasPressed()) { curVelTarget = CLOSE_VELOCITY; }
        
        if (gamepad2.dpadLeftWasPressed()) {
            curVelTarget = curVelTarget != CLOSE_VELOCITY ? CLOSE_VELOCITY : FAR_VELOCITY;
        }

        tag = VisionPortalCamera.getTargetTag(aTagProc, targetTag);
        turret.TARGET_TAG = tag;
        if (gamepad2.leftBumperWasPressed()) {
            turret.switchTurretState(Turret.TurretState.FOLLOW_TAG);
        }
        turret.loop(); //NECESSARY FOR TURRET TO WORK

        //Either use velocity or allow drivers to use left stick to use power
        if (gamepad2.rightBumperWasReleased()) {
            if (launcher.targetVelocity == curVelTarget && !didFirst) {
                launcher.applyVelocity(curVelTarget);
                appliedVelocityDirectly = true;
            } else {
                launcher.switchVelocity(curVelTarget);
            }
            didFirst = true;
        } else if (launcher.targetVelocity == 0) {
            launcher.togglePower(gamepad2.left_trigger > 0.1, launcher.MAX_POWER);
        }

        if (launcher.motor != null) {
            stableVelocity = launcher.reachedVelocity() && launcher.motor.getVelocity() > 0;
        } else {
            stableVelocity = false;
        }

        if (gamepad2.dpadRightWasPressed()) {
            intake.swapDirection();
            transfer.swapDirection();
            lTransport.swapDirection();
            rTransport.swapDirection();
        }
        boolean ACTIVE_INTAKE = gamepad2.right_trigger > 0.1;
        intake.togglePower(ACTIVE_INTAKE, intake.MAX_POWER);
        transfer.togglePower(ACTIVE_INTAKE, transfer.MAX_POWER);
        lTransport.togglePower(ACTIVE_INTAKE, lTransport.MAX_POWER);
        rTransport.togglePower(ACTIVE_INTAKE, rTransport.MAX_POWER);

        stopper.togglePosition(!gamepad2.y);
        telemetry.addData("FlowerTaker Tapped (Gamepad 1 A)", gamepad1.aWasPressed() ? "Yes" : "No");
    }

    //Telemetry + Debug Data
    private void commonPrintData() {
        telemetry.addLine("|||--IMPORTANT INFORMATION--|||");
        if (launcher.motor != null) {
            telemetry.addData("Launcher Velocity", launcher.motor.getVelocity());
            telemetry.addData("Launcher Power", launcher.motor.getPower());
        } else {
            telemetry.addData("Launcher Velocity", "N/A (Motor Missing)");
            telemetry.addData("Launcher Power", "N/A (Motor Missing)");
        }
        telemetry.addData("Launcher Target Velocity (Supposed)", curVelTarget);
        telemetry.addData("Launcher Target Velocity (ACTUAL)", launcher.targetVelocity);
        telemetry.addData("Launcher Velocity Stable", stableVelocity);
        telemetry.addData("April Tag Visible", tag != null);
        telemetry.addData("Turret State", turret.curTurretState);
        telemetry.addData("Turret Target Global Yaw", turret.TARGET_GLOBAL_YAW);
        telemetry.addLine("|||-------------------------|||");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("|||----GAMEPAD1 CONTROLS----|||");
        telemetry.addData("Moving (Forward & Strafe)", "Left Stick");
        telemetry.addData("Rotating", "Right Stick");
        telemetry.addData("Slow Down", "Left/Right Bumper");
        telemetry.addLine("|||-------------------------|||");
        telemetry.addLine("|||----GAMEPAD2 CONTROLS----|||");
        telemetry.addData("Launcher", "Right Bumper (Toggle)");
        telemetry.addData("Turret State Toggle", "Left Bumper (Toggle)");
        telemetry.addData("Intake", "Right Trigger (Hold)");
        telemetry.addData("Open Stopper", "Y (Hold)");
        telemetry.addData("Swap Intake Direction", "Dpad Right (Toggle)");
        telemetry.addData("Switch Launcher Velocity", "Dpad Up (Max) / Down (Min)");
        telemetry.addLine("|||-------------------------|||");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("didFirst", didFirst);
        telemetry.addData("appliedVelocityDirectly", appliedVelocityDirectly);

        // Check if color sensor is connected
        if (colorSensor != null) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Ratio-based Yellow detection (Works for Close: ~0.0105 & Far: ~0.0034)
            double totalLight = colors.red + colors.green + colors.blue;
            boolean isTargetYellow = (colors.green > colors.red)
                                  && (colors.red > 1.2 * colors.blue)
                                  && (colors.green > 1.2 * colors.blue)
                                  && (totalLight > 0.004);

            boolean showGreen = false;

            // Must be continuously detected for at least 1.5 seconds
            if (isTargetYellow) {
                if (yellowPresenceTimer == null) {
                    yellowPresenceTimer = new ElapsedTime();
                } else if (yellowPresenceTimer.seconds() >= 0.5) {
                    showGreen = true;
                    if (!hasRumbledForGreen) {
                        if (gamepad1 != null) gamepad1.rumbleBlips(2);
                        if (gamepad2 != null) gamepad2.rumbleBlips(2);
                        hasRumbledForGreen = true;
                    }
                }
            } else {
                yellowPresenceTimer = null; // Reset continuous timer if yellow is lost
                hasRumbledForGreen = false;
            }

            if (indicator != null) {
                if (showGreen) {
                    indicator.setPattern(GREEN_INDICATOR_PATTERN);
                } else {
                    indicator.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                }
            }

            telemetry.addData("Target Yellow In Range", isTargetYellow ? "YES" : "NO");
            double timeSeen = (yellowPresenceTimer != null) ? yellowPresenceTimer.seconds() : 0.0;
            telemetry.addData("Time Held (Target >= 1.5s)", String.format(Locale.US, "%.2f s", timeSeen));
            telemetry.addData("Indicator Status", showGreen ? ("GREEN (" + GREEN_INDICATOR_PATTERN + ")") : "OFF");
            telemetry.addData("Red", colors.red);
            telemetry.addData("Green", colors.green);
            telemetry.addData("Blue", colors.blue);
        } else {
            telemetry.addData("Color Sensor", "Missing / Not Connected");
        }
    }
}
