package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Roller<T extends DcMotorSimple> {

    public T motor;
    public double POWER_INCREMENT = 0.1;
    public double MAX_POWER = 1;



    public Roller(T motor) {
        this.motor = motor;
    }

    public Roller(HardwareMap map, String motorName) {
        try {
            this.motor = (T) map.get(motorName);
        } catch (Exception e) {
            this.motor = null;
        }
    }

    /**
    Applies power
     */
    public void applyPower(double power) {
        if (motor == null) return;
        power = Math.min(power, MAX_POWER);
        power = Math.max(power, -1);
        motor.setPower(power);
    }

    /** Replacement for SWAP_DIRECTION
        Will reverse roll if swap is true else it goes forwards
        @param swap reverse roll if true
     */
    public void swapDirection(boolean swap) {
        if (motor == null) return;
        if (swap) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Swaps the direction (if its going reverse it swaps forward, etc.)
     */
    public void swapDirection() {
        if (motor == null) return;
        switch (motor.getDirection()) {
            case FORWARD:
                swapDirection(true);
                break;
            case REVERSE:
                swapDirection(false);
                break;
        }
    }



    /*
    Functionality Methods
    ---------------------
     */

    /**
    Adjusts the power to a boolean phrase. Useful for speeding up the roller
     @param increase Increases or decreases the power by POWER_INCREMENT
     */
    public void adjustPower(boolean increase) {
        if (motor == null) return;
        double power = Math.abs(motor.getPower());
        power += increase ? POWER_INCREMENT : -POWER_INCREMENT;
        applyPower(power);
    }

    /**
    Toggles power of roller
     @param toggle true applies power, false applies power of 0
     @param power power value to be applied if toggle is true
     */
    public void togglePower(boolean toggle, double power) {
        if (motor == null) return;
        if (!toggle) { power = 0.; }
        applyPower(power);
    }


}
