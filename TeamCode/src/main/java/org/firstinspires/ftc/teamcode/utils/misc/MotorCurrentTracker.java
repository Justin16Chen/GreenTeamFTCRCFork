package org.firstinspires.ftc.teamcode.utils.misc;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorCurrentTracker {
    public static final CurrentUnit CURRENT_UNIT = CurrentUnit.MILLIAMPS;
    private final int maxNormalCurrent; // Note: this int MUST be in MilliAmps unit
    private final int abnormalValidationFrames;
    private int consecutiveAbnormalFrames;
    private final int abnormalSafetyFrames; // the class assumes that after a current spike has been validated, it will spike for this many frames (doesn't check actual current during this period)
    private int safetyFramesTimer;
    private final DcMotorEx motor;
    public MotorCurrentTracker(DcMotorEx motor, int maxNormalCurrent, int abnormalValidationFrames, int abnormalSafetyFrames) {
        this.motor = motor;
        this.maxNormalCurrent = maxNormalCurrent;
        this.abnormalValidationFrames = abnormalValidationFrames;
        this.abnormalSafetyFrames = abnormalSafetyFrames;
        consecutiveAbnormalFrames = 0;
        safetyFramesTimer = 0;
    }
    public int getConsecutiveAbnormalFrames() {
        return consecutiveAbnormalFrames;
    }
    public boolean hasValidatedAbnormalCurrent() {
        return consecutiveAbnormalFrames >= abnormalValidationFrames;
    }
    public boolean hasRawAbnormalCurrent() {
        return motor.getCurrent(CURRENT_UNIT) > maxNormalCurrent;
    }
    public void updateCurrentTracking() {
        // update safety
        if (hasValidatedAbnormalCurrent()) {
            safetyFramesTimer++;

            if (safetyFramesTimer > abnormalSafetyFrames)
                safetyFramesTimer = 0;
        }
        // check for validation (not when safety is running)
        if (safetyFramesTimer == 0) {
            if (motor.getCurrent(CURRENT_UNIT) > maxNormalCurrent)
                consecutiveAbnormalFrames++;
            else
                consecutiveAbnormalFrames = 0;
        }
    }

}

