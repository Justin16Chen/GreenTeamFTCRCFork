package org.firstinspires.ftc.teamcode.utils.generalOpModes;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.IntConsumer;
import java.util.function.IntSupplier;

// wrapper class that monitors gamepad input
// can check when the user has started pressing a button and how long they have been pressing it for
public class GamepadTracker {
    private final Gamepad gamepad;

    private int aFrameCount = 0;
    private int bFrameCount = 0;
    private int xFrameCount = 0;
    private int yFrameCount = 0;
    private int dpadUpFrameCount = 0;
    private int dpadDownFrameCount = 0;
    private int dpadLeftFrameCount = 0;
    private int dpadRightFrameCount = 0;
    private int leftBumperFrameCount = 0;
    private int rightBumperFrameCount = 0;
    private int leftTriggerFrameCount = 0;
    private int rightTriggerFrameCount = 0;
    private int startButtonFrameCount = 0;
    private int backButtonFrameCount = 0;

    public GamepadTracker(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad == null)
            return;

        // Update each button's frame count
        updateButtonFrame(gamepad.a, () -> aFrameCount, (c) -> aFrameCount = c);
        updateButtonFrame(gamepad.b, () -> bFrameCount, (c) -> bFrameCount = c);
        updateButtonFrame(gamepad.x, () -> xFrameCount, (c) -> xFrameCount = c);
        updateButtonFrame(gamepad.y, () -> yFrameCount, (c) -> yFrameCount = c);

        updateButtonFrame(gamepad.dpad_up, () -> dpadUpFrameCount, (c) -> dpadUpFrameCount = c);
        updateButtonFrame(gamepad.dpad_down, () -> dpadDownFrameCount, (c) -> dpadDownFrameCount = c);
        updateButtonFrame(gamepad.dpad_left, () -> dpadLeftFrameCount, (c) -> dpadLeftFrameCount = c);
        updateButtonFrame(gamepad.dpad_right, () -> dpadRightFrameCount, (c) -> dpadRightFrameCount = c);

        updateButtonFrame(gamepad.left_bumper, () -> leftBumperFrameCount, (c) -> leftBumperFrameCount = c);
        updateButtonFrame(gamepad.right_bumper, () -> rightBumperFrameCount, (c) -> rightBumperFrameCount = c);

        updateButtonFrame(gamepad.start, () -> startButtonFrameCount, (c) -> startButtonFrameCount = c);
        updateButtonFrame(gamepad.back, () -> backButtonFrameCount, (c) -> backButtonFrameCount = c);

        // For triggers, consider them "pressed" if they exceed a threshold
        updateButtonFrame(gamepad.left_trigger > 0.2, () -> leftTriggerFrameCount, (c) -> leftTriggerFrameCount = c);
        updateButtonFrame(gamepad.right_trigger > 0.2, () -> rightTriggerFrameCount, (c) -> rightTriggerFrameCount = c);
    }

    private void updateButtonFrame(boolean isPressed, IntSupplier frameCountSupplier, IntConsumer frameCountSetter) {
        if (isPressed) {
            frameCountSetter.accept(frameCountSupplier.getAsInt() + 1);
        } else {
            frameCountSetter.accept(0);
        }
    }

    // First-frame checker methods
    public boolean isAClicked() { return aFrameCount == 1; }
    public boolean isBClicked() { return bFrameCount == 1; }
    public boolean isXClicked() { return xFrameCount == 1; }
    public boolean isYClicked() { return yFrameCount == 1; }

    public boolean isDpadUpClicked() { return dpadUpFrameCount == 1; }
    public boolean isDpadDownClicked() { return dpadDownFrameCount == 1; }
    public boolean isDpadLeftClicked() { return dpadLeftFrameCount == 1; }
    public boolean isDpadRightClicked() { return dpadRightFrameCount == 1; }

    public boolean isLBClicked() { return leftBumperFrameCount == 1; }
    public boolean isRBClicked() { return rightBumperFrameCount == 1; }
    public boolean isLTClicked() { return leftTriggerFrameCount == 1; }
    public boolean isRTClicked() { return rightTriggerFrameCount == 1; }
    public boolean isStartButtonClicked() {
        return startButtonFrameCount == 1;
    }
    public boolean isBackButtonClicked() {
        return backButtonFrameCount == 1;
    }

    // Frame count getters
    public int getAFrameCount() { return aFrameCount; }
    public int getBFrameCount() { return bFrameCount; }
    public int getXFrameCount() { return xFrameCount; }
    public int getYFrameCount() { return yFrameCount; }

    public int getDpadUpFrameCount() { return dpadUpFrameCount; }
    public int getDpadDownFrameCount() { return dpadDownFrameCount; }
    public int getDpadLeftFrameCount() { return dpadLeftFrameCount; }
    public int getDpadRightFrameCount() { return dpadRightFrameCount; }

    public int getLBFrameCount() { return leftBumperFrameCount; }
    public int getRBFrameCount() { return rightBumperFrameCount; }
    public int getLTFrameCount() { return leftTriggerFrameCount; }
    public int getRTFrameCount() { return rightTriggerFrameCount; }
    public int getStartButtonFrameCount() {
        return startButtonFrameCount;
    }
    public int getBackButtonFrameCount() {
        return backButtonFrameCount;
    }

    // Pressed state checkers
    public boolean isAPressed() { return aFrameCount > 0; }
    public boolean isBPressed() { return bFrameCount > 0; }
    public boolean isXPressed() { return xFrameCount > 0; }
    public boolean isYPressed() { return yFrameCount > 0; }

    public boolean isDpadUpPressed() { return dpadUpFrameCount > 0; }
    public boolean isDpadDownPressed() { return dpadDownFrameCount > 0; }
    public boolean isDpadLeftPressed() { return dpadLeftFrameCount > 0; }
    public boolean isDpadRightPressed() { return dpadRightFrameCount > 0; }

    public boolean isLBPressed() { return leftBumperFrameCount > 0; }
    public boolean isRBPressed() { return rightBumperFrameCount > 0; }
    public boolean isLTPressed() { return leftTriggerFrameCount > 0; }
    public boolean isRTPressed() { return rightTriggerFrameCount > 0; }
    public boolean isStartButtonPressed() {
        return startButtonFrameCount > 0;
    }
    public boolean isBackButtonPressed() {
        return backButtonFrameCount > 0;
    }

    // joystick getters
    public double getLeftStickX() {
        return gamepad.left_stick_x;
    }
    public double getLeftStickY() {
        return gamepad.left_stick_y;
    }
    public double getRightStickX() {
        return gamepad.right_stick_x;
    }
    public double getRightStickY() {
        return gamepad.right_stick_y;
    }
}