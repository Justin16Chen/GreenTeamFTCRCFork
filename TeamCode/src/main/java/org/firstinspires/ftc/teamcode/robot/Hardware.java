package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// standardizes retrieving from hardware map
// makes reusing robot components in main tele and other test teles really straightforwards
// ensures consistency between comp and test teles
@Config
public class Hardware {
    // store these as static so they can be adjusted on dashboard for ease of use
    public static String cameraName = "webcam", frontColorSensorName = "frontColorSensor", middleColorSensorName = "middleColorSensor", backColorSensorName = "backColorSensor";
    public static String pinpointName = "odo";
    public static double pinpointXOffset = 5.562, pinpointYOffset = -0.05;
    public static String frontLeftName = "FL", frontRightName = "FR", backLeftName = "BL", backRightName = "BR";
    public static boolean reverseFrontLeft = true, reverseFrontRight = false, reverseBackLeft = true, reverseBackRight = false;
    public static String intakeName = "intake", shooterLeftName = "leftShooter", shooterRightName = "rightShooter";
    public static String leftHoodServoName = "leftHoodServo", rightHoodServoName = "rightHoodServo";
    public static int leftHoodServoDownPWM = 550, leftHoodServoUpPWM = 2400, rightHoodServoDownPWM = 2400, rightHoodServoUpPWM = 550;
    public static String flipperServoName = "flipperServo";
    public static int flipperOpenPWM = 975, flipperClosePWM = 1050;
    public static String leftParkServoName = "leftParkServo", rightParkServoName = "rightParkServo";
    public static int leftStorePWM = 1816, rightStorePWM = 1216, leftParkPWM = 1348, rightParkWPM = 1676;


    public final HardwareMap hardwareMap;
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public CameraName getCamera() {
        return hardwareMap.get(WebcamName.class, cameraName);
    }
    public DcMotorEx getFLDriveMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, frontLeftName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Hardware.reverseFrontLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return motor;
    }
    public DcMotorEx getFRDriveMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, frontRightName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Hardware.reverseFrontRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return motor;
    }
    public DcMotorEx getBLDriveMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, backLeftName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Hardware.reverseBackLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return motor;
    }
    public DcMotorEx getBRDriveMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, backRightName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(Hardware.reverseBackRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        return motor;
    }
    public DcMotorEx getIntakeMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, intakeName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
    public DcMotorEx getLeftShooterMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, shooterLeftName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }
    public DcMotorEx getRightShooterMotor() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, shooterRightName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        return motor;
    }
//    public CRServo getLeftHoodServo() {
//        return hardwareMap.get(CRServo.class, leftHoodServoName);
//    }
//    public CRServo getRightHoodServo() {
//        return hardwareMap.get(CRServo.class, rightHoodServoName);
//    }
    public ServoImplEx getLeftHoodServo() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, leftHoodServoName);
        servo.setPwmRange(new PwmControl.PwmRange(leftHoodServoDownPWM, leftHoodServoUpPWM));
        return servo;
    }
    public ServoImplEx getRightHoodServo() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, rightHoodServoName);
        servo.setPwmRange(new PwmControl.PwmRange(rightHoodServoDownPWM, rightHoodServoUpPWM));
        return servo;
    }

    public ServoImplEx getLeftParkServo() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, leftParkServoName);
        servo.setPwmRange(new PwmControl.PwmRange(leftStorePWM, leftParkPWM));
        return servo;
    }
    public ServoImplEx getRightParkServo() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, rightParkServoName);
        servo.setPwmRange(new PwmControl.PwmRange(rightStorePWM, rightParkWPM));
        return servo;
    }
    public ServoImplEx getFlipperServo() {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, flipperServoName);
        servo.setPwmRange(new PwmControl.PwmRange(flipperClosePWM, flipperOpenPWM));
        return servo;
    }
}