package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {

    private enum Level {
        GROUND,
        LOW,
        MEDIUM,
        HIGH
    }

    private final double OPEN_POSITION = 0.12;
    private final double CLOSED_POSITION = 0.34;

    private static final int EXTENDED_POSITION = 640;
    private static final int RETRACTED_POSITION = 34;
    private static final int MAXIMUM_POSITION = 680;
    //junction heights TODO: find values for junction heights
    private static final int GROUND_HEIGHT = 50;
    private static final int LOW_HEIGHT = 200;
    private static final int MEDIUM_HEIGHT = 400;
    private static final int HIGH_HEIGHT = 600;
    private static Level currentLevel = Level.GROUND;
    private static int targetHeight = 34;


    private static final double kP = 0.05;
    private static final double TOLERANCE = 31;

    private static final double SLIDE_SPEED = 0.3;
    private static final double HOLDING_SPEED = 0.1;
    private static final double STOPPED_SPEED = 0.03;

    private final Servo clawServo;
    private final MotorEx leftSlideMotor;
    private final MotorEx rightSlideMotor;

    public Outtake(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        leftSlideMotor = new MotorEx(hardwareMap, "leftSlideMotor", Motor.GoBILDA.RPM_312);
        rightSlideMotor = new MotorEx(hardwareMap, "rightSlideMotor", Motor.GoBILDA.RPM_312);

        clawServo.setDirection(Servo.Direction.FORWARD);

        initializeSlideMotor(leftSlideMotor);
        initializeSlideMotor(rightSlideMotor);
    }

    private void initializeSlideMotor(MotorEx motor) {
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setPositionTolerance(TOLERANCE);
        motor.setPositionCoefficient(kP);
    }

    public void openClaw() {
        clawServo.setPosition(OPEN_POSITION);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSED_POSITION);
    }

    public void incrementLevel() {
        switch(currentLevel) {
            case GROUND:
                currentLevel = Level.LOW;
                break;
            case LOW:
                currentLevel = Level.MEDIUM;
                break;
            case MEDIUM:
            case HIGH:
                currentLevel = Level.HIGH;
                break;
        }
    }

    public void decrementLevel() {
        switch(currentLevel) {
            case GROUND:
            case LOW:
                currentLevel = Level.GROUND;
                break;
            case MEDIUM:
                currentLevel = Level.LOW;
                break;
            case HIGH:
                currentLevel = Level.MEDIUM;
                break;
        }
    }

    private void setTargetHeight() {
        switch(currentLevel) {
            case GROUND:
                targetHeight = GROUND_HEIGHT;
                break;
            case LOW:
                targetHeight = LOW_HEIGHT;
                break;
            case MEDIUM:
                targetHeight = MEDIUM_HEIGHT;
                break;
            case HIGH:
                targetHeight = HIGH_HEIGHT;
                break;
        }
    }

    public void extend() {
        setTargetHeight();
        leftSlideMotor.setTargetPosition(targetHeight);
        rightSlideMotor.setTargetPosition(targetHeight);
    }

    public void retract() {
        leftSlideMotor.setTargetPosition(RETRACTED_POSITION);
        rightSlideMotor.setTargetPosition(RETRACTED_POSITION);
    }

    public void periodic() {
        if (leftSlideMotor.getCurrentPosition() > MAXIMUM_POSITION) {
            leftSlideMotor.set(STOPPED_SPEED);
        }
        if (rightSlideMotor.getCurrentPosition() > MAXIMUM_POSITION) {
            rightSlideMotor.set(STOPPED_SPEED);
        }
    }

}
