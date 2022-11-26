package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {

    private final double OPEN_POSITION = 0.12;
    private final double CLOSED_POSITION = 0.34;

    private static final int EXTENDED_POSITION = 640;
    private static final int RETRACTED_POSITION = 34;
    private static final int MAXIMUM_POSITION = 680;

    private static final double kP = 0.05;
    private static final double TOLERANCE = 31;

    private static final double SLIDE_SPEED = 0.3;
    private static final double HOLDING_SPEED = 0.1;
    private static final double STOPPED_SPEED = 0.03;

    private Servo clawServo;
    private MotorEx leftSlideMotor;
    private MotorEx rightSlideMotor;

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

    public void extend() {
        leftSlideMotor.setTargetPosition(EXTENDED_POSITION);
        rightSlideMotor.setTargetPosition(EXTENDED_POSITION);
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
