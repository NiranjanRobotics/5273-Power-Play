package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake extends SubsystemBase {

    private final double openPosition = 0.12;
    private final double closedPosition = 0.34;

    private Servo clawServo;
    private MotorEx slideMotor;

    public Outtake(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        slideMotor = new MotorEx(hardwareMap, "slideMotor", Motor.GoBILDA.RPM_435);

        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void openClaw() {
        clawServo.setPosition(openPosition);
    }

    public void closeClaw() {
        clawServo.setPosition(closedPosition);
    }



}
