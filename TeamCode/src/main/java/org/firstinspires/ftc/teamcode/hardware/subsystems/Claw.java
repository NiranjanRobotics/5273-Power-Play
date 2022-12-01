package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;


public class Claw {

    private final double OPEN_POSITION = 0.12;
    private final double CLOSED_POSITION = 0.34;

    private final Servo clawServo;

    public Claw(HardwareMap hardwareMap){
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.FORWARD);
    }
    public void openClaw() {

        clawServo.setPosition(OPEN_POSITION);
    }

    public void closeClaw() {
        clawServo.setPosition(CLOSED_POSITION);
    }


}
