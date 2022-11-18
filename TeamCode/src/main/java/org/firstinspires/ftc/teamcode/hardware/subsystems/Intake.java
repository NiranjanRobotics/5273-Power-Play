package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private MotorEx rightMotor;
    private MotorEx leftMotor;

    private final double runPower = 0.85;
    private double stopPower;

    public Intake(HardwareMap hardwareMap) {
        rightMotor = new MotorEx(hardwareMap, "rightIntake", Motor.GoBILDA.RPM_312);
        leftMotor = new MotorEx(hardwareMap, "leftIntake", Motor.GoBILDA.RPM_312);

        rightMotor.setRunMode(Motor.RunMode.RawPower);
        leftMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void run() {
        rightMotor.set(runPower);
        leftMotor.set(runPower);
    }

    public void stop() {
        rightMotor.set(stopPower);
        leftMotor.set(stopPower);
    }

}
