package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {

    private final MotorEx rightMotor;
    private final MotorEx leftMotor;

    private final double runPower = 0.85;
    private double stopPower;

    public Intake(HardwareMap hardwareMap) {
        leftMotor = new MotorEx(hardwareMap, "leftIntakeWheel", Motor.GoBILDA.RPM_312);
        rightMotor = new MotorEx(hardwareMap, "rightIntakeWheel", Motor.GoBILDA.RPM_312);

        leftMotor.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void run() {
        runAtPower(runPower);
    }

    public void runReverse() {
        runAtPower(-runPower);
    }

    private void runAtPower(double power) {
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void stop() {
        runAtPower(stopPower);
    }
}
