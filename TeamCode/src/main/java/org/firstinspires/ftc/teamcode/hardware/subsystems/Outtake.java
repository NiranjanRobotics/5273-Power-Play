package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake extends SubsystemBase {
    public final LinearSlides linearSlides;
    public final Arm arm;
    public final Claw claw;

    public Outtake(HardwareMap hardwareMap) {
        linearSlides = new LinearSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
    }
    //todo use them all

    @Override
    public void periodic() {
        linearSlides.periodic();
    }
}
