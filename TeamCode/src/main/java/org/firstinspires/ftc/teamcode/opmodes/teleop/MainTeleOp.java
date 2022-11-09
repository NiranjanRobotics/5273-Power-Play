package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends BaseOpMode {

    private boolean centricity = false;
    private final double TRIGGER_CONSTANT = 0.15;
    private final double SLOW_MODE_PERCENT = 0.4;
    private double fieldCentricOffset0 = 0.0;
    private double fieldCentricOffset1 = 0.0;

    // opmode vars here =========================================================================
    public void subInit() {
        driveSpeed = 1.0;
    }

    public void subLoop() {
 //       Update
//        cycle = 1.0/(time-prevRead);
//        prevRead = time;
//        timingScheduler.run();

        //Movement =================================================================================================
        drive();


        //Subsystem control =========================================================================================

        if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)){ }

        else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT){ }

        else { }

        if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { }

        else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) { }

        else { }


        // Driver 2

        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){ }

        else if (gamepadEx2.wasJustPressed(Button.RIGHT_BUMPER)){ }

        if(gamepadEx2.wasJustPressed(Button.LEFT_STICK_BUTTON)) { }

        else if(gamepadEx2.wasJustPressed(Button.DPAD_RIGHT)) { }

        else if(gamepadEx2.wasJustPressed(Button.DPAD_LEFT)) { }

        else if(gamepadEx2.wasJustPressed(Button.DPAD_UP)) { }

        else if(gamepadEx2.wasJustPressed(Button.DPAD_DOWN)) { }

        if (gamepadEx2.wasJustPressed(Button.Y)){ }

    /*
    Controller 1
    A:      B:      X:      Y:
    DPAD
    L: D: U: R:
    Joystick
    L: Field centric movement
    R:Set orientation / rotation (determine through practice)
    Trigger L/R:
    Bumper:
    L:     R:
    Other
    Start:  Back: switch between automation and driving

    Controller 2
    A:      B:      X:      Y:
    DPAD
    L:      D:      U:      R:
    Joystick
    L:movement/reset field centric or progress automation
    R:movement/switch robot/field centric or none
    Trigger L/R: slow driving
    Bumper
    L:    R:
    Other
    Start:  Back: switch between automation and driving
     */

        CommandScheduler.getInstance().run();

        telemetry.addData("X", bot.roadRunner.getPoseEstimate().getX());
        telemetry.addData("Y", bot.roadRunner.getPoseEstimate().getY());
        telemetry.addData("Heading", bot.roadRunner.getPoseEstimate().getHeading());
        telemetry.addData("Driver Left Stick", "left X" + gamepadEx1.getLeftX() + ": " + gamepadEx1.getLeftY());

    }

    private void drive() {
        final double gyroTolerance = 0.05;

        double tempAngle0 = bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset0;
        double tempAngle1 = bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset1;
        // set absolute value of angle always less than or equal to 180
        final double gyroAngle0 = (tempAngle0 > 180) ? tempAngle0 - 360 : tempAngle0;
        // if imu is null, then use other imu
        final double gyroAngle1 = (bot.imu1 != null) ?
                ((tempAngle1 > 180) ? tempAngle1 - 360 : tempAngle1)
                : gyroAngle0;
        final double avgGyroAngle = ((gyroAngle0 + gyroAngle1)/2);

        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftX(), gamepadEx1.getLeftY()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() * Math.abs(gamepadEx1.getRightX()),
                        0);
        if (bot.roadRunner.mode == RRMecanumDrive.Mode.IDLE) {

            boolean dpadPressed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    || gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            boolean buttonPressed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B));

            double forwardSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : -1) : 0;
            double strafeSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : -1) : 0;
            double turnSpeed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B)) ? (gamepadEx1.getButton(GamepadKeys.Button.B) ? 1 : -1) : 0;

            if (centricity) {//epic java syntax
                bot.drive.driveFieldCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed,
                        (  Math.abs(avgGyroAngle - gyroAngle0) < gyroTolerance
                                || Math.abs(avgGyroAngle - gyroAngle1) < gyroTolerance) ?
                                Math.abs(gyroAngle0 - avgGyroAngle) <
                                        Math.abs(gyroAngle1 - avgGyroAngle) ?
                                        gyroAngle0 : gyroAngle1 : avgGyroAngle
                        // Epic Java Syntax here
                        /*
                         * In theory, this check ensures that when the avgGyroAngle is VERY off
                         * due to one IMU giving ~0.01, and the second giving ~1.99 which SHOULD be considered an angle of 2 or 0
                         * This problem was encountered while first testing the dual IMU dependant field centric drive
                         * the robot would run two motors on the corners of the robot in opposite directions, causing negligible movement
                         * Because I believe the rarer incorrect averages, these ternary statements, should correct this.
                         */
                );
            }
            else if (dpadPressed || buttonPressed) {
                double tempDriveSpeed = driveSpeed *= SLOW_MODE_PERCENT;
                bot.drive.driveRobotCentric(
                        strafeSpeed * tempDriveSpeed,
                        forwardSpeed * -tempDriveSpeed,
                        turnSpeed * tempDriveSpeed
                );
            }

            else {
                bot.drive.driveRobotCentric(
                        driveVector.getY() * driveSpeed,
                        driveVector.getX() * -driveSpeed,
                        turnVector.getX() * driveSpeed
                );
            }

        }

        /* set field centric offset
         * imu is inertial measurement unit, is in the control hubs and is set at 0 when the robot is started
         * offset is set at the angle the imu measures from where it was started, allowing calibration of field centricity
         */
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            fieldCentricOffset0 = bot.imu0.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
            fieldCentricOffset1 = bot.imu1.getAngularOrientation()
                    .toAngleUnit(AngleUnit.DEGREES).firstAngle;
        }

        // switch centricity mode
        if (gamepadEx1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            centricity = !centricity;
        }
    }
}
