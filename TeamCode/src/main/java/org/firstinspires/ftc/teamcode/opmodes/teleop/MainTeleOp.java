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

//      Update
//      cycle = 1.0/(time-prevRead);
//      prevRead = time;
//      timingScheduler.run();

//      Movement =================================================================================================
        drive();

        //Subsystem Control =========================================================================================

        if (gamepadEx1.isDown(GamepadKeys.Button.LEFT_BUMPER)){ }

        else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT){ }

        else { }

        if (gamepadEx1.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { }

        else if (gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) { }

        else { }


        if (gamepadEx1.wasJustPressed(Button.RIGHT_STICK_BUTTON)) { }

        if (gamepadEx1.wasJustPressed(Button.LEFT_STICK_BUTTON)) { }


        if(gamepadEx1.wasJustPressed(Button.DPAD_RIGHT)) { }

        else if(gamepadEx1.wasJustPressed(Button.DPAD_LEFT)) { }

        else if(gamepadEx1.wasJustPressed(Button.DPAD_UP)) { }

        else if(gamepadEx1.wasJustPressed(Button.DPAD_DOWN)) { }


        if (gamepadEx1.wasJustPressed(Button.A)){ }

        if (gamepadEx1.wasJustPressed(Button.B)) { }

        if (gamepadEx1.wasJustPressed(Button.X)) { }

        if (gamepadEx1.wasJustPressed(Button.Y)) { }


        // Driver 2



        if (gamepadEx2.isDown(GamepadKeys.Button.LEFT_BUMPER)){ }

        else if (gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > TRIGGER_CONSTANT){ }

        else { }


        if (gamepadEx2.isDown(GamepadKeys.Button.RIGHT_BUMPER)) { }

        else if (gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_CONSTANT) { }

        else { }
        


        if (gamepadEx2.wasJustPressed(Button.A)) { }

        if (gamepadEx2.wasJustPressed(Button.B)) { }

        if (gamepadEx2.wasJustPressed(Button.X)) { }

        if (gamepadEx2.wasJustPressed(Button.Y)) { }


        /*
        Controller 1
        Buttons
            A: intake in
            B: hold to release claw, release to reset outtake to base position
            X: intake out
            Y: raise slides to set positions
        DPAD (unused)
        Joystick
            L: movement (field centric or robot centric)
            R: Set orientation / rotation (determine through practice)
        Trigger
            L: reset field centric offset
            R: switch centricity
        Bumper:
            L: decrement slide height selection
            R: increment slide height selection
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

        final double gyroTolerance = 10;

        double tempAngle0 = bot.imu0.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset0;
        double tempAngle1 = bot.imu1.getAngularOrientation().toAngleUnit(AngleUnit.DEGREES).firstAngle
                - fieldCentricOffset1;

        // set absolute value of angle always less than or equal to 180

        final double gyroAngle0 = tempAngle0; // accounts for rotation of extension hub and center-lifts angle to -180->180

        // if imu is null, then use other imu

        final double gyroAngle1 = (bot.imu1 != null) ? tempAngle1 :gyroAngle0;
        final double avgGyroAngle = ((gyroAngle0 + gyroAngle1)/2);
        telemetry.addData("avgGyroAngle" ,avgGyroAngle);

        telemetry.addData("tempAngle0", tempAngle0);
        telemetry.addData("tempAngle1", tempAngle1);
        telemetry.addData("gyroAngle0", gyroAngle0);
        telemetry.addData("gyroAngle1", gyroAngle1);
        telemetry.addData("fieldCentricOffset0", fieldCentricOffset0);
        telemetry.addData("fieldCentricOffset1", fieldCentricOffset1);

        Vector2d driveVector = new Vector2d(gamepadEx1.getLeftY(), gamepadEx1.getLeftX()),
                turnVector = new Vector2d(
                        gamepadEx1.getRightX() , 0);
        if (bot.roadRunner.mode == RRMecanumDrive.Mode.IDLE) {

            boolean dpadPressed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)
                    || gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT));
            boolean buttonPressed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B));

            double forwardSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_RIGHT) ? 1 : -1) : 0;
            double strafeSpeed = (gamepadEx1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP)) ? (gamepadEx1.getButton(GamepadKeys.Button.DPAD_UP) ? 1 : -1) : 0;
            double turnSpeed = (gamepadEx1.getButton(GamepadKeys.Button.X) || gamepadEx1.getButton(GamepadKeys.Button.B)) ? (gamepadEx1.getButton(GamepadKeys.Button.B) ? 1 : -1) : 0;

            if (centricity) {//epic java syntax
                bot.drive.driveFieldCentric(
                        driveVector.getX() * driveSpeed,
                        driveVector.getY() * driveSpeed,
                        turnVector.getX() * driveSpeed,
                         Math.abs(gyroAngle1 - gyroAngle0) < gyroTolerance ? avgGyroAngle : gyroAngle0

                        //field centric W


                        // Epic Java Syntax here
                        /*
                         * In theory, this check ensures that when the avgGyroAngle is VERY off
                         * due to one IMU giving  near -180, and the second giving near 180 which SHOULD be considered an angle of 0 but its actually in the opposite direction
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
                        driveVector.getX() * driveSpeed,
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
