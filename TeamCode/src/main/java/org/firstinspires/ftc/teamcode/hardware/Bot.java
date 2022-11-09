package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.drive.RRMecanumDrive;

public class Bot {
    // in TeleOp and Autonomous we should be able to call "new Bot(this)"

    public static Bot instance;

    //TODO: Declare subsystems here


    //required subsystems
    public final MecanumDrive drive;
    public final RRMecanumDrive roadRunner;
    public final BNO055IMU imu0;
    public final BNO055IMU imu1;
    public OpMode opMode;

    /** Get the current Bot instance from somewhere other than an OpMode */
    public static Bot getInstance() {
        if (instance == null) {
            throw new IllegalStateException("tried to getInstance of Bot when uninitialized");
        }
        return instance;
    }

    public static Bot getInstance(OpMode opMode) {
        if (instance == null) {
            return instance = new Bot(opMode);
        }
        instance.opMode = opMode;
        return instance;
    }

    public void reset(){
        //TODO: add reset code here
    }

    private Bot(OpMode opMode){
        this.opMode = opMode;
        enableAutoBulkRead();

        //TODO: initialize subsystems

        //this.templateSubsystem = new TemplateSubsystem(opMode);

        //required subsystems
        this.drive = new MecanumDrive(false,
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFL),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorFR),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBL),
                new MotorEx(opMode.hardwareMap, GlobalConfig.motorBR));
        this.roadRunner = new RRMecanumDrive(opMode.hardwareMap);

        imu0 = roadRunner.imu;
        imu1 = (roadRunner.imu2 != null) ? roadRunner.imu2 : null;
    }

//  private void initializeImu() {
//    final Parameters params = new Parameters();
//    params.angleUnit = AngleUnit.RADIANS;
//    imu.initialize(params);
//  }

    private void enableAutoBulkRead() {
        for (LynxModule mod : opMode.hardwareMap.getAll(LynxModule.class)) {
            mod.setBulkCachingMode(BulkCachingMode.AUTO);
        }
    }
}
