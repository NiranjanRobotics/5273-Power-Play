package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.hardware.Bot;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.paths.AutoPaths;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.pipeline.AprilTagDetectionPipeline;

@Autonomous(name = "Main Autonomous", group = "Competition")
public class MainAutonomous extends LinearOpMode {//TODO: add reversing for competition

    private Bot bot;

    double confidence;
    AprilTagDetectionPipeline pipeline;
    boolean performActions = true;
    GamepadEx gamepad;


    @Override
    public void runOpMode() throws InterruptedException {
        Bot.instance = null;

        bot = Bot.getInstance(this);
        gamepad = new GamepadEx(gamepad1);

        AutoPaths paths = new AutoPaths(this);

        // telemetry.addData("Side", GlobalConfig.side);
        telemetry.addData("Alliance", GlobalConfig.alliance);
        telemetry.update();


        // pipeline = new AprilTagDetectionPipeline(this, telemetry);

        //TODO: add initialization here

        //bot.roadRunner.setPoseEstimate(paths.initialPosition());

        //TODO: add pipeline here

        while (!isStarted()) {
            if (isStopRequested())
                return;
            if (gamepad1.x) {
                performActions = false;
            }
            if (gamepad1.y) {
                // pipeline.saveImage();
            }
        }

        waitForStart();

        // List<AutoPaths.AutoPathElement> trajectories = paths.getTrajectories(detected);
        // pipeline.close();


        // Roadrunner


        if (isStopRequested())
            return;

        /*

        // TODO: add path logic here

        for (AutoPaths.AutoPathElement item : trajectories) {

            telemetry.addData("executing path element", item.getName());
            telemetry.update();

            if (item instanceof AutoPaths.AutoPathElement.Path) {
                bot.roadRunner.followTrajectory(((AutoPaths.AutoPathElement.Path) item).getTrajectory());
            } else if (item instanceof AutoPaths.AutoPathElement.Action && performActions) {
                ((AutoPaths.AutoPathElement.Action) item).getRunner().invoke();
            }

            if (isStopRequested())
                return;
        }

         */

        // poseEstimate = bot.roadRunner.getPoseEstimate();

    }
}
