package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

class AutoPaths(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

    sealed class AutoPathElement(open val name: String) {

        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)
        //AutoPathElement.Path(name, trajectory)

        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

}