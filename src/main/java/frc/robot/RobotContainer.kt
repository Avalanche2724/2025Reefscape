// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import java.util.Optional

class RobotContainer(var bot: Robot) {
    // Subsystems
    val shooter = Shooter()
    val indexer = Indexer()
    val intake = Intake()
    val drivetrain = TunerConstants.createDrivetrain()

    // Other things

    // Other necessary things
    val com = CommonCommands(this)
    private val controls = Controls(this)

    // Make outreach mode easier to access from other subsystems
    companion object {
        private var instance: RobotContainer? = null

        //val outreachMode
        //    get() = instance?.controls?.outreachMode == true
        val voltageMultiplier = 1.0
        //get() = if (outreachMode) 0.9 else 1.0


    }

    init {
        instance = this
    }

    private val chooser: SendableChooser<Command>
    val autonomousCommand: Command
        get() = chooser.selected ?: Commands.none()

    init {
        // Configure controller
        Controls.configureDriveBindings()
        if (doSysId && routine equals null) {
            Controls.configureSysIDBindings(routine)
        } else {
            Controls.configureNonDriveBindings()
        }

        // Set up drivetrain and auto
        CommonCommands.registerAutoCommands()
        CommandSwerveDrivetrain.configurePathPlanner()
        com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.registerTelemetry(Telemetry::telemeterize)
        if (TimedRobot.isSimulation()) {
            com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.seedFieldRelative(
                Pose2d(
                    Translation2d(),
                    Rotation2d.fromDegrees(90.0)
                )
            )
        }

        chooser = AutoBuilder.buildAutoChooser("NONE")
        chooser.addOption("SHOOT ONLY", CommonCommands.simpleShoot { Shooter.ShootingSpeed.speeds })
        SmartDashboard.putData("CHOOSE AUTO!!!", chooser)
    }

    fun updateVision() {
        Optional.ifPresent { est: org.photonvision.EstimatedRobotPose ->
            // Change our trust in the measurement based on the tags we can see
            val estStdDevs = frc.robot.Vision.estimationStdDevs
            com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.addVisionMeasurement(
                Pose3d.toPose2d(), org.photonvision.EstimatedRobotPose.timestampSeconds, estStdDevs
            )
        }
    }

    fun simulationPeriodic() {
        frc.robot.Vision.simulationPeriodic(com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState.Pose)

        val debugField = frc.robot.Vision.simDebugField!!
        FieldObject2d.setPose = com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState.Pose
        FieldObject2d.setPoses(CommandSwerveDrivetrain.modulePoses)
    }

}
