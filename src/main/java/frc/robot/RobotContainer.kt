// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter

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

    val autonomousCommand = Commands.none()

    init {
        // Configure controller
        controls.configureDriveBindings()
        controls.configureNonDriveBindings()
    }
}
