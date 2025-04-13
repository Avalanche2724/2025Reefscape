package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import frc.robot.subsystems.Shooter
import java.util.function.Supplier


class CommonCommands(private val bot: RobotContainer) {
    private val drivetrain = bot.drivetrain
    private val intake = bot.intake
    private val indexer = bot.indexer
    private val shooter = bot.shooter

    fun bothIntake(): Command =
        parallel(intake.intakeCmd(), indexer.intakeCmd())

    fun bothEject(): Command =
        parallel(intake.ejectCmd(), indexer.ejectCmd())

    // this entire thing is kinda cursed and probably doesn't completely comply with the command
    // lifecycle
    // and I haven't even tested to see if checking the sensor on a different periodic
    // even makes a noticeable difference!!!!!
    private var noteStopEnabled = false

    private fun Command.stopWhenNoteSensed() =
        FunctionalCommand(
            {
                initialize()
                noteStopEnabled = true
            },
            {
                if (noteStopEnabled) execute()
            },
            { interrupted: Boolean ->
                end(interrupted)
                noteStopEnabled = false
            },
            { (!noteStopEnabled) || isFinished }
        ).apply { addRequirements(*requirements.toTypedArray()) }

    init {
        bot.bot.addPeriodic({
            if (noteStopEnabled) {
                if (indexer.noteDetected.asBoolean) {
                    bot.indexer.stopMotor()
                    bot.intake.stopMotor()
                    noteStopEnabled = false
                }
            }
        }, 0.002)
    }

    // Intake and stop the command once note detected
    fun intakeUntilNote() =
        bothIntake().stopWhenNoteSensed()

    /**
     * Intakes and tries to fix stuck notes
     */
    fun superIntake() =
        repeatingSequence(
            bothIntake().until(intake.isMotorStalling),
            bothEject().until(intake.isMotorMovingBack.and(indexer.isMotorMovingBack))
        ).stopWhenNoteSensed().unless(indexer.noteDetected)

    fun simpleShoot(speeds: Supplier<Shooter.ShootingSpeed.Speeds>): Command =
        race(
            shooter.speedCmd(speeds),
            sequence(
                waitUntil { bot.shooter.atDesiredSpeeds(speeds) },
                race(bothIntake(), waitSeconds(0.4))
            )
        )

    fun superShoot(speeds: Supplier<Shooter.ShootingSpeed.Speeds>): Command =
        sequence(
            race(superIntake(), waitSeconds(3.0)),
            simpleShoot(speeds)
        )


    private val shootDelayTime = if (TimedRobot.isSimulation()) 1.0 else 5.0
}
