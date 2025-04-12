package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.*
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Shooter.ShootingSpeed
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

    private fun teleopPointAtSpeaker(): Command =
        startEnd(
            { drivetrain.weShouldBePointingAt = drivetrain.speakerLocation },
            { drivetrain.weShouldBePointingAt = null })

    private fun getBetterShooterSpeeds() = shooter.speedFromDistance(
        drivetrain.distanceToSpeaker - Units.inchesToMeters(16.5) // reference point at speaker
    )

    fun teleopShoot(): Command =
        race(
            teleopPointAtSpeaker(),
            race(
                shooter.speedCmd(::getBetterShooterSpeeds),
                sequence(
                    waitUntil {
                        shooter.atDesiredSpeeds() && drivetrain.goodPointingToSpeaker && drivetrain.notActivelyMoving
                    },
                    race(bothIntake(), waitSeconds(0.4))
                )
            )
        )


    private val shootDelayTime = if (TimedRobot.isSimulation()) 1.0 else 5.0
    fun registerAutoCommands() {
        NamedCommands.registerCommands(
            mapOf(
                "shoot" to simpleShoot { ShootingSpeed.SUBWOOFER.speeds }.raceWith(waitSeconds(5.0)),
                "superShoot" to superShoot { ShootingSpeed.SUBWOOFER.speeds }.raceWith(
                    waitSeconds(
                        shootDelayTime
                    )
                ),
                "intake" to superIntake(),
                // spin up motor shooter
                "coolSpinShot" to shooter.speedCmdUnending { ShootingSpeed.AUTOSHOT.speeds },
                "coolShot" to simpleShoot { ShootingSpeed.AUTOSHOT.speeds }.raceWith(
                    waitSeconds(
                        shootDelayTime
                    )
                ),
                "subSpinShot" to shooter.speedCmdUnending { ShootingSpeed.SUBWOOFER.speeds },
                "subShot" to simpleShoot { ShootingSpeed.SUBWOOFER.speeds }.raceWith(
                    waitSeconds(
                        shootDelayTime
                    )
                ),
                "intakeAndSpin" to parallel(
                    superIntake(),
                    shooter.speedCmdUnending { ShootingSpeed.AUTOSHOT.speeds }),
                "intakeAndSubSpin" to parallel(
                    superIntake(),
                    shooter.speedCmdUnending { ShootingSpeed.SUBWOOFER.speeds })

            )
        )
    }
}
