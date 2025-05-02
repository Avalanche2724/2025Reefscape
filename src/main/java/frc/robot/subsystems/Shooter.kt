package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Shooter.ShootingSpeed.Speeds
import java.util.function.Supplier
import kotlin.math.absoluteValue


// possible TODO: telemetry, supply limits, current detection, simulation
private const val TALONFX_ID_TOP = 12
private const val TALONFX_ID_BOTTOM = 0

private const val SPEED_TOL = 0.25;

class Shooter : SubsystemBase() {
    private val topMotor = TalonFX(TALONFX_ID_TOP).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                kS = 0.15545
                kV = 0.12693
                kP = 0.19241
            }
        })
    }
    private val bottomMotor = TalonFX(TALONFX_ID_BOTTOM).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                kS = 0.10585
                kV = 0.12615
                kP = 0.19302
            }
        })
    }
    private val controlTop = VelocityVoltage(0.0)
    private val controlBottom = VelocityVoltage(0.0)


    init {
        defaultCommand = stopCmd()
    }

    private fun motorStop() {
        topMotor.set(0.0)
        bottomMotor.set(0.0)
    }


    fun atDesiredSpeeds(speed: Supplier<Speeds>) =
        (topMotor.velocity.valueAsDouble - speed.get().top / 60).absoluteValue < SPEED_TOL &&
                (bottomMotor.velocity.valueAsDouble - speed.get().bottom / 60).absoluteValue < SPEED_TOL

    fun atDesiredSpeeds() =
        (topMotor.velocity.valueAsDouble - lastSetTop).absoluteValue < SPEED_TOL &&
                (bottomMotor.velocity.valueAsDouble - lastSetBottom).absoluteValue < SPEED_TOL


    private var lastSetTop = 0.0
    private var lastSetBottom = 0.0

    fun runWithSpeed(speed: Speeds) {
        lastSetTop = speed.top / 60
        lastSetBottom = speed.bottom / 60
        topMotor.setControl(controlTop.apply { Velocity = lastSetTop })
        bottomMotor.setControl(controlBottom.apply { Velocity = lastSetBottom })
    }

    fun speedCmd(speed: Supplier<Speeds>) = runEnd({ runWithSpeed(speed.get()) }, ::motorStop)
    fun speedCmdUnending(speed: Supplier<Speeds>) = run { runWithSpeed(speed.get()) }


    fun stopCmd() = run { this.motorStop() }

    override fun periodic() {
        SmartDashboard.putNumber("Shooter velocity top", topMotor.velocity.valueAsDouble)
        SmartDashboard.putNumber("Shooter velocity bottom", bottomMotor.velocity.valueAsDouble)
        SmartDashboard.putNumber("Shooter last set top", lastSetTop)
        SmartDashboard.putNumber("Shooter last set bottom", lastSetBottom)
    }

    enum class ShootingSpeed(var speeds: Speeds) {
        AMP(Speeds(400.0, 1000.0)),
        AUTOSHOT(Speeds(1700.0, 2500.0)),

        OLDSUBWOOFER(Speeds(1200.0, 3600.0)), // previously 1200/3200

        //OLDSUBWOOFER(Speeds(1200.0, 3250.0)),
        SUBWOOFER(Speeds(1800.0, 4050.0)),

        //LINESHOT(Speeds(4000.0, 1800.0)),
        FARTHERSHOT(Speeds(3200.0, 2200.0)),
        OUTREACHSHOT1(Speeds(700.0, 1400.0)),
        OUTREACHSHOT2(Speeds(2000.0, 1000.0));

        data class Speeds(val top: Double, val bottom: Double)
    }

    data class ShooterCalibration(val distanceInches: Double, val speeds: Speeds)

    // Thanks to lynk for the data and code lol

    private val calibrations: Array<ShooterCalibration> = arrayOf(
        ShooterCalibration(35.9, Speeds(1200.0, 3200.0)),
        ShooterCalibration(46.9, Speeds(1500.0, 2500.0)),
        ShooterCalibration(59.5, Speeds(2200.0, 2200.0)),
        ShooterCalibration(72.2, Speeds(2700.0, 2200.0)),
        ShooterCalibration(83.5, Speeds(3000.0, 1900.0)),
        ShooterCalibration(95.7, Speeds(2900.0, 1700.0)),
        ShooterCalibration(108.1, Speeds(2800.0, 1550.0)),
        ShooterCalibration(120.9, Speeds(2800.0, 1425.0)),
        ShooterCalibration(132.0, Speeds(2700.0, 1400.0))
    )

    fun speedFromDistance(meters: Double): Speeds {
        val distance = Units.metersToInches(meters)
        var priorEntry: ShooterCalibration? = null
        var speed: Speeds? = null

        for (calibration in calibrations) {
            if (distance <= calibration.distanceInches) {
                if (priorEntry == null) {
                    // Closer than minimum calibration distance gets same speed as minimum distance
                    speed = calibration.speeds
                } else {
                    // Linear interpolation between calibration entries
                    val fraction =
                        (distance - priorEntry.distanceInches) / (calibration.distanceInches - priorEntry.distanceInches)
                    speed = Speeds(
                        fraction * calibration.speeds.top + (1 - fraction) * priorEntry.speeds.top,
                        fraction * calibration.speeds.bottom + (1 - fraction) * priorEntry.speeds.bottom
                    )
                }
                break
            }
            priorEntry = calibration
        }

        return speed ?: calibrations.last().speeds
    }

}
