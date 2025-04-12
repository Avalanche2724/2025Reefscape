package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WrapperCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.RobotContainer

private const val TALONFX_ID = 5
private const val LEFT_SENSOR = 0
private const val RIGHT_SENSOR = 1

/** Subsystem for indexer (the spinny thingy between the intake and shooter)  */
class Indexer : SubsystemBase() {
    private var leftSensor = DigitalInput(LEFT_SENSOR)
    private var rightSensor = DigitalInput(RIGHT_SENSOR)

    /** True if left index sensor detects something  */
    private var leftTrigger = Trigger { !leftSensor.get() }

    /** True if right index sensor detects something  */
    private var rightTrigger = Trigger { !rightSensor.get() }

    /** True if a note is detected in the indexer  */
    var noteDetected: Trigger = leftTrigger.and(rightTrigger)

    private val motor = TalonFX(TALONFX_ID).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
            CurrentLimits.apply {
                StatorCurrentLimit = 80.0 // DO NOT TURN THIS DOWN, THE CURRENT IS NECESSARY TO FEED INTO SHOOTER
                StatorCurrentLimitEnable = true
            }
            Slot0.apply {
                kP = 0.35
            }
        })
    }

    private val control = VoltageOut(0.0).apply {
        UpdateFreqHz = 0.0
    }
    private val brake = VelocityVoltage(0.0).apply {
        UpdateFreqHz = 0.0
    }

    private val torqueCurrent = motor.torqueCurrent
    private val acceleration = motor.acceleration
    private val velocity = motor.velocity
    private val position = motor.position
    private val voltage = motor.motorVoltage
    private val signals = listOf(torqueCurrent, acceleration, velocity, position, voltage)
        .onEach { it.setUpdateFrequency(100.0) }

    val isMotorMovingBack = Trigger { velocity.valueAsDouble < -14 }

    init {
        defaultCommand = stopCmd()
    }

    private var loopsWithoutControlRequest = 0
    override fun periodic() {
        signals.onEach {
            it.refresh()
            SmartDashboard.putNumber("Indexer " + it.name, it.valueAsDouble)
        }

        SmartDashboard.putBoolean("Note sense left", leftTrigger.asBoolean)
        SmartDashboard.putBoolean("Note sense right", rightTrigger.asBoolean)

        loopsWithoutControlRequest++
        if (loopsWithoutControlRequest == 2) {
            setMotor(0.0)
        }
    }

    private fun setMotor(volts: Double) {
        loopsWithoutControlRequest = 0
        motor.setControl(
            if (volts == 0.0) brake
            else control.withOutput(volts * RobotContainer.voltageMultiplier)
        )
    }

    private fun setMotor(out: Output) = setMotor(out.volts)

    fun stopMotor() = setMotor(Output.STOP)

    private fun motorSpeedCmd(out: Output): WrapperCommand =
        runEnd({ setMotor(out) }, { setMotor(Output.STOP) })
            .withName("IndexerSpeed" + out.name)

    fun intakeCmd() = motorSpeedCmd(Output.INTAKE)
    fun ejectCmd() = motorSpeedCmd(Output.EJECT)
    fun stopCmd() = motorSpeedCmd(Output.STOP)

    enum class Output(val volts: Double) {
        // in units of volts
        INTAKE(9.0),
        EJECT(-9.0),
        STOP(0.0)
    }
}
