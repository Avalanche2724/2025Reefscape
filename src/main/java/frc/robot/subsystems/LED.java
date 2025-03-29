package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class LED extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 18;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  // Fields for blue fire simulation
  private final List<Flame> flames = new ArrayList<>();
  private final int simulationBarLength = kLength; // Use LED strip length for simulation
  private final int notActuallyMaxIntensity = 16;
  private final Random random = new Random();

  public LED() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to netchecker (which now uses our blue fire simulation)
    setDefaultCommand(netchecker().withName("ledchanger"));
  }

  @Override
  public void periodic() {
    // Continuously update the LED strip with the current buffer data.
    m_led.setData(m_buffer);
  }

  /**
   * Creates a command that runs a given LED pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   * @return a command that applies the pattern
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer)).ignoringDisable(true);
  }

  public Command thingy() {
    return runPattern(
        LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(90)));
  }

  public Command thingy2() {
    return runPattern(LEDPattern.solid(Color.kBlack));
  }

  /**
   * The netchecker command evaluates robot state and applies different LED patterns:
   *
   * <ul>
   *   <li>If near target position: solid red (with rumble)
   *   <li>If holding a game piece: solid green (with rumble)
   *   <li>Otherwise, if coral mode is off: solid blue
   *   <li>If coral mode is on: blue fire simulation is used to animate the LED strip
   * </ul>
   */
  public Command netchecker() {
    return run(() -> {
          var position = Robot.instance.robotContainer.drivetrain.getState().Pose.getX();
          var hasGamePiece = Robot.instance.robotContainer.intake.hasGamePiece();
          var coralMode = Robot.instance.robotContainer.controls.isOnCoralBindings;

          if (Math.abs(position - AllianceFlipUtil.applyX(7.4)) < Meters.convertFrom(1.5, Inch)) {
            LEDPattern.solid(Color.kRed).applyTo(m_buffer);
            Robot.instance.robotContainer.controls.driver.rumble.accept(0.7);
          } else if (hasGamePiece) {
            LEDPattern.solid(Color.kGreen).applyTo(m_buffer);
            Robot.instance.robotContainer.controls.driver.rumble.accept(0.2);
          } else {
            Robot.instance.robotContainer.controls.driver.rumble.accept(0);
            if (!coralMode) {
              LEDPattern.solid(Color.kBlue).applyTo(m_buffer);
            } else {
              // Blue fire simulation:
              // • Create a new flame and add it to the simulation.
              // • Update all flames (each flame’s length and position evolve over time).
              // • Remove any flames that have “died.”
              // • Build a fire-intensity list (one intensity value per LED).
              // • Map the intensity values into RGB colors for each LED.
              Flame newFlame = buildFlame(simulationBarLength);
              flames.add(newFlame);

              updateFlames(flames, simulationBarLength);
              flames.removeIf(flame -> flame.getLength() <= 0);

              List<Integer> fires = new ArrayList<>();
              for (int i = 0; i < simulationBarLength; i++) {
                fires.add(0);
              }
              makeFireList(fires, flames);
              updateLEDs(fires, notActuallyMaxIntensity);
            }
          }
          // Update the LED strip with the new data.
          m_led.setData(m_buffer);
        })
        .ignoringDisable(true);
  }

  // --- Blue Fire Simulation Methods ---

  /**
   * Creates a new flame for the simulation.
   *
   * @param barLength the length of the simulation (number of LEDs)
   * @return a new Flame instance with randomized length
   */
  private Flame buildFlame(int barLength) {
    int num = Math.round((float) (0.9 * barLength));
    int length1 = random.nextInt(num) + 1;
    int length = Math.round((float) (length1 * length1 / num));
    return new Flame(length, 0);
  }

  /**
   * Updates each flame’s length and position to simulate fire evolution.
   *
   * @param flames the list of flames to update
   * @param barLength the total number of LEDs (simulation width)
   */
  private void updateFlames(List<Flame> flames, int barLength) {
    for (Flame flame : flames) {
      int length = flame.getLength();
      int position = flame.getPosition();

      int lengthNum = random.nextInt(2) + 1;
      if (lengthNum == 1) {
        length = length - 1;
      }
      int posNum = random.nextInt(4) + 1;
      if (posNum == 1) {
        position++;
      }
      if (position + length >= barLength) {
        length = length - 1;
      }
      flame.setLength(length);
      flame.setPosition(position);
    }
  }

  /**
   * Aggregates the contributions from each flame into a fire intensity list.
   *
   * @param fires a list of intensity values (one per LED) that will be updated
   * @param flames the list of flames contributing to the intensity
   */
  private void makeFireList(List<Integer> fires, List<Flame> flames) {
    for (Flame flame : flames) {
      int length = flame.getLength();
      int position = flame.getPosition();
      for (int i = 0; i < length; i++) {
        int idx = position + i;
        if (idx < fires.size()) {
          fires.set(idx, fires.get(idx) + 1);
        }
      }
    }
  }

  /**
   * Updates the LED buffer colors based on the fire intensity values.
   *
   * @param fires the list of fire intensity values (one per LED)
   * @param maxIntensity the maximum expected intensity used for scaling
   */
  private void updateLEDs(List<Integer> fires, int maxIntensity) {
    for (int i = 0; i < fires.size(); i++) {
      int fire = fires.get(i);
      if (fire > maxIntensity) {
        // When intensity is very high, use a bright blue color.
        m_buffer.setLED(i, new Color(100, 200, 255));
      } else {
        int red = Math.round(fire * 75 / (float) maxIntensity);
        int green = Math.round(fire * 135 / (float) maxIntensity);
        int blue = Math.round(fire * 255 / (float) maxIntensity);
        m_buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  // --- Inner Class for Flame Simulation ---
  private static class Flame {
    private int length;
    private int position;

    public Flame(int length, int position) {
      this.length = length;
      this.position = position;
    }

    public int getLength() {
      return length;
    }

    public int getPosition() {
      return position;
    }

    public void setLength(int length) {
      this.length = length;
    }

    public void setPosition(int position) {
      this.position = position;
    }
  }
}
