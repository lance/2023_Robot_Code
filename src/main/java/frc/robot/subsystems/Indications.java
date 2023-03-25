package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIndications;
import frc.robot.utilities.LEDSubStrip;
import java.util.EnumSet;
import java.util.HashMap;

public class Indications extends SubsystemBase {

  // The NetworkTable table name that subsystems will use to report their state
  public static final String INDICATIONS_TABLE = "Indications";

  // The Topic name that subsystems will publish their states to
  public static final String CURRENT_STATE_TOPIC = "/states/current";

  // A StringTopic instance where state is stored
  private static StringTopic currentStateTopic;

  // Enumerate the various robot states that we want to
  // indicate with the LEDs
  public enum RobotStates {
    DRIVE_FORWARD,
    DRIVE_BACKWARD,
    OFF,
    ERROR,
    UNKNOWN,
    // etc...
  }

  public interface Indicator {
    public void indicate();
  }

  private AddressableLED armLEDs;
  private AddressableLEDBuffer armLEDsBuffer;

  @SuppressWarnings("unused")
  private LEDSubStrip proximalLeftStrip;

  private HashMap<RobotStates, Indicator> stateIndications;
  private StringEntry currentStateEntry;
  private RobotStates currentState = RobotStates.OFF;

  public Indications() {
    // TODO Add constants for start and end locations and move strip constants out of kSensors
    armLEDs = new AddressableLED(kIndications.ledPort);
    armLEDsBuffer = new AddressableLEDBuffer(kIndications.ledLength);
    armLEDs.setLength(armLEDsBuffer.getLength());
    proximalLeftStrip = new LEDSubStrip(armLEDsBuffer, 0, 60);
    armLEDs.setData(armLEDsBuffer);
    armLEDs.start();
    stateIndications = new HashMap<RobotStates, Indicator>();
    stateIndications.put(
        RobotStates.DRIVE_FORWARD,
        () -> {
          this.monotone(proximalLeftStrip, Color.kGreen);
        });
    stateIndications.put(
        RobotStates.DRIVE_BACKWARD,
        () -> {
          this.monotone(proximalLeftStrip, Color.kRed);
        });
    stateIndications.put(
        RobotStates.OFF,
        () -> {
          this.monotone(proximalLeftStrip, Color.kBlack);
        });
    stateIndications.put(
        RobotStates.ERROR,
        () -> {
          this.monotone(proximalLeftStrip, Color.kRed);
        });
    stateIndications.put(
        RobotStates.UNKNOWN,
        () -> {
          this.monotone(proximalLeftStrip, Color.kOrange);
        });

    // Initial state is OFF
    this.currentStateEntry = Indications.getCurrentStateTopic().getEntry(currentState.name());

    // Listen for changes to the state
    NetworkTableInstance.getDefault()
        .addListener(
            currentStateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kPublish),
            event -> {
              if (event.valueData != null) {
                tryIndicating(event.valueData.value.getString());
              }
            });
  }

  /**
   * Commands that want to publish a state indication should publish to this topic in NetworkTables.
   * For example:
   *
   * <p>indications.getCurrentStateTopic().set(RobotStates.DRIVE_FORWARD.name());
   *
   * @return the topic for publishing robot states
   */
  public static StringTopic getCurrentStateTopic() {
    if (Indications.currentStateTopic == null) {
      // Get the default network table instance
      NetworkTableInstance instance = NetworkTableInstance.getDefault();

      // Create a new table for indications
      NetworkTable indicationsTable = instance.getTable(Indications.INDICATIONS_TABLE);

      // We will store the current state at CURRENT_STATE_TOPIC (/states/current) allowing for other
      // entries in e.g. /states/history
      Indications.currentStateTopic =
          indicationsTable.getStringTopic(Indications.CURRENT_STATE_TOPIC);
    }
    return Indications.currentStateTopic;
  }

  /**
   * Use LED indicators to display the state of the robot
   *
   * @param state one of the known states
   */
  public void indicate(RobotStates state) {
    Indicator i = stateIndications.get(state);
    if (i != null) {
      i.indicate();
    }
  }

  public void monotone(LEDSubStrip section, Color color) {
    for (int i = 0; i <= section.getLength(); i++) {
      section.setLED(i, color);
    }
  }

  public void alternate(LEDSubStrip section, float interval, Color color1, Color color2) {
    // 2 Colored Pattern
    int patternSize = 2;

    // sets all of the LEDs between the first LED and the last LED indicated.
    for (int i = 0; i <= section.getLength(); i++) {
      // color 1
      if (((i + interval) % patternSize) < 1) {
        section.setLED(i, color1);

      } else {
        // color 2
        section.setLED(i, color2);
      }
    }
  }

  public void flashing(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i <= section.getLength(); i++) {
      // turns the LEDs onn if the interval is one
      if ((interval % 2) == 1) {
        section.setLED(i, color);
      } else {
        // sets the LEDs off
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void runway(LEDSubStrip section, Color color, int interval) {
    // finds many LEDS are in the section
    int sectionSize = section.getLength();

    for (int i = 0; i <= section.getLength(); i++) {
      // finds where is the sequence the "chasing" LED is
      if ((i % sectionSize) == interval) {
        armLEDsBuffer.setLED(i, color);
      } else {
        armLEDsBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Attempts to indicate the robot state through LEDs given a provided state name. The state name
   * should correspond to the name() value for a RobotState, e.g. RobotStates.DRIVE_FORWARD.name()
   *
   * @param possibleState
   */
  private void tryIndicating(String possibleState) {
    RobotStates state;
    try {
      // If a component has published an unknown state to the network tables,
      // RobotStates.valueOf() will throw IllegalArgumentException.
      state = RobotStates.valueOf(possibleState);
    } catch (IllegalArgumentException e) {
      state = RobotStates.UNKNOWN;
    }

    // If the current state has changed, indicate that
    if (this.currentState != state) {
      currentState = state;
      indicate(state);
    }
  }
}
