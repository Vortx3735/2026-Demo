package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.WLEDController;

public class LEDSubsystem extends SubsystemBase {

  public LEDSubsystem() {}

  public enum LEDState {
    OFF(-1),
    IDLE(1),
    HAS_TAG(2),
    NO_TAG(3);

    private final int presetId;

    LEDState(int id) {
      this.presetId = id;
    }
  }

  private final WLEDController wled = new WLEDController(Constants.WLED_IP);
  private LEDState currentState = LEDState.OFF;

  public void setState(LEDState newState) {
    if (newState == currentState) return; // Optimization

    if (newState == LEDState.OFF) {
      wled.setPower(false);
    } else {
      wled.setPower(true);
      wled.setPreset(newState.presetId);
    }

    currentState = newState;
  }

  public Command setStateCommand(LEDState state) {
    return run(() -> setState(state));
  }

  // @Override
  // public void periodic() {
  // if (DriverStation.isDisabled()) {
  //   setState(LEDState.IDLE);
  //   return;
  // }
  // if (hasTag.getAsBoolean()) {
  //   this.setState(LEDState.HAS_TAG);
  // } else {
  //   this.setState(LEDState.NO_TAG);
  // }
  // }
}
