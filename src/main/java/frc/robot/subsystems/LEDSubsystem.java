package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.WLEDController;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  public LEDSubsystem() {}

  public enum LEDState {
    OFF(-1),
    IDLE(1),
    CHASE(2),
    RED(3),
    BLUE(4),
    GREEN(5),
    RED_BLINK(6),
    BLUE_BLINK(7),
    GREEN_BLINK(8);

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

  public LEDState getState() {
    return currentState;
  }

  public Command setStateCommand(LEDState state) {
    return run(() -> setState(state));
  }

  public Command blinkCommand() {
    if (getState() == LEDState.BLUE) {
      return setStateCommand(LEDState.BLUE_BLINK);
    } else if (getState() == LEDState.RED) {
      return setStateCommand(LEDState.RED_BLINK);
    } else {
      return setStateCommand(LEDState.GREEN_BLINK);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          this.setState(LEDState.RED);
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
          this.setState(LEDState.BLUE);
        } else {
          this.setState(LEDState.IDLE);
        }
      } else {
        this.setState(LEDState.IDLE);
      }
    }
    Logger.recordOutput("LED/currentState", getState());
  }
}
