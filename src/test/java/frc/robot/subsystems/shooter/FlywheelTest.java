package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class FlywheelTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Flywheel flywheel =
        new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertNotNull(flywheel, "Flywheel should be instantiated successfully");
  }

  @Test
  public void testInitialTargetRps() {
    Flywheel flywheel =
        new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertTrue(flywheel.targetrps == 0.0, "Initial targetrps should be 0");
  }

  @Test
  public void testIsAtSpeedInitially() {
    Flywheel flywheel =
        new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    // Both currentrps and targetrps start at 0, so isAtSpeed should return true
    assertTrue(flywheel.isAtSpeed(), "Flywheel should be at speed when both velocities are 0");
  }

  @Test
  public void testStopResetsTargetRps() {
    Flywheel flywheel =
        new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    flywheel.targetrps = 50.0;
    flywheel.stop();
    assertTrue(flywheel.targetrps == 0.0, "targetrps should be 0 after stop()");
  }

  @Test
  public void testCommandsNotNull() {
    Flywheel flywheel =
        new Flywheel(Constants.FlywheelConstants.FLYWHEEL_MOTOR_ID, Mode.SIM);
    assertNotNull(flywheel.setVelocityPIDCommand(), "setVelocityPIDCommand should not be null");
    assertNotNull(flywheel.stopCommand(), "stopCommand should not be null");
    assertNotNull(flywheel.shootCommand(), "shootCommand should not be null");
  }
}
