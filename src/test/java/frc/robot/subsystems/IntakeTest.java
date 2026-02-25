package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.Constants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class IntakeTest {

  @BeforeAll
  static void halInit() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  @Test
  public void testInitialization() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    assertNotNull(intake, "Intake should be instantiated successfully");
  }

  @Test
  public void testDefaultSpeed() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    assertEquals(0.25, intake.getSpeed(), 1e-6, "Default intake speed should be 0.25");
  }

  @Test
  public void testSetSpeed() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    intake.setSpeed(0.75);
    assertEquals(0.75, intake.getSpeed(), 1e-6, "Intake speed should be updated to 0.75");
  }

  @Test
  public void testSetSpeedZero() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    intake.setSpeed(0.0);
    assertEquals(0.0, intake.getSpeed(), 1e-6, "Intake speed should be set to 0");
  }

  @Test
  public void testCommandsNotNull() {
    Intake intake = new Intake(Constants.IntakeConstants.INTAKE_MOTOR_ID);
    assertNotNull(intake.intakeCommand(), "intakeCommand should not be null");
    assertNotNull(intake.stopCommand(), "stopCommand should not be null");
  }
}
