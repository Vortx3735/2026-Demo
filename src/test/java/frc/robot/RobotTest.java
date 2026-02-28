package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  public void testRobotInit() {
    // Initialize the HAL (Hardware Abstraction Layer)
    // We use assertTrue to ensure it initialized correctly, and avoid side-effects in assertions
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

    // Create the Robot object.
    // In this project, RobotContainer is instantiated in the Robot constructor.
    Robot robot = new Robot();

    assertNotNull(robot, "Robot should be instantiated successfully");

    // Attempt to call robotInit if possible, to verify further initialization logic
    // robot.robotInit(); // robotInit is protected in IterativeRobotBase, but we are in the same
    // package.
    // However, since Robot.java doesn't override it and logic is in constructor, constructor is the
    // main check.

    // Close the robot to free resources
    robot.close();
  }
}
