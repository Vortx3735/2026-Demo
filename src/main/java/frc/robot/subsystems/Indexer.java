package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private double motorSpeed;

  // Network Table Entry
  final DoubleEntry indexerMotorSpeedEntry;

  public Indexer(int motorId) {
    indexerMotor = new TalonFX(motorId);

    // Indexer Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable indexerTable = inst.getTable("Indexer");
    indexerMotorSpeedEntry = indexerTable.getDoubleTopic("indexerMotorSpeed").getEntry(0);
  }

  public void setIndexerSpeed(double speed) {
    motorSpeed = speed;
  }

  public double getIndexerSpeed() {
    return motorSpeed;
  }

  public void run() {
    indexerMotor.set(motorSpeed);
  }

  public void stop() {
    indexerMotor.set(0);
  }

  public Command runCommand(double speed) {
    // Execute setIndexerSpeed AND set the motor every loop
    return run(() -> {
      setIndexerSpeed(speed);
      run(); // Ensure the motor is actually updated
    }).withName("run indexer");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop indexer");
  }

  @Override
  public void periodic() {
    publishTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Publish telemetry to Network Table
  private void publishTelemetry() {
    // Use getDutyCycle() for Phoenix 6
    indexerMotorSpeedEntry.set(indexerMotor.getDutyCycle().getValueAsDouble());
  }
}
