package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.networktables.DoubleEntry;
// NetworkTable imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final TalonFX indexerMotor;
  private final TalonFX rollerMotor;
  private double motorSpeed;

  // Network Table Entry
  final DoubleEntry indexerMotorSpeedEntry;

  public Indexer(int indexerID, int rollerID) {
    indexerMotor = new TalonFX(indexerID);
    rollerMotor = new TalonFX(rollerID);
    rollerMotor.setControl(
        new Follower(
            indexerID, MotorAlignmentValue.Opposed)); // TODO: change direction based on real robot
    // Indexer Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable indexerTable = inst.getTable("Indexer");
    indexerMotorSpeedEntry = indexerTable.getDoubleTopic("indexerMotorSpeed").getEntry(0);
    indexerMotorSpeedEntry.set(1);
  }

  public void setIndexerSpeed(double speed) {
    motorSpeed = speed;
  }

  public double getIndexerSpeed() {
    return motorSpeed;
  }

  public void run(Boolean inverted) {
    if (inverted) {
      indexerMotor.set(-motorSpeed);
    } else {
      indexerMotor.set(motorSpeed);
    }
  }

  public void stop() {
    indexerMotor.set(0);
  }

  public Command runIndexerCommand(Boolean inverted) {
    // Execute setIndexerSpeed AND set the motor every loop
    return run(() -> {
          setIndexerSpeed(indexerMotorSpeedEntry.getAsDouble());
          run(inverted); // Ensure the motor is actually updated
        })
        .withName("run indexer");
  }

  public Command stopCommand() {
    return new RunCommand(() -> stop(), this).withName("stop indexer");
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/motorSpeed", motorSpeed);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    Logger.recordOutput("Indexer/simulatedVoltage", indexerMotor.getSimState().getMotorVoltage());
  }
}
