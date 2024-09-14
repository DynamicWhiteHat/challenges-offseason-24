package frc.robot.subsystems.intake.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class RollerIOSim implements RollerIO {
  // For instructions on how to implement this class, refer to the README.md file

  private DCMotorSim m_sim;
  // define more members here as necessary
  private double truevoltage;

  public RollerIOSim() {
    // TODO: Implement this constructor
    m_sim =
        new DCMotorSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.kRollerGearing,
            IntakeConstants.kRollerJKgMetersSquared);
  }

  @Override
  public void updateInputs(RollerInputs inputs) {
    m_sim.update(0.02);
    inputs.voltage = getVoltage();
    inputs.velocityRadPerSec = getVelocityRadPerSec();
  }

  @Override
  public void setVoltage(double voltage) {
    // TODO: Implement this method
    truevoltage = voltage;
    m_sim.setInputVoltage(voltage);
  }

  @Override
  public double getVoltage() {
    // TODO: Implement this method
    return truevoltage;
  }

  @Override
  public double getVelocityRadPerSec() {
    // TODO: Implement this method
    return m_sim.getAngularVelocityRadPerSec();
  }
}
