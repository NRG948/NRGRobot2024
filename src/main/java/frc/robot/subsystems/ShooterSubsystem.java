
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) * 3
      / MotorParameters.NeoV1_1.getFreeSpeedRPM();
  private final CANSparkFlex shooterMotor = new CANSparkFlex(
      RobotConstants.CAN.SparkMax.SHOOTER_PORT, MotorType.kBrushless);

  private double currentRPM;
  private double goalRPM = 0;

  private final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  private PIDController shooterPIDController = new PIDController(0.002088, 0, 0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private double shooterVoltage;
  private boolean isEnabled = false;

  private BooleanLogEntry enabledLogger = new BooleanLogEntry(DataLogManager.getLog(), "/Shooter/Enabled");
  private DoubleLogEntry goalRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Goal RPM");
  private DoubleLogEntry shooterRPMLogger = new DoubleLogEntry(DataLogManager.getLog(), "/Shooter/Shooter RPM");

  /** Creates ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterEncoder.setVelocityConversionFactor(0.333);
  }

  private void setGoalRPMInternal(double goalRPM) {
    this.goalRPM = goalRPM;
    shooterPIDController.setSetpoint(goalRPM);
    goalRPMLogger.append(goalRPM);
  }

  public void setGoalRPM(double goalShooterRPM) {
    if (!isEnabled) {
      enabledLogger.append(true);
    }

    isEnabled = true;
    setGoalRPMInternal(goalShooterRPM);
  }

  /** Disables the Shooter PID controller and stops the motor. */
  public void disable() {
    if (isEnabled) {
      enabledLogger.append(false);
      goalRPMLogger.append(0);
    }

    isEnabled = false;
    stopMotor();
    shooterPIDController.reset();
    goalRPM = 0;
  }

  /** Stops Shooter Motor. */
  public void stopMotor() {
    shooterMotor.stopMotor();
  }

  /**
   * Parameter depends on # of motors
   * tempVoltage temporary # of parameter
   */
  public void setMotorVoltage(double shooterVoltage) {
    shooterMotor.setVoltage(shooterVoltage);
  }

  public double getRPM() {
    return currentRPM;
  }

  public double getGoalRPM() {
    return goalRPM;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  @Override
  public void periodic() {

    currentRPM = shooterEncoder.getVelocity();

    if (isEnabled) {
      double shooterFeedForwardVoltage = feedforward.calculate(goalRPM);

      shooterVoltage = shooterFeedForwardVoltage + shooterPIDController.calculate(currentRPM);

      setMotorVoltage(shooterVoltage);
    }
    shooterRPMLogger.append(currentRPM);
  }

  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout shooterLayout = tab.getLayout("Shooter", BuiltInLayouts.kList)
      .withPosition(2,0)
      .withSize(2, 3);

    shooterLayout.addDouble("Goal RPM", () -> goalRPM);
    shooterLayout.addDouble("Current RPM", () -> currentRPM);
    shooterLayout.addBoolean("Enabled", () -> isEnabled);
  }
}
