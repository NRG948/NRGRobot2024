
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.EnumSet;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotConstants.ManipulatorConstants;
import frc.robot.parameters.MotorParameters;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private static final double KS = 0.15;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) * 3
      / MotorParameters.NeoV1_1.getFreeSpeedRPM();

  public enum GoalShooterRPM {
    STOP(0.0),
    SHOOTING(69); // TODO: Get Real RPM

    private final double rpm;
    
    GoalShooterRPM(double rpm) {
      this.rpm = rpm;
    }

    private double getRPM() {
      return rpm;
    }
  }

  private final CANSparkFlex shooterMotor = new CANSparkFlex(ManipulatorConstants.kShooterMotorPort,
      MotorType.kBrushless);

  private double currentShooterRPM;
  private GoalShooterRPM currentGoalRPM = GoalShooterRPM.STOP;

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

  private void setGoalRPMInternal(GoalShooterRPM goalRPM) {
    currentGoalRPM = goalRPM;
    shooterPIDController.setSetpoint(goalRPM.getRPM());
    goalRPMLogger.append(currentGoalRPM.getRPM());
  }

  public void setGoalRPM(GoalShooterRPM goalShooterRPM) {
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
    currentGoalRPM = GoalShooterRPM.STOP;
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
    return currentShooterRPM;
  }

  public GoalShooterRPM getCurrentGoalRPM() {
    return currentGoalRPM;
  }

  public boolean isEnabled() {
    return isEnabled;
  }

  @Override
  public void periodic() {

    currentShooterRPM = shooterEncoder.getVelocity();

    if (isEnabled) {
      double shooterFeedForwardVoltage = feedforward.calculate(currentGoalRPM.getRPM());

      shooterVoltage = shooterFeedForwardVoltage + shooterPIDController.calculate(currentShooterRPM);

      setMotorVoltage(shooterVoltage);
    }
    shooterRPMLogger.append(currentShooterRPM);
  }
}
