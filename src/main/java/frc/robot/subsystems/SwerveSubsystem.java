// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.parameters.SwerveAngleEncoder;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.parameters.SwerveMotors;
import frc.robot.util.SwerveModuleVelocities;
import frc.robot.util.SwerveModuleVoltages;

@RobotPreferencesLayout(groupName = "Drive", column = 0, row = 1, width = 2, height = 3)
public class SwerveSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<SwerveDriveParameters> PARAMETERS = new RobotPreferences.EnumValue<SwerveDriveParameters>(
      "Drive", "Robot Base", SwerveDriveParameters.PracticeBase2024);
  
  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_DRIVE_TAB = new RobotPreferences.BooleanValue("Drive", "Enable Tab", false);

  public static boolean ENABLE_FIELD_TAB = false;

  public static final double DRIVE_KP = 1.0;

  private static final byte NAVX_UPDATE_FREQUENCY_HZ = 50;

  // 4 pairs of motors for drive & steering.
  private final TalonFX frontLeftDriveMotor = new TalonFX(
      PARAMETERS.getValue().getMotorId(SwerveMotors.FrontLeftDrive));
  private final CANSparkMax frontLeftSteeringMotor = new CANSparkMax(
      PARAMETERS.getValue().getMotorId(SwerveMotors.FrontLeftSteering), MotorType.kBrushless);

  private final TalonFX frontRightDriveMotor = new TalonFX(
      PARAMETERS.getValue().getMotorId(SwerveMotors.FrontRightDrive));
  private final CANSparkMax frontRightSteeringMotor = new CANSparkMax(
      PARAMETERS.getValue().getMotorId(SwerveMotors.FrontRightSteering), MotorType.kBrushless);

  private final TalonFX backLeftDriveMotor = new TalonFX(
      PARAMETERS.getValue().getMotorId(SwerveMotors.BackLeftDrive));
  private final CANSparkMax backLeftSteeringMotor = new CANSparkMax(
      PARAMETERS.getValue().getMotorId(SwerveMotors.BackLeftSteering), MotorType.kBrushless);

  private final TalonFX backRightDriveMotor = new TalonFX(
      PARAMETERS.getValue().getMotorId(SwerveMotors.BackRightDrive));
  private final CANSparkMax backRightSteeringMotor = new CANSparkMax(
      PARAMETERS.getValue().getMotorId(SwerveMotors.BackRightSteering), MotorType.kBrushless);

  // 4 CANcoders for the steering angle.
  private final CANcoder frontLeftAngle = new CANcoder(
      PARAMETERS.getValue().getAngleEncoderId(SwerveAngleEncoder.FrontLeft));
  private final CANcoder frontRightAngle = new CANcoder(
      PARAMETERS.getValue().getAngleEncoderId(SwerveAngleEncoder.FrontRight));
  private final CANcoder backLeftAngle = new CANcoder(
      PARAMETERS.getValue().getAngleEncoderId(SwerveAngleEncoder.BackLeft));
  private final CANcoder backRightAngle = new CANcoder(
      PARAMETERS.getValue().getAngleEncoderId(SwerveAngleEncoder.BackRight));

  private final SwerveModule frontLeftModule = createSwerveModule(
      frontLeftDriveMotor, frontLeftSteeringMotor, frontLeftAngle,
      PARAMETERS.getValue().getAngleOffset(SwerveAngleEncoder.FrontLeft), "Front Left");
  private final SwerveModule frontRightModule = createSwerveModule(
      frontRightDriveMotor, frontRightSteeringMotor, frontRightAngle,
      PARAMETERS.getValue().getAngleOffset(SwerveAngleEncoder.FrontRight), "Front Right");
  private final SwerveModule backLeftModule = createSwerveModule(
      backLeftDriveMotor, backLeftSteeringMotor, backLeftAngle,
      PARAMETERS.getValue().getAngleOffset(SwerveAngleEncoder.BackLeft), "Back Left");
  private final SwerveModule backRightModule = createSwerveModule(
      backRightDriveMotor, backRightSteeringMotor, backRightAngle,
      PARAMETERS.getValue().getAngleOffset(SwerveAngleEncoder.BackRight), "Back Right");

  private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP, NAVX_UPDATE_FREQUENCY_HZ);

  private final SwerveDriveKinematics kinematics = PARAMETERS.getValue().getKinematics();

  private final SwerveDrive drivetrain;
  private final SwerveDrivePoseEstimator odometry;
  private final Field2d field = new Field2d();

  // The current sensor state updated by the periodic method.
  private Rotation2d rawOrientation;
  private Rotation2d rawOrientationOffset = new Rotation2d();
  private Rotation2d orientation = new Rotation2d();
  private Pose2d lastVisionMeasurement = new Pose2d();

  private DoubleLogEntry rawOrientationLog = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/rawOrientation");
  private DoubleLogEntry rawOrientationOffsetLog = new DoubleLogEntry(DataLogManager.getLog(),
      "/SwerveSubsystem/rawOrientationOffset");
  private DoubleLogEntry poseXLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pose X");
  private DoubleLogEntry poseYLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pose Y");
  private DoubleLogEntry poseAngleLog = new DoubleLogEntry(DataLogManager.getLog(), "/SwerveSubsystem/Pose Angle");

  // Simulation support.
  private final boolean isSimulation;
  private Rotation2d simOrientation = new Rotation2d();

  /**
   * Creates a {@link SwerveModule} object and intiailizes its motor controllers.
   * 
   * @param driveMotor    The drive motor controller.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle    An absolute encoder that measures the wheel angle.
   * @param name          The name of the module.
   * 
   * @return An initialized {@link SwerveModule} object.
   */
  private static SwerveModule createSwerveModule(
      TalonFX driveMotor,
      CANSparkMax steeringMotor,
      CANcoder wheelAngle,
      double angleOffset,
      String name) {

    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    steeringMotor.setIdleMode(IdleMode.kBrake);
    steeringMotor.setInverted(PARAMETERS.getValue().isSteeringInverted());

    CANcoderConfigurator wheelAngleConfigurator = wheelAngle.getConfigurator();
    CANcoderConfiguration wheelAngleConfig = new CANcoderConfiguration();

    wheelAngleConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    wheelAngleConfig.MagnetSensor.MagnetOffset = angleOffset / 360.0;
    wheelAngleConfigurator.apply(wheelAngleConfig);

    final double metersPerRotation = (PARAMETERS.getValue().getWheelDiameter() * Math.PI)
        / PARAMETERS.getValue().getDriveGearRatio();

    StatusSignal<Double> drivePosition = driveMotor.getPosition();
    StatusSignal<Double> driveVelocity = driveMotor.getVelocity();
    StatusSignal<Double> wheelOrientation = wheelAngle.getAbsolutePosition();
    StatusSignal<Double> angularVelocity = wheelAngle.getVelocity();

    return new SwerveModule(
        PARAMETERS.getValue(),
        driveMotor,
        () -> drivePosition.refresh().getValueAsDouble() * metersPerRotation,
        () -> driveVelocity.refresh().getValueAsDouble() * metersPerRotation,
        steeringMotor,
        () -> Rotation2d.fromDegrees(wheelOrientation.refresh().getValueAsDouble() * 360.0),
        () -> Math.toRadians(angularVelocity.refresh().getValueAsDouble()),
        name);
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    isSimulation = Robot.isSimulation();

    initializeSensorState();

    drivetrain = new SwerveDrive(PARAMETERS.getValue(), modules, () -> getOrientation());
    odometry = new SwerveDrivePoseEstimator(
        kinematics, getOrientation(), drivetrain.getModulesPositions(), new Pose2d());
  }

  /**
   * Initializes the sensor state.
   */
  private void initializeSensorState() {
    ahrs.reset();
    updateSensorState();
  }

  /**
   * Updates the sensor state.
   * <p>
   * This method **MUST* be called by the {@link #periodic()} method to ensure the
   * sensor state is up to date.
   */
  private void updateSensorState() {
    rawOrientation = !isSimulation ? Rotation2d.fromDegrees(-ahrs.getAngle()) : simOrientation;
    rawOrientationLog.append(rawOrientation.getDegrees());
    orientation = rawOrientation.plus(rawOrientationOffset);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
    odometry.addVisionMeasurement(visionMeasurement, timestamp);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)} */
  public void addVisionMeasurement(Pose2d visionMeasurment, double timestamp, Matrix<N3, N1> stdDevs){
    odometry.addVisionMeasurement(visionMeasurment, timestamp, stdDevs);
    lastVisionMeasurement = visionMeasurment;
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   * 
   * @return The maximum drive speed.
   */
  public static double getMaxSpeed() {
    return PARAMETERS.getValue().getMaxDriveSpeed();
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   * 
   * @return The maximum drive acceleration.
   */
  public static double getMaxAcceleration() {
    return PARAMETERS.getValue().getMaxDriveAcceleration();
  }

  /**
   * Returns the swerve drive kinematics for this subsystem.
   * 
   * @return The swerve drive kinematics.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce
   * swerve drive kinematics constraints when following a trajectory.
   * 
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce
   *         swerve drive kinematics constraints when following a trajectory.
   */
  public static SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return PARAMETERS.getValue().getKinematicsConstraint();
  }

  /**
   * Returns the drive constraints.
   * 
   * @return The drive constraints.
   */
  public TrapezoidProfile.Constraints getDriveConstraints() {
    return new TrapezoidProfile.Constraints(getMaxSpeed(), getMaxAcceleration());
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce
   * velocity and acceleration constraints on the {@link ProfiledPIDController}
   * used to reach the goal robot orientation.
   * 
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce
   *         velocity and acceleration constraints on the controller used to reach
   *         the goal robot orientation.
   */
  public static TrapezoidProfile.Constraints getRotationalConstraints() {
    return PARAMETERS.getValue().getRotationalConstraints();
  }

  /**
   * Return the wheel base radius in meters.
   * 
   * @return The wheel base radius in meters.
   */
  public static double getWheelBaseRadius() {
    return PARAMETERS.getValue().getWheelBaseRadius();
  }

  /**
   * Creates a HolonomicDriveController for the subsystem.
   * 
   * @return A HolonomicDriveController.
   */
  public HolonomicDriveController createDriveController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.0, 0.0, 0.0, getRotationalConstraints());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new HolonomicDriveController(
        new PIDController(DRIVE_KP, 0.0, 0.0),
        new PIDController(DRIVE_KP, 0.0, 0.0),
        thetaController);
  }

  /**
   * Drives the robot based on joystick inputs.
   * 
   * @param xSpeed        Speed of the robot in the x direction.
   * @param ySpeed        Speed of the robot in the y direction.
   * @param rSpeed        Rotation speed of the robot.
   * @param fieldRelative Whether the x and y values are relative to field.
   */
  public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
    drivetrain.drive(xSpeed, ySpeed, rSpeed, fieldRelative);
  }

  /**
   * Sets the current module's states based on the chassis speed.
   * 
   * @param speeds The chassis speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.setChassisSpeeds(speeds);
  }

  /**
   * Returns the current chassis speed.
   * 
   * @return The chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getChassisSpeeds();
  }

  public SwerveModuleState[] getModuleStates() {
    return drivetrain.getModuleStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return drivetrain.getModulesPositions();
  }

  /**
   * Returns the swerve module velocities.
   * 
   * @return The swerve module velocities.
   */
  public SwerveModuleVelocities[] getModuleVelocities() {
    return drivetrain.getModuleVelocities();
  }

  /**
   * Sets the swerve module states.
   * 
   * @param states An array of four {@link SwerveModuleState} objects in the
   *               order: front left, front right, back left, back right
   */

  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states);
  }

  /**
   * Gets the module motor voltages.
   * 
   * @return The module motor
   */
  public SwerveModuleVoltages[] getModuleVoltages() {
    return drivetrain.getModuleVoltages();
  }

  /**
   * Sets the module motor voltages.
   * 
   * @param moduleVoltages The module motor voltages.
   */
  public void setModuleVoltages(SwerveModuleVoltages[] moduleVoltages) {
    drivetrain.setModuleVoltages(moduleVoltages);
  }

  // Stops motors from the subsystem - may need to remove this (not sure - Om)
  public void stopMotors() {
    drivetrain.stopMotor();
  }

  /**
   * Resets the robots position on the field.
   * 
   * @param desiredPosition Sets the initial position.
   */
  public void resetPosition(Pose2d desiredPosition) {
    orientation = desiredPosition.getRotation();
    rawOrientationOffset = orientation.minus(rawOrientation);
    rawOrientationOffsetLog.append(rawOrientationOffset.getDegrees());

    odometry.resetPosition(getOrientation(), drivetrain.getModulesPositions(), desiredPosition);
  }

  /**
   * Resets the orientation the robot.
   */
  public void resetOrientation(Rotation2d orientation) {
    Pose2d currentPos = odometry.getEstimatedPosition();
    Pose2d newPos2d = new Pose2d(currentPos.getTranslation(), orientation);
    resetPosition(newPos2d);
  }

  /**
   * Return current position & orientation of the robot on the field.
   * 
   * @return The current position and orientation of the robot.
   */
  public Pose2d getPosition() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the current position and orienation of the robot on the field in
   * 3-dimensional space.
   * 
   * @return The current position and orientation in 3-dimensional space.
   */
  public Pose3d getPosition3d() {
    Pose2d robotPose2d = getPosition();

    return new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
  }

  /**
   * Returns the field orientation of the robot as a {@link Rotation2d} object.
   * 
   * @return Gets the field orientation of the robot.
   */
  public Rotation2d getOrientation() {
    return orientation;
  }

  @Override
  public void periodic() {
    // Read sensors to update subsystem state.
    updateSensorState();

    // Update the current module state.
    drivetrain.periodic();

    // Update odometry last since this relies on the subsystem sensor and module
    // states.
    odometry.update(getOrientation(), drivetrain.getModulesPositions());

    // Send the robot and module location to the field
    Pose2d robotPose = getPosition();

    field.setRobotPose(robotPose);

    ArrayList<Pose2d> modulePoses = new ArrayList<Pose2d>(4);

    for (Translation2d wheelPosition : PARAMETERS.getValue().getWheelPositions()) {
      modulePoses.add(
          new Pose2d(
              wheelPosition.rotateBy(robotPose.getRotation())
                  .plus(robotPose.getTranslation()),
              robotPose.getRotation()));
    }

    field.getObject("Swerve Modules").setPoses(modulePoses);

    poseXLog.append(robotPose.getX());
    poseYLog.append(robotPose.getY());
    poseAngleLog.append(robotPose.getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    drivetrain.simulationPeriodic();

    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());

    simOrientation = new Rotation2d(
        simOrientation.getRadians() + (chassisSpeeds.omegaRadiansPerSecond * Robot.kDefaultPeriod));
  }

  /**
   * Adds a tab for swerve drive in Shuffleboard.
   */
  public void addShuffleboardTab() {
    if (ENABLE_DRIVE_TAB.getValue()) {
      ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Drive");

      drivetrain.addShuffleboardLayouts(swerveDriveTab);

      ShuffleboardLayout odometryLayout = swerveDriveTab.getLayout("Odometry", BuiltInLayouts.kList)
          .withPosition(6, 0)
          .withSize(4, 4);

      odometryLayout.add("Orientation", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> -getOrientation().getDegrees(), null);
        }
      }).withWidget(BuiltInWidgets.kGyro).withPosition(0, 0);

      ShuffleboardLayout positionLayout = odometryLayout.getLayout("Position", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 5, "Number of rows", 1));

      positionLayout.addDouble("X", () -> odometry.getEstimatedPosition().getX())
          .withPosition(0, 0);
      positionLayout.addDouble("Y", () -> odometry.getEstimatedPosition().getY())
          .withPosition(1, 0);

      positionLayout.addDouble("est. X", () -> lastVisionMeasurement.getX())
          .withPosition(2, 0);
      positionLayout.addDouble("est. Y", () -> lastVisionMeasurement.getY())
          .withPosition(3, 0);
      positionLayout.addDouble("est. angle", () -> lastVisionMeasurement.getRotation().getDegrees())
          .withPosition(4, 0);    
    }

    if (ENABLE_FIELD_TAB) {
      ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
      ShuffleboardLayout fieldLayout = fieldTab.getLayout("Field", BuiltInLayouts.kGrid)
          .withPosition(0, 0)
          .withSize(6, 4)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 1));

      fieldLayout.add(field);
    }
  }
}
