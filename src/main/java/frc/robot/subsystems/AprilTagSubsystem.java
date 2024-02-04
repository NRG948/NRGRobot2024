// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.RobotConstants;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */

@RobotPreferencesLayout(groupName = "AprilTag", row = 0, column = 4, width = 2, height = 1)
public class AprilTagSubsystem extends PhotonVisionSubsystemBase {
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "AprilTag", "Enable Tab", false);

  private final PhotonPoseEstimator estimator;
  private double lastEstTimestamp = 0;
  private final SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<Integer>();
  private final AprilTagFieldLayout aprilTagLayout;

  /** Creates a new PhotonVisionSubsystem. */
  public AprilTagSubsystem() {
    super("948Mono001", RobotConstants.APRILTAG_CAMERA_TO_ROBOT);
    for (int i = 1; i <= 16; i++) {
      aprilTagIdChooser.addOption(String.valueOf(i), i);
    }
    aprilTagIdChooser.setDefaultOption("1", 1);
    aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    estimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
        getRobotToCameraTransform());
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be
   * empty. This should
   * only be called once per loop.
   * 
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
   *         timestamp, and targets
   *         used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimateGlobalPose() {
    var visionEst = estimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from
   * {@link #getEstimateGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   * 
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = SINGLE_TAG_STD_DEVS;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = MULTI_TAG_STD_DEVS;
    }
    if (numTags == 1 && avgDist > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    }

    return estStdDevs;
  }

  /**
   * Returns the AprilTag target of the input ID.
   * 
   * @param id The AprilTag ID.
   * @return The target with the input ID.
   */
  public Optional<PhotonTrackedTarget> getTarget(int id) {
    return getTargets().stream().filter(target -> target.getFiducialId() == id).findFirst();
  }

  /**
   * Returns the transform from the camera to the AprilTag with input ID.
   * 
   * @param id The AprilTag ID.
   * @return The transform from the camera to the AprilTag with input ID.
   */
  public Transform3d getCameraToTarget(int id) {
    return getTarget(id).get().getBestCameraToTarget();
  }

  /**
   * Returns the distance to the target with the input ID. Returns 0 if target not found.
   * 
   * @param id The AprilTag ID.
   * @return The distance to the target with the input ID.
   */
  public double getDistanceToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    var bestCameraToTarget = target.get().getBestCameraToTarget();
    return Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
  }

  /**
   * Returns the angle to the target with the input ID. Returns 0 if target not found.
   * 
   * @param id The AprilTag ID.
   * @return The angle to the target with the input ID.
   */
  public double getAngleToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    return target.get().getYaw();
  }

  public static int getSpeakerCenterAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 4 : 7;
  }

  public static int getAmpAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 5 : 6;
  }

  @Override
  public void periodic() {
    super.periodic();
    estimator.update(getLatestResult());
  }

  /**
   * Adds a tab for April Tag in Shuffleboard.
   */
  public void addShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab visionTab = Shuffleboard.getTab("April Tag");
    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 5);
    targetLayout.add("Id Selection", aprilTagIdChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", () -> getDistanceToTarget(aprilTagIdChooser.getSelected()));
    targetLayout.addDouble("Angle", () -> getAngleToTarget(aprilTagIdChooser.getSelected()));

    VideoSource video = new HttpCamera("photonvision_Port_1184_Output_MJPEG_Server",
        "http://photonvision.local:1184/?action=stream",
        HttpCameraKind.kMJPGStreamer);
    visionTab.add("April Tag", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }
}