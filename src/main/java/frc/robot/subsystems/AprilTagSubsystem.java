/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

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
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** This subsystem is responsible for getting target information from PhotonVision. */
@RobotPreferencesLayout(
    groupName = "AprilTag",
    column = 2,
    row = 2,
    width = 2,
    height = 2,
    type = "Grid Layout",
    gridColumns = 2,
    gridRows = 2)
public class AprilTagSubsystem extends PhotonVisionSubsystemBase {
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  public static Pose3d NO_APRILTAG = new Pose3d();
  public static EstimatedRobotPose NO_APRILTAG_ESTIMATE =
      new EstimatedRobotPose(NO_APRILTAG, 0, List.of(), PoseStrategy.LOWEST_AMBIGUITY);

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLED =
      new RobotPreferences.BooleanValue("AprilTag", "Enabled", false);

  @RobotPreferencesValue(column = 1, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Tab", false);

  private final PhotonPoseEstimator estimator;
  private double lastEstTimestamp = 0;
  private final SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<Integer>();
  private final AprilTagFieldLayout aprilTagLayout;
  private int selectedAprilTag;
  private Pose3d selectedAprilTagPose = new Pose3d();
  private Pose3d globalEstimatedPose = new Pose3d();

  /** Creates a new PhotonVisionSubsystem. */
  public AprilTagSubsystem() {
    super("948Mono001", RobotConstants.APRILTAG_ROBOT_TO_CAMERA);
    for (int i = 1; i <= 16; i++) {
      aprilTagIdChooser.addOption(String.valueOf(i), i);
    }
    aprilTagIdChooser.setDefaultOption("1", 1);
    aprilTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    estimator =
        new PhotonPoseEstimator(
            aprilTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            getRobotToCameraTransform());
  }

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimateGlobalPose() {
    var visionEst = estimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }
    globalEstimatedPose = visionEst.orElse(NO_APRILTAG_ESTIMATE).estimatedPose;
    return visionEst;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimateGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
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
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
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
    return calculateAngleToTarget(target.get());
  }

  /**
   * Returns the ID of the center speaker APrilTag based on the alliance.
   *
   * @return The center speaker AprilTag ID.
   */
  public static int getSpeakerCenterAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 4 : 7;
  }

  /**
   * Returns the speaker center AprilTag.
   *
   * @return A non empty Optional value if we detect specified AprilTag. Otherwise returns
   *     Optional.empty.
   */
  public Optional<PhotonTrackedTarget> getSpeakerCenterAprilTag() {
    return getTarget(getSpeakerCenterAprilTagID());
  }

  /**
   * Returns the pose of the center speaker AprilTag based on alliance.
   *
   * @return The center speaker AprilTag pose.
   */
  public Pose3d getSpeakerCenterAprilTagPose() {
    return getAprilTagPose(getSpeakerCenterAprilTagID());
  }

  /**
   * Returns the ID of the AMP AprilTag based on alliance.
   *
   * @return The AMP AprilTag ID.
   */
  public static int getAmpAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 5 : 6;
  }

  /**
   * Return the pose of the specified AprilTag.
   *
   * @param id The ID of the AprilTag.
   * @return The pose of the AprilTag.
   */
  public Pose3d getAprilTagPose(int id) {
    return aprilTagLayout.getTagPose(id).orElse(NO_APRILTAG);
  }

  @Override
  public void periodic() {
    super.periodic();

    estimator.update(getLatestResult());

    if (ENABLE_TAB.getValue()) {
      selectedAprilTag = aprilTagIdChooser.getSelected().intValue();
      selectedAprilTagPose = getAprilTagPose(selectedAprilTag);
    }
  }

  /** Adds a tab for April Tag in Shuffleboard. */
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab visionTab = Shuffleboard.getTab("April Tag");
    ShuffleboardLayout targetLayout =
        visionTab.getLayout("Target Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
    targetLayout.add("Id Selection", aprilTagIdChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", () -> getDistanceToTarget(selectedAprilTag));
    targetLayout.addDouble("Angle", () -> getAngleToTarget(selectedAprilTag));

    VideoSource video =
        new HttpCamera(
            "photonvision_Port_1182_Output_MJPEG_Server",
            "http://photonvision.local:1182/?action=stream",
            HttpCameraKind.kMJPGStreamer);
    visionTab
        .add("April Tag", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);

    ShuffleboardLayout apriltagLayout =
        visionTab
            .getLayout("Target Position", BuiltInLayouts.kList)
            .withPosition(6, 0)
            .withSize(2, 4);

    ShuffleboardLayout selectedLayout =
        apriltagLayout
            .getLayout("Selected AprilTag", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 3, "Number of rows", 2));
    selectedLayout.addDouble("X", () -> selectedAprilTagPose.getX()).withPosition(0, 0);
    selectedLayout.addDouble("Y", () -> selectedAprilTagPose.getY()).withPosition(1, 0);
    selectedLayout.addDouble("Z", () -> selectedAprilTagPose.getZ()).withPosition(2, 0);
    selectedLayout
        .addDouble("Roll", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getX()))
        .withPosition(0, 1);
    selectedLayout
        .addDouble("Pitch", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getY()))
        .withPosition(1, 1);
    selectedLayout
        .addDouble("Yaw", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getZ()))
        .withPosition(2, 1);

    ShuffleboardLayout estimatedLayout =
        apriltagLayout
            .getLayout("Global Estimated Pose", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 3, "Number of rows", 2));
    estimatedLayout.addDouble("X", () -> globalEstimatedPose.getX()).withPosition(0, 0);
    estimatedLayout.addDouble("Y", () -> globalEstimatedPose.getY()).withPosition(1, 0);
    estimatedLayout.addDouble("Z", () -> globalEstimatedPose.getZ()).withPosition(2, 0);
    estimatedLayout
        .addDouble("Roll", () -> Math.toDegrees(globalEstimatedPose.getRotation().getX()))
        .withPosition(0, 1);
    estimatedLayout
        .addDouble("Pitch", () -> Math.toDegrees(globalEstimatedPose.getRotation().getY()))
        .withPosition(1, 1);
    estimatedLayout
        .addDouble("Yaw", () -> Math.toDegrees(globalEstimatedPose.getRotation().getZ()))
        .withPosition(2, 1);
  }
}
