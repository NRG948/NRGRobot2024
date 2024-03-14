/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GeometryUtils;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * This is a base class for subsystems responsible for getting target information from PhotonVision.
 */
public abstract class PhotonVisionSubsystemBase extends SubsystemBase {

  protected final PhotonCamera camera;
  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult result = new PhotonPipelineResult();

  private double angleToTarget;
  private double distanceToTarget;
  private double poseAmbiguity;

  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;

  /**
   * Creates a new PhotonVisionSubsystemBase.
   *
   * @param cameraName The name of the PhotonVision camera.
   * @param robotToCamera The transform from the center of the robot to the camera.
   */
  public PhotonVisionSubsystemBase(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraToRobot = robotToCamera.inverse();
    this.robotToCamera = robotToCamera;

    System.out.println(
        cameraName + " robot to camera: " + GeometryUtils.transformToString(robotToCamera));
    System.out.println(
        cameraName + " camera to robot: " + GeometryUtils.transformToString(cameraToRobot));

    hasTargetLogger =
        new BooleanLogEntry(DataLogManager.getLog(), String.format("/%s/Has Target", cameraName));
    distanceLogger =
        new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Distance", cameraName));
    angleLogger =
        new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Angle", cameraName));
  }

  @Override
  public void periodic() {
    PhotonPipelineResult currentResult = camera.getLatestResult();

    if (this.result.hasTargets() != currentResult.hasTargets()) {
      hasTargetLogger.append(currentResult.hasTargets());
    }

    this.result = currentResult;

    if (hasTargets()) {
      PhotonTrackedTarget bestTarget = getBestTarget();
      angleToTarget = -bestTarget.getYaw();
      Transform3d bestTargetTransform = bestTarget.getBestCameraToTarget();
      distanceToTarget = Math.hypot(bestTargetTransform.getX(), bestTargetTransform.getY());
      poseAmbiguity = bestTarget.getPoseAmbiguity();

      distanceLogger.append(distanceToTarget);
      angleLogger.append(angleToTarget);
    }
  }

  /**
   * Returns the latest vision result.
   *
   * @return The latest vision result.
   */
  protected PhotonPipelineResult getLatestResult() {
    return result;
  }

  /**
   * Returns the angle to the specified target in radians
   *
   * @param target The target.
   * @return The angle to target in radians.
   */
  public static double calculateAngleToTarget(PhotonTrackedTarget target) {
    return Math.toRadians(-target.getYaw());
  }

  /**
   * Returns the transform from the camera to center of the robot.
   *
   * @return The transform from the camera to center of the robot.
   */
  public Transform3d getCameraToRobotTransform() {
    return cameraToRobot;
  }

  /**
   * Returns the transform from the center of the robot to the camera.
   *
   * @return The transform from the center of the robot to the camera.
   */
  public Transform3d getRobotToCameraTransform() {
    return robotToCamera;
  }

  /**
   * Returns whether the result contains any targets.
   *
   * @return Returns true if there are targets.
   */
  public boolean hasTargets() {
    return result.hasTargets();
  }

  /**
   * Returns information on the best target. Check hasTargets() before using this function.
   *
   * @return Information on the best target.
   */
  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  /**
   * Returns the distance to the best target. Check hasTargets() to make sure you're getting a valid
   * value.
   *
   * @return The distance, in meters, to the best target.
   */
  public double getDistanceToBestTarget() {
    return distanceToTarget;
  }

  /**
   * Returns the angle to the best target. Check hasTargets() to make sure you're getting a valid
   * value.
   *
   * @return The angle to the best target.
   */
  public double getAngleToBestTarget() {
    return angleToTarget;
  }

  /**
   * Returns the pose ambiguity of the best target. Check hasTargets() to make sure you're getting a
   * valid value.
   *
   * @return The pose ambiguity of the best target.
   */
  public double getPoseAmbiguity() {
    return poseAmbiguity;
  }

  /**
   * Returns the estimated time, in seconds, the target was detected.
   *
   * @return The timestamp in seconds or -1 if no target was detected.
   */
  public double getTargetTimestamp() {
    return result.getTimestampSeconds();
  }

  /**
   * Returns a list of visible targets.
   *
   * @return A list of visible targets.
   */
  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }
}
