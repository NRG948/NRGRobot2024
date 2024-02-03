// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is a base class for subsystems responsible for getting target
 * information from PhotonVision.
 */
public abstract class PhotonVisionSubsystemBase extends SubsystemBase {

  protected final PhotonCamera camera;
  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult result = new PhotonPipelineResult();

  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;

  /**
   * Creates a new PhotonVisionSubsystemBase.
   * 
   * @param cameraName    The name of the PhotonVision camera.
   * @param cameraToRobot The transform from the camera to center of the robot.
   */
  public PhotonVisionSubsystemBase(String cameraName, Transform3d cameraToRobot) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraToRobot = cameraToRobot;
    this.robotToCamera = cameraToRobot.inverse();

    hasTargetLogger = new BooleanLogEntry(DataLogManager.getLog(), String.format("/%s/Has Target", cameraName));
    distanceLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Distance", cameraName));
    angleLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Angle", cameraName));
  }

  @Override
  public void periodic() {
    PhotonPipelineResult currentResult = camera.getLatestResult();

    if (this.result.hasTargets() != currentResult.hasTargets()) {
      hasTargetLogger.append(currentResult.hasTargets());
    }

    this.result = currentResult;

    if (hasTargets()) {
      distanceLogger.append(getDistanceToBestTarget());
      angleLogger.append(getAngleToBestTarget());
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
   * Returns information on the best target.
   * Check hasTargets() before using this function.
   * 
   * @return Information on the best target.
   */
  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  /**
   * Returns the distance to the best target.
   * 
   * @return The distance, in meters, to the best target.
   */
  public double getDistanceToBestTarget() {
    if (!hasTargets()) {
      return 0;
    }
    Transform3d bestTarget = getBestTarget().getBestCameraToTarget();
    return Math.hypot(bestTarget.getX(), bestTarget.getY());
  }

  /**
   * Returns the angle to the best target.
   * 
   * @return The angle to the best target.
   */
  public double getAngleToBestTarget() {
    if (!hasTargets()) {
      return 0;
    }

    return getBestTarget().getYaw();
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