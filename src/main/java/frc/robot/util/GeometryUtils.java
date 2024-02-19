/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class GeometryUtils {
  public static String transformToString(Transform3d transform) {
    Rotation3d rotation = transform.getRotation();

    return String.format(
        "Transform3d(Translation3d(x: %.2f, y: %.2f, z: %.2f), "
            + "Rotation3d(roll: %.2f, pitch: %.2f, yaw: %.2f))",
        transform.getX(),
        transform.getY(),
        transform.getZ(),
        Math.toDegrees(rotation.getX()),
        Math.toDegrees(rotation.getY()),
        Math.toDegrees(rotation.getZ()));
  }
}
