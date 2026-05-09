// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static String frontCameraName = "front";
  public static String backCameraName = "back";
  public static String rightCameraName = "right";
  public static String leftCameraName = "left";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d frontCameraTransform =
      new Transform3d(
          Meters.of(0.748 - 0.413),
          Meters.of(1.897 - 1.7526),
          Meters.of(0.261),
          new Rotation3d(0.0, -18.0 * Math.PI / 180, 0.0));
  public static Transform3d backCameraTransform = // tuned using cad
      new Transform3d(
          Inches.of(-((9.699 + 11.128) / 2)),
          Inches.of(-((6.818 + 8.191) / 2)),
          Inches.of(19),
          new Rotation3d(0.0, -18.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Math.PI)));
  public static Transform3d rightCameraTransform = // tuned
      new Transform3d(
          //   Inches.of(16.5 - 47),
          Meters.of(0.04), // y
          Meters.of(0.419 - 0.77), // x
          Inches.of(21),
          new Rotation3d(0.0, -30.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, -Math.PI / 2.0)));
  public static Transform3d leftCameraTransform = // tuned using cad
      new Transform3d(
          Inches.of(-((1.685 + 2.361) / 2)),
          Inches.of((13.114 + 14.611) / 2),
          Inches.of(18.8),
          new Rotation3d(0.0, -30.0 * Math.PI / 180, 0.0)
              .rotateBy(new Rotation3d(0.0, 0.0, Math.PI / 2.0)));

  // Basic filtering threshold
  public static double maxAmbiguity = 0.1;
  public static double maxZError = 99999999;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  // higher value = trust it less
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera front
        1.0, // Camera back
        1.0, // Camera left
        1.0 // Camera right
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
