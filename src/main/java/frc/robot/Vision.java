/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class Vision {
  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;

  static {
    var originalLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var list = originalLayout.getTags();
    list.removeIf(
        tag -> {
          int tagId = tag.ID;
          return !((tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22));
        });
    APRILTAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(
            list, originalLayout.getFieldLength(), originalLayout.getFieldWidth());
  }

  public VisionSystemSim visionSim =
      RobotContainer.Robot.isSimulation() ? new VisionSystemSim("main") : null;

  public Camera cameraFlSwerve =
      new Camera(
          "Arducam_fl_swerve",
          new Transform3d(
              new Translation3d(Inches.of(14 - 1.6), Inches.of(14 - 5.3), Inches.of(8.7)),
              new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(-15))));

  public Camera cameraFrElev =
      new Camera(
          "Arducam_fr_elev",
          new Transform3d(
              // TODO this translation seems way off maybe revise later
              new Translation3d(Inches.of(14 - 1.93), Inches.of(-14 + 7.95), Inches.of(7.8)),
              new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(15))));

  {
    if (RobotContainer.Robot.isSimulation()) {
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(APRILTAG_FIELD_LAYOUT);
    }
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  // ----- Simulation

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!RobotContainer.Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
  public class Camera {
    // Standard deviations for vision estimations:
    private static final Matrix<N3, N1> singleTagDevs = VecBuilder.fill(4, 4, 1_000_000);
    private static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(0.4, 0.4, 15);
    private static final Matrix<N3, N1> multiTagDevsEnabled = VecBuilder.fill(0.4, 0.4, 1_000_000);

    // PNP params
    private static final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> pnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1e12));
    private final String visionEstimationKey;
    // PhotonVision stuff
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Optional<Matrix<N3, N3>> cameraMatrix;
    private Optional<Matrix<N8, N1>> cameraDistortion;

    public Camera(String cameraName, Transform3d robotToCam) {
      visionEstimationKey = "Vision Estimation " + cameraName;
      // Other things:
      camera = new PhotonCamera(cameraName);
      cameraMatrix = camera.getCameraMatrix();
      cameraDistortion = camera.getDistCoeffs();

      photonEstimator =
          new PhotonPoseEstimator(
              APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

      if (RobotContainer.Robot.isSimulation()) {
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(79));
        cameraProp.setCalibError(0, 0);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(4);

        var cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, robotToCam);
        cameraSim.enableDrawWireframe(true);
      }
    }

    // TODO we need alerting and stuff for when the camera is not connected
    public void getEstimatedGlobalPose(
        double headingFpgaTimestamp,
        Rotation2d heading,
        BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> estimateConsumer) {
      // Update camera matrix/distortion if necessary
      if (camera.isConnected()) {
        if (cameraMatrix.isEmpty()) {
          cameraMatrix = camera.getCameraMatrix();
        }
        if (cameraDistortion.isEmpty()) {
          cameraDistortion = camera.getDistCoeffs();
        }
      }

      var unreadResults = camera.getAllUnreadResults();

      // Add heading data from thingy
      photonEstimator.addHeadingData(headingFpgaTimestamp, heading);
      // photonEstimator.addHeadingData(Sy);

      for (var change : unreadResults) {
        // This could either be the multi-tag pnp estimate, or the single-tag trig estimate if only
        // one tag is seen
        photonEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        var solvePnpEst = photonEstimator.update(change);
        if (solvePnpEst.isEmpty()) continue; // 0 tags

        var solvePnpEstimate = solvePnpEst.get();

        Util.logPose2d("Pose" + visionEstimationKey, solvePnpEstimate.estimatedPose.toPose2d());

        // Get tags
        var targets = change.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
          var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(solvePnpEstimate.estimatedPose.toPose2d().getTranslation());
        }
        avgDist /= numTags;

        var estStdDevs =
            numTags == 1
                ? singleTagDevs
                : (DriverStation.isDisabled() ? multiTagDevs : multiTagDevsEnabled);
        // Just taken from the photonvision example
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        // TODO caused some issue; fix later

        if (numTags == 0) {
          // This should not happen:
          System.out.println("Warning: numtags = 0");
        } else if (numTags == 1) {
          // If num tags = 1, just use the trig solvepnp pose, don't update heading
          // Note: we're assuming we currently have an accurate field relative heading but whatever
          if (avgDist <= 4) {
            estimateConsumer.accept(solvePnpEstimate, estStdDevs);
          }
        } else {
          // If we have multiple tags: use constrained pnp, lock heading to original pnp heading if
          // disabled
          // if (DriverStation.isDisabled()) {
          //  photonEstimator.addHeadingData(
          //      change.getTimestampSeconds(),
          //      solvePnpEstimate.estimatedPose.toPose2d().getRotation());
          // }

          // photonEstimator.setPrimaryStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP);
          // var constrainedEst =
          //     photonEstimator.update(change, cameraMatrix, cameraDistortion, pnpParams);
          // if (constrainedEst.isPresent()) {
          if (false) {
            // var constrainedEstimate = constrainedEst.get();

            // constrainedPoseTelemetry.set(constrainedEstimate.estimatedPose.toPose2d());

            // estimateConsumer.accept(constrainedEstimate, estStdDevs);
          } else {
            // TODO Idk why this happens debug later (??)
            // For now just use the solvepnp estimate
            // System.out.println("Warning: constrained pnp failed?");
            estimateConsumer.accept(solvePnpEstimate, estStdDevs);
          }
        }
      }
    }
  }

  /*
   todo: this thing

    public static Transform2d solveRobotToCamera(
        Pose2d cameraPose1, Pose2d cameraPose2, Rotation2d angleOnRobot) {
      // Extract the camera positions and rotations
      double x1 = cameraPose1.getTranslation().getX();
      double y1 = cameraPose1.getTranslation().getY();
      double x2 = cameraPose2.getTranslation().getX();
      double y2 = cameraPose2.getTranslation().getY();

      double theta1 = cameraPose1.getRotation().getRadians();
      double theta2 = cameraPose2.getRotation().getRadians();

      // Compute the coefficients for x and y
      double cos1 = Math.cos(theta1);
      double sin1 = Math.sin(theta1);
      double cos2 = Math.cos(theta2);
      double sin2 = Math.sin(theta2);

      // Compute the determinant (denominator) for solving the system
      double denominator = (cos1 - cos2) * (cos1 - cos2) + (sin1 - sin2) * (sin1 - sin2);

      // Calculate x and y
      double x = ((x2 - x1) * (cos1 - cos2) + (y2 - y1) * (sin1 - sin2)) / -denominator;
      double y = ((x2 - x1) * (sin1 - sin2) + (y2 - y1) * (cos2 - cos1)) / denominator;

      // Return the robotToCamera transform as a Transform2d
      return new Transform2d(new Translation2d(x, y).rotateBy(angleOnRobot), angleOnRobot);
    }
  */
}
