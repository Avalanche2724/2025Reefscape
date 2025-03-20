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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
  // TODO: should be andymark; appears to have bug in sim (?)
  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;

  static {
    var layout1 = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var list = layout1.getTags();
    list.removeIf(
        tag -> {
          int tagId = tag.ID;
          return !((tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22));
        });
    APRILTAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(list, layout1.getFieldLength(), layout1.getFieldWidth());
  }

  public VisionSystemSim visionSim = Robot.isSimulation() ? new VisionSystemSim("main") : null;

  public Camera cameraFrElev =
      new Camera(
          "Arducam_fr_elev",
          new Transform3d(
              new Translation3d(Inches.of(14 - 6.4), Inches.of(-14 + 3.5), Inches.of(11)),
              new Rotation3d(Degrees.of(0), Degrees.of(-15), Degrees.of(12))));

  public Camera cameraFlSwerve =
      new Camera(
          "Arducam_fl_swerve",
          new Transform3d(
              new Translation3d(Inches.of(14 - 1.55), Inches.of(14 - 5.25), Inches.of(8.7)),
              new Rotation3d(Degrees.of(0), Degrees.of(-10), Degrees.of(-15))));

  {
    if (Robot.isSimulation()) {
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(APRILTAG_FIELD_LAYOUT);
    }
  }

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  // ----- Simulation

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }

  @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
  public class Camera {
    // Standard deviations for vision estimations:
    private static final Matrix<N3, N1> singleTagDevs = VecBuilder.fill(5, 5, 10);
    private static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(0.8, 0.8, 1.6);
    // PNP params
    private static final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> pnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1e12));
    // Publishers for position telemetry
    private final StructPublisher<Pose2d> constrainedPoseTelemetry;
    private final StructPublisher<Pose2d> pnpPoseTelemetry;
    // PhotonVision stuff
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    // Other things:
    private final Transform3d robotToCam;
    private final String cameraName;
    private final String visionEstimationKey;
    private Optional<Matrix<N3, N3>> cameraMatrix;
    private Optional<Matrix<N8, N1>> cameraDistortion;

    public Camera(String cameraName, Transform3d robotToCam) {
      this.cameraName = cameraName;
      this.visionEstimationKey = "Vision Estimation " + cameraName;
      this.robotToCam = robotToCam;
      camera = new PhotonCamera(cameraName);
      cameraMatrix = camera.getCameraMatrix();
      cameraDistortion = camera.getDistCoeffs();

      var smartDashboardTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");

      constrainedPoseTelemetry =
          smartDashboardTable.getStructTopic("CON" + visionEstimationKey, Pose2d.struct).publish();
      pnpPoseTelemetry =
          smartDashboardTable.getStructTopic("PNP" + visionEstimationKey, Pose2d.struct).publish();

      photonEstimator =
          new PhotonPoseEstimator(
              APRILTAG_FIELD_LAYOUT, PoseStrategy.CONSTRAINED_SOLVEPNP, this.robotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

      if (Robot.isSimulation()) {
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(79));
        cameraProp.setCalibError(0.4, 0.3);
        cameraProp.setFPS(20);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(4);

        var cameraSim = new PhotonCameraSim(camera, cameraProp);
        visionSim.addCamera(cameraSim, this.robotToCam);
        cameraSim.enableDrawWireframe(true);
      }
    }

    // TODO: should we return multiple poses?
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

        pnpPoseTelemetry.set(solvePnpEstimate.estimatedPose.toPose2d());

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

        var estStdDevs = numTags == 1 ? singleTagDevs : multiTagDevs;
        // Just taken from the photonvision example
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

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
          if (DriverStation.isDisabled()) {
            photonEstimator.addHeadingData(
                change.getTimestampSeconds(),
                solvePnpEstimate.estimatedPose.toPose2d().getRotation());
          }

          photonEstimator.setPrimaryStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP);
          var constrainedEst =
              photonEstimator.update(change, cameraMatrix, cameraDistortion, pnpParams);
          if (constrainedEst.isPresent()) {
            var constrainedEstimate = constrainedEst.get();

            constrainedPoseTelemetry.set(constrainedEstimate.estimatedPose.toPose2d());

            estimateConsumer.accept(constrainedEstimate, estStdDevs);
          } else {
            // TODO Idk why this happens debug later
            // For now just use the solvepnp estimate
            System.out.println("Warning: constrained pnp failed?");
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
