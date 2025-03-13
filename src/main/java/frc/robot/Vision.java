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
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  // TODO: should be andymark; appears to have bug in sim (?)
  public static final AprilTagFieldLayout kTagLayout;

  static {
    var layout1 = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var list = layout1.getTags();
    list.removeIf(
        tag -> {
          int tagId = tag.ID;
          return !((tagId >= 6 && tagId <= 11) || (tagId >= 17 && tagId <= 22));
        });
    kTagLayout = new AprilTagFieldLayout(list, layout1.getFieldLength(), layout1.getFieldWidth());
  }

  public VisionSystemSim visionSim = Robot.isSimulation() ? new VisionSystemSim("main") : null;

  public Camera camera1 =
      new Camera(
          "Arducam_fr_elev",
          new Transform3d(
              new Translation3d(Inches.of(7.5), Inches.of(-10.5), Inches.of(11.1)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-15.2), Degrees.of(15))));

  public Camera camera2 =
      new Camera(
          "Arducam_fl_swerve",
          new Transform3d(
              new Translation3d(Inches.of(12.6), Inches.of(8.75), Inches.of(9.0)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-10), Degrees.of(-15))));

  {
    if (Robot.isSimulation()) {
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);
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

    // TODO Experiment and determine estimation noise on an actual robot.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    private final Transform3d robotToCam;
    private final PhotonCamera camera;
    private final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> lockedPnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1e8));
    private final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> unlockedPnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 100));
    private final PhotonPoseEstimator photonEstimator;
    private String cameraName;
    private String visionEstimationKey;
    private Optional<Matrix<N3, N3>> cameraMatrix;
    private Optional<Matrix<N8, N1>> cameraDistortion;
    private Matrix<N3, N1> curStdDevs;
    public final StructPublisher<Pose2d> drivePoseTelemetry;

    public Camera(String cameraName, Transform3d robotToCam) {
      this.cameraName = cameraName;
      this.visionEstimationKey = "Vision Estimation " + cameraName;
      this.robotToCam = robotToCam;
      camera = new PhotonCamera(cameraName);
      cameraMatrix = camera.getCameraMatrix();
      cameraDistortion = camera.getDistCoeffs();

      drivePoseTelemetry =
          NetworkTableInstance.getDefault()
              .getTable("SmartDashboard")
              .getStructTopic(visionEstimationKey, Pose2d.struct)
              .publish();

      photonEstimator =
          new PhotonPoseEstimator(kTagLayout, PoseStrategy.CONSTRAINED_SOLVEPNP, this.robotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

      if (Robot.isSimulation()) {
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(79));
        cameraProp.setCalibError(0.3, 0.1);
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
        BiConsumer<Optional<EstimatedRobotPose>, Matrix<N3, N1>> estimateConsumer) {
      if (camera.isConnected()) {
        if (cameraMatrix.isEmpty()) {
          cameraMatrix = camera.getCameraMatrix();
        }
        if (cameraDistortion.isEmpty()) {
          cameraDistortion = camera.getDistCoeffs();
        }
      }

      var unreadResults = camera.getAllUnreadResults();

      if (DriverStation.isEnabled()) {
        photonEstimator.addHeadingData(headingFpgaTimestamp, heading);
      }

      for (var change : unreadResults) {
        photonEstimator.setPrimaryStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        var coprocPnpEst = photonEstimator.update(change);
        photonEstimator.setPrimaryStrategy(PoseStrategy.CONSTRAINED_SOLVEPNP);
        if (coprocPnpEst.isEmpty()) {
          System.out.print("Warning: coproc pnp est empty");
          continue;
        }

        if (DriverStation.isDisabled()) {
          var coprocRotation = coprocPnpEst.get().estimatedPose.getRotation().toRotation2d();
          // Map to nearest 90 deg angle
          var newRotation =
              Rotation2d.fromDegrees(Math.round(coprocRotation.getDegrees() / 90) * 90);
          // If the rotation is near a 90 degree angle, use that angle
          if (Math.abs(coprocRotation.getDegrees() - newRotation.getDegrees()) < 6) {
            photonEstimator.addHeadingData(coprocPnpEst.get().timestampSeconds, newRotation);
          } // Else add the entire rotation
          else {
            photonEstimator.addHeadingData(coprocPnpEst.get().timestampSeconds, coprocRotation);
          }
        }

        var visionEst =
            photonEstimator.update(
                change,
                cameraMatrix,
                cameraDistortion,
                DriverStation.isDisabled() ? unlockedPnpParams : lockedPnpParams);
        updateEstimationStdDevs(visionEst, change.getTargets());
        drivePoseTelemetry.set(visionEst.map(est -> est.estimatedPose.toPose2d()).orElse(null));
        estimateConsumer.accept(visionEst, curStdDevs);

        if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
              est ->
                  getSimDebugField()
                      .getObject(visionEstimationKey)
                      .setPose(est.estimatedPose.toPose2d()),
              () -> getSimDebugField().getObject(visionEstimationKey).setPoses());
        }
      }
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        // No pose input. Default to single-tag std devs
        curStdDevs = kSingleTagStdDevs;
      } else {
        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
          var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty()) continue;
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
          // No tags visible. Default to single-tag std devs
          curStdDevs = kSingleTagStdDevs;
        } else {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1) estStdDevs = kMultiTagStdDevs;
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDevs = estStdDevs;
        }
      }
    }
  }
}
