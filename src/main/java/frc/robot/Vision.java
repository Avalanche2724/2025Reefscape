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
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
  // TODO: should be andymark; appears to have bug in sim (?)
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private VisionSystemSim visionSim;

  {
    if (Robot.isSimulation()) {
      // Create the vision system simulation which handles cameras and targets on the field.
      visionSim = new VisionSystemSim("main");
      // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
      visionSim.addAprilTags(kTagLayout);
      // Remove all apriltags NOT on the blue reef for simulation
      visionSim.removeVisionTargets(
          visionSim.getVisionTargets().stream()
              .filter(vt -> !(vt.fiducialID >= 17 && vt.fiducialID <= 22))
              .toArray(VisionTargetSim[]::new));

      System.out.println(visionSim.getVisionTargets());
    }
  }

  public Camera camera1 =
      new Camera(
          "Arducam_OV9281_USB_Camera (1)",
          new Transform3d(
              new Translation3d(Inches.of(7.5), Inches.of(-10.5), Inches.of(11.06)),
              new Rotation3d(Degrees.of(0.0), Degrees.of(-15.2), Degrees.of(12.5))));

  // public Camera camera2;

  @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
  public class Camera {
    // TODO Experiment and determine estimation noise on an actual robot.
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    private String cameraName;
    private String visionEstimationKey;
    private final Transform3d robotToCam;
    private final PhotonCamera camera;
    private Optional<Matrix<N3, N3>> cameraMatrix;
    private Optional<Matrix<N8, N1>> cameraDistortion;
    private final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> enabledPnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1e5));
    private final Optional<PhotonPoseEstimator.ConstrainedSolvepnpParams> disabledPnpParams =
        Optional.of(new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1));

    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;

    public Camera(String cameraName, Transform3d robotToCam) {
      this.cameraName = cameraName;
      this.visionEstimationKey = "Vision Estimation " + cameraName;
      this.robotToCam = robotToCam;
      camera = new PhotonCamera(cameraName);
      cameraMatrix = camera.getCameraMatrix();
      cameraDistortion = camera.getDistCoeffs();

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
      photonEstimator.addHeadingData(headingFpgaTimestamp, heading);

      for (var change : camera.getAllUnreadResults()) {
        var visionEst =
            photonEstimator.update(
                change,
                cameraMatrix,
                cameraDistortion,
                DriverStation.isDisabled() ? disabledPnpParams : enabledPnpParams);
        updateEstimationStdDevs(visionEst, change.getTargets());
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

  // ----- Simulation

  public void simulationPeriodic(Pose2d robotSimPose) {
    visionSim.update(robotSimPose);
  }

  /** Reset pose history of the robot in the vision system simulation. */
  public void resetSimPose(Pose2d pose) {
    if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
    if (!Robot.isSimulation()) return null;
    return visionSim.getDebugField();
  }
}
