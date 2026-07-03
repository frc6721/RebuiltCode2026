package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** IO implementation for real Photon hardware. */
public class VisionIOPhoton implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public VisionIOPhoton(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.poseEstimator = new PhotonPoseEstimator(aprilTagLayout, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
      }

      if (!result.hasTargets()) {
        continue;
      }

      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
      boolean usedMultiTag = estimatedPose.isPresent();
      if (estimatedPose.isEmpty()) {
        estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      Logger.recordOutput("Vision/" + camera.getName() + "/usedMultiTag", usedMultiTag);

      if (estimatedPose.isEmpty()) {
        continue;
      }

      EstimatedRobotPose estimate = estimatedPose.get();

      double totalTagDistance = 0.0;
      for (var target : estimate.targetsUsed) {
        tagIds.add((short) target.fiducialId);
        totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
      }

      double avgTagDistance =
          estimate.targetsUsed.isEmpty() ? 0.0 : totalTagDistance / estimate.targetsUsed.size();

      double ambiguity =
          estimate.targetsUsed.size() > 1
              ? 0.0
              : (estimate.targetsUsed.isEmpty() ? 1.0 : estimate.targetsUsed.get(0).poseAmbiguity);
      Logger.recordOutput("Vision/" + camera.getName() + "/Ambiguity", ambiguity);

      poseObservations.add(
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.estimatedPose,
              ambiguity,
              estimate.targetsUsed.size(),
              avgTagDistance,
              PoseObservationType.PHOTONVISION));
    }

    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
