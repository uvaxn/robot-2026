// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new PhotonVision. */

  private CommandSwerveDrivetrain swerveDrive;
  public StructPublisher<Pose3d> EstimatedPosition;
  public final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;

  private final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public CameraSubsystem(CommandSwerveDrivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;
    camera = new PhotonCamera("Limelight4");
    photonEstimator = new PhotonPoseEstimator(aprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, Constants.kcamToRobot);
    EstimatedPosition = NetworkTableInstance.getDefault().getStructTopic("EstimatedPose", Pose3d.struct).publish();
  }

  public Optional<Pose2d> getTagPose2d(int id) {
      var tag = aprilTagLayout.getTagPose(id);
      if (tag.isPresent()) {
          return Optional.of(tag.get().toPose2d());
      }
      return Optional.empty();
  }

  public List<PhotonPipelineResult> latestResults() { 
    var results = camera.getAllUnreadResults();
    return results;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    var cameraResults = camera.getAllUnreadResults();
    for (PhotonPipelineResult cameraResult : cameraResults) {
        visionEstimate = photonEstimator.update(cameraResult);
        if (visionEstimate.isPresent()) {
            EstimatedPosition.set(visionEstimate.get().estimatedPose);
        }
    }

    return visionEstimate;
}


  @Override
  public void periodic() {
      var results = camera.getAllUnreadResults();
      
      Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
      for (PhotonPipelineResult result : results) {
          visionEstimate = photonEstimator.update(result);
      }

      if (visionEstimate.isPresent()) {
          swerveDrive.addVisionMeasurement(
              visionEstimate.get().estimatedPose.toPose2d(),
              visionEstimate.get().timestampSeconds
          );
          EstimatedPosition.set(visionEstimate.get().estimatedPose);
      } else {
        System.out.println("no april tag!");
      }
  }
}
