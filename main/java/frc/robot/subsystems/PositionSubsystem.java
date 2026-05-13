// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain swerveDrive;
    private final CameraSubsystem camera;

    public PositionSubsystem(CommandSwerveDrivetrain swerveDrive, CameraSubsystem camera) {
        this.swerveDrive = swerveDrive;
        this.camera = camera;
    }

    private Matrix<N3, N1> getStdDevs(EstimatedRobotPose estimate) {
        var targets = estimate.targetsUsed;
        int tagCount = targets.size();
        double avgDist = targets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(0);

        if (tagCount >= 2) {
            return VecBuilder.fill(0.3, 0.3, 9999);
        } else if (tagCount == 1 && avgDist <= 4) {
            return VecBuilder.fill(0.7, 0.7, 9999);
        } else {
            return VecBuilder.fill(4.0, 4.0, 9999);
        }
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose();
        if (estimatedPose.isPresent()) {
            swerveDrive.addVisionMeasurement(
                estimatedPose.get().estimatedPose.toPose2d(),
                estimatedPose.get().timestampSeconds,
                getStdDevs(estimatedPose.get())
            );
        }
    }
}