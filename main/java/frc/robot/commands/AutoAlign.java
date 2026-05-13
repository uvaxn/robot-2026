// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlign extends Command {

    // Red and blue hub tag IDs
    private static final List<Integer> RED_HUB_TAGS  = List.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final List<Integer> BLUE_HUB_TAGS = List.of(18, 19, 20, 21, 24, 25, 26, 27);

    private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDeadband(Vars.MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final ProfiledPIDController rotationPID = new ProfiledPIDController(
        Vars.AlignToHubP,
        Vars.AlignToHubI,
        Vars.AlignToHubD,
        new TrapezoidProfile.Constraints(
            Vars.MaxAngularRate,
            Vars.MaxAngularRate * 2
        )
    );

    private final CommandSwerveDrivetrain swerveDrive;
    private final CameraSubsystem cameraSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier leftSupplier;

    // Precomputed hub center for odometry fallback
    private Translation2d hubCenter;
    private List<Integer> hubTagIds;

    public AutoAlign(CommandSwerveDrivetrain swerveDrive,
                     CameraSubsystem cameraSubsystem,
                     DoubleSupplier forwardSupplier,
                     DoubleSupplier leftSupplier,
                     double indexerPercent) {
        this.swerveDrive = swerveDrive;
        this.cameraSubsystem = cameraSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.leftSupplier = leftSupplier;
        addRequirements(swerveDrive);

        // PID treats rotation as continuous — -pi to pi
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Math.toRadians(2)); // 2 degree tolerance
    }

    @Override
    public void initialize() {
        // Determine alliance and pick correct tag IDs
        var alliance = DriverStation.getAlliance();
        hubTagIds = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
            ? RED_HUB_TAGS
            : BLUE_HUB_TAGS;

        // Precompute hub center from field layout for odometry fallback
        double totalX = 0, totalY = 0;
        int count = 0;
        for (int id : hubTagIds) {
            Optional<Pose2d> tagPose = cameraSubsystem.getTagPose2d(id);
            if (tagPose.isPresent()) {
                totalX += tagPose.get().getX();
                totalY += tagPose.get().getY();
                count++;
            }
        }
        hubCenter = (count > 0)
            ? new Translation2d(totalX / count, totalY / count)
            : new Translation2d(0, 0);

        // Reset PID to current heading
        swerveDrive.samplePoseAt(Timer.getFPGATimestamp())
            .ifPresent(pose -> rotationPID.reset(pose.getRotation().getRadians()));
    }

    @Override
    public void execute() {
        // Get current robot pose from odometry
        Optional<Pose2d> possiblePose = swerveDrive.samplePoseAt(Timer.getFPGATimestamp());
        if (possiblePose.isEmpty()) {
            swerveDrive.setControl(
                request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                       .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                       .withRotationalRate(0)
            );
            return;
        }

        Pose2d robotPose = possiblePose.get();

        // Try to find nearest visible hub tag from cached results
        double bestDist = Double.MAX_VALUE;
        Translation2d target = null;

        for (var result : cameraSubsystem.getCachedResults()) {
            if (!result.hasTargets()) continue;
            for (var photonTarget : result.getTargets()) {
                int id = photonTarget.getFiducialId();
                if (!hubTagIds.contains(id)) continue; // skip non-hub tags

                Optional<Pose2d> tagPose = cameraSubsystem.getTagPose2d(id);
                if (tagPose.isEmpty()) continue;

                double dist = robotPose.getTranslation()
                                       .getDistance(tagPose.get().getTranslation());
                if (dist < bestDist) {
                    bestDist = dist;
                    target = tagPose.get().getTranslation();
                }
            }
        }

        // Fall back to hub center if no hub tags visible
        if (target == null) {
            target = hubCenter;
        }

        // Calculate angle from robot to target
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        double targetAngle = Math.atan2(toTarget.getY(), toTarget.getX());

        rotationPID.setGoal(targetAngle);
        double output = rotationPID.calculate(robotPose.getRotation().getRadians());

        swerveDrive.setControl(
            request.withVelocityX(forwardSupplier.getAsDouble() * Vars.MaxSpeed)
                   .withVelocityY(leftSupplier.getAsDouble() * Vars.MaxSpeed)
                   .withRotationalRate(output)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}