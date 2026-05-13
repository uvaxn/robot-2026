// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
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
    private static final Set<Integer> RED_HUB_TAGS  = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    private static final Set<Integer> BLUE_HUB_TAGS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);

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
    private Set<Integer> hubTagIds;
    private Translation2d lockedTarget; // add this field
    public AutoAlign(CommandSwerveDrivetrain swerveDrive,
                     CameraSubsystem cameraSubsystem,
                     DoubleSupplier forwardSupplier,
                     DoubleSupplier leftSupplier
                     ) {    
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
    if (alliance.isEmpty()) {
        DriverStation.reportWarning("AutoAlign: No alliance, defaulting to Blue", false);
    }
    hubTagIds = (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
        ? RED_HUB_TAGS : BLUE_HUB_TAGS;

    // Compute hub center from field layout
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

        lockedTarget = hubCenter; // default to hub center
        Pose2d robotPose = swerveDrive.getState().Pose;
        double[] bestDist = { Double.MAX_VALUE };
        for (var result : cameraSubsystem.getCachedResults()) {
            if (!result.hasTargets()) continue;
            for (var t : result.getTargets()) {
                int id = t.getFiducialId();
                if (!hubTagIds.contains(id)) continue;
                cameraSubsystem.getTagPose2d(id).ifPresent(pose -> {
                    double d = robotPose.getTranslation().getDistance(pose.getTranslation());
                    if (d < bestDist[0]) { bestDist[0] = d; lockedTarget = pose.getTranslation(); }
                });
            }
        }

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
        Translation2d target = (lockedTarget != null) ? lockedTarget : hubCenter;
        // Calculate angle from robot to target
        Translation2d toTarget = target.minus(robotPose.getTranslation());
        double targetAngle = Math.atan2(toTarget.getY(), toTarget.getX());

        rotationPID.setGoal(targetAngle);
            double output = MathUtil.clamp(
                rotationPID.calculate(robotPose.getRotation().getRadians()),
                -Vars.MaxAngularRate,
                Vars.MaxAngularRate
            );

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