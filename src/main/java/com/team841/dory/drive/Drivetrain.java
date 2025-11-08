package com.team841.dory.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.limelight.LimelightHelpers;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import com.team254.vision.VisionFieldPoseEstimate;
import com.team254.vision.VisionProcessor;
import com.team841.dory.constants.Field;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;


public class Drivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem{

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond);

    Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer = new Consumer<>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
            addVisionMeasurement(visionFieldPoseEstimate);
                 return;
        }
    };

    VisionProcessor visionProcessor = new VisionProcessor(visionEstimateConsumer);

    NetworkTable CharlieTable = NetworkTableInstance.getDefault().getTable(RC.Vision.LimelightCharlieName);
    NetworkTable GammaTable = NetworkTableInstance.getDefault().getTable(RC.Vision.LimelightGammaName);

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);
    
    public final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds()
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                    .withSteerRequestType(SteerRequestType.Position);

    public ProfiledPIDController vxController = new ProfiledPIDController(
            9.4, 0.01, 0.1, new TrapezoidProfile.Constraints(
                    4.25, 1.9) // max velocity, max acceleration
    );
    public ProfiledPIDController vyController = new ProfiledPIDController(
            9.4, 0.01, 0.1, new TrapezoidProfile.Constraints(
                    4.25, 1.9) // max velocity, max acceleration
    );

    public final SwerveRequest.FieldCentricFacingAngle driveHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.Position)
                    .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
                    .withDesaturateWheelSpeeds(false);
    
    public final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
                //.withDeadband(MaxSpeed * 0.1)
                .withDeadband(0.06)
                //.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withRotationalDeadband(0.06)
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules){
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);

        this.vxController.setTolerance(Units.inchesToMeters(0.5));
        this.vyController.setTolerance(Units.inchesToMeters(0.5));

        driveHeading.HeadingController.setPID(23.356, 0, 2.5039);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        driveHeading.HeadingController.setTolerance(0.01, 0.01);
    }

    @Override
    public void periodic(){
        visionPeriodic();
        telemetry.telemeterize(this.getState());
    }

    public void setPose(Pose2d pose) {
        this.resetPose(pose);
    }

    /**
     * Run calculation to the reefpose to find the polar angle the robot is to the reef in wpilib coordinate system
     * @return double
     */
    public double getAngleToReefPolar() {
        boolean isRed = RC.isRedAlliance.get();
        Translation2d robotVector;

        if (isRed) robotVector = this.getState().Pose.getTranslation().minus(Field.Positions.Reef.redTranslation2d);
        else robotVector = this.getState().Pose.getTranslation().minus(Field.Positions.Reef.blueTranslation2d);

        return Math.atan2(robotVector.getY(), robotVector.getX()) * 57.2957795131;
    }

    /**
     * Calculate the scoring letter the robot is closet to based of the angle to the reef it is at
     * @param angle the angle in degrees to determine the scoring position
     * @return Field.ScoringPositions
     */
    public Field.ScoringPositions getScoringPosition(double angle) {
        if (angle >= 0 && angle < 30) {
            return Field.ScoringPositions.H;
        } else if (angle >= 30 && angle < 60) {
            return Field.ScoringPositions.I;
        } else if (angle >= 60 && angle < 90) {
            return Field.ScoringPositions.J;
        } else if (angle >= 90 && angle < 120) {
            return Field.ScoringPositions.K;
        } else if (angle >= 120 && angle < 150) {
            return Field.ScoringPositions.L;
        } else if (angle >= 150 && angle < 180) {
            return Field.ScoringPositions.A;
        } else if (angle >= -180 && angle < -150) {
            return Field.ScoringPositions.B;
        } else if (angle >= -150 && angle < -120) {
            return Field.ScoringPositions.C;
        } else if (angle >= -120 && angle < -90) {
            return Field.ScoringPositions.D;
        } else if (angle >= -90 && angle < -60) {
            return Field.ScoringPositions.E;
        } else if (angle >= -60 && angle < -30) {
            return Field.ScoringPositions.F;
        } else {
            return Field.ScoringPositions.G;
        }
    }

    public Pose2d getPoseToScore(double angle) {
        if (RC.isRedAlliance.get()) {
            return getScoringPosition(angle).getPoseRed();
        } else {
            return getScoringPosition(angle).getPoseBlue();
        }
    }

    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> this.getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> this.getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds).withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons()).withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())), new PPHolonomicDriveController(
                                    // PID constants for translation
                                    new PIDConstants(6.75, 0, 0),
                                    // PID constants for rotation
                                    new PIDConstants(5, 0, 0)), config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        if (visionFieldPoseEstimate.getVisionMeasurementStdDevs() == null) {
            super.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds())
            );
        } else {
            super.addVisionMeasurement(
                    visionFieldPoseEstimate.getVisionRobotPoseMeters(),
                    Utils.fpgaToCurrentTime(visionFieldPoseEstimate.getTimestampSeconds()),
                    visionFieldPoseEstimate.getVisionMeasurementStdDevs()
            );
        }
    }

    public void visionPeriodic(){
        setLLSettings();

        double timestamp = Timer.getTimestamp();
        visionProcessor.driveYawAngularVelocity.addSample(timestamp, this.getState().Speeds.omegaRadiansPerSecond);
        visionProcessor.measuredRobotRelativeChassisSpeeds.set(this.getState().Speeds);
        visionProcessor.robotPose.addSample(timestamp, this.getState().Pose);
        boolean gammaSeesTarget = GammaTable.getEntry("tv").getDouble(0) == 1.0;
        boolean charlieSeesTarget = CharlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (gammaSeesTarget) { // does it see target?
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightGammaName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightGammaName);
            if (megatag != null && megatag2 != null){
                visionProcessor.updateVision(
                    gammaSeesTarget,
                    FiducialObservation.fromLimelight(megatag.rawFiducials),
                    MegatagPoseEstimate.fromLimelight(megatag),
                    MegatagPoseEstimate.fromLimelight(megatag2),
                    "Vision/Gamma");
            }
        }
        if (charlieSeesTarget) {
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Vision.LimelightCharlieName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Vision.LimelightCharlieName);
            if (megatag != null && megatag2 != null){
                visionProcessor.updateVision(
                    charlieSeesTarget,
                    FiducialObservation.fromLimelight(megatag.rawFiducials),
                    MegatagPoseEstimate.fromLimelight(megatag),
                    MegatagPoseEstimate.fromLimelight(megatag2),
                    "Vision/Charlie");
            }
        }
    }

    private void setLLSettings() {
        GammaTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Vision.GammaPose);
        CharlieTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Vision.CharliePose);

        Rotation2d gyroAngle = this.getState().Pose.getRotation();
        double gyroAngularVelocity = Math.toDegrees(this.getState().Speeds.omegaRadiansPerSecond);
        try {
            LimelightHelpers.SetIMUMode(RC.Vision.LimelightGammaName, 1);
            LimelightHelpers.SetIMUMode(RC.Vision.LimelightCharlieName, 1);
            LimelightHelpers.SetRobotOrientation(
                    RC.Vision.LimelightGammaName,
                    gyroAngle.getDegrees(), gyroAngularVelocity, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(
                    RC.Vision.LimelightCharlieName,
                    gyroAngle.getDegrees(), gyroAngularVelocity, 0, 0, 0, 0);

        } catch (Exception e) {
           return;
        }
    }
}
