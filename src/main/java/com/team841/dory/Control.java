package com.team841.dory;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.constants.SC;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.superstructure.AlgaePivot;
import com.team841.dory.superstructure.Escalator;
import com.team841.dory.superstructure.Escalator.Position;
import com.team841.dory.superstructure.Shooter.ShooterSpeeds;
import com.team841.dory.superstructure.FlapSystemAndHang;
import com.team841.dory.superstructure.MoveCommand;
import com.team841.dory.superstructure.AlgaePivot;
import com.team841.dory.superstructure.AlgaePivot.AlgaePivotPosition;
import com.team841.dory.superstructure.PivotMoveCommand;
import com.team841.dory.superstructure.Shooter;
import com.team841.dory.superstructure.Shooter.ShooterSpeeds;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Control {

    public static double scoreTimeout = 0.5;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Drivetrain drivetrain;
    private final Escalator escalator;
    private final AlgaePivot algaePivot;
    private final Shooter shooter;
    private final FlapSystemAndHang flapSystem;

    public final Command snapScoreL4;
    public final Command snapScoreL3;
    public final Command snapScoreL2;

    public final Command noSnapYCommand_L4_Barge;
    public final Command noSnapXCommand_L3_HighAlgae;
    public final Command noSnapBCommand_L2_LowAlgae;
    public final Command noSnapACommand_L1_Processor;

    public final Command goToStow;
    public final Command goToStowToCoralIntake;
    public final Command goToStowAfterScore;

    public final Command intake;
    public final Command manualShoot;
    public final Command stopShooter;

    public final Command swapMode;

    public final Command driveCommand;

    public final CommandXboxController joystick = new CommandXboxController(0);

    public boolean isCoralMode = true;

    public Control(Drivetrain drivetrain, Escalator escalator, AlgaePivot algaePivot, Shooter shooter, FlapSystemAndHang flapSystemAndHang){
        this.drivetrain = drivetrain;
        this.escalator = escalator;
        this.algaePivot = algaePivot;
        this.shooter = shooter;
        this.flapSystem = flapSystemAndHang;
        this.isCoralMode = true;

        if (!AutoBuilder.isConfigured()) {
            drivetrain.configureAutoBuilder();
        }

        // Commands for Autonomous Mode

        NamedCommands.registerCommand("L2", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));
      
        NamedCommands.registerCommand("L3", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L3, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear), 
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), 
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L4,
                        this.shooter::shooterHasCoral, this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("AutoL4", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoSnapInline(),
                        new MoveCommand(
                                this.escalator,
                                Escalator.Position.L4,
                                this.shooter::shooterHasCoral,
                                this.shooter::escalatorClear)),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))
                        .withTimeout(1))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("GoDown",
                new InstantCommand(() -> this.escalator.setPosition(Position.HomeAndIntake, false), escalator));

        NamedCommands.registerCommand("Intake", 
                new ParallelCommandGroup(this.shooter.runShooterIntake(), this.flapSystem.runIntake()));

        NamedCommands.registerCommand("PassiveRaise",
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, true)));

        NamedCommands.registerCommand("FlapDown",
                new RunCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.5), flapSystem)
                        .withTimeout(1)
                        .finallyDo(flapSystem::stopFlapper));
        
        // Commands for Teleop

        this.snapScoreL4 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        // Command for L4 and Barge, Stows if already at position

        this.noSnapYCommand_L4_Barge = new ConditionalCommand(
                // L4
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.L4, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.L4)), 
                // Barge
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new ConditionalCommand(
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeHighStow), 
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                        () -> this.shooter.hasAlgae())
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.Barge, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.BargeScore)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.Barge)), 
                () -> this.isCoralMode);
        this.noSnapYCommand_L4_Barge.addRequirements(escalator, algaePivot);

        // Command for L3 and High Algae Reef Pickup, Stows if already at position

        this.noSnapXCommand_L3_HighAlgae = new ConditionalCommand(
                // L3
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.L3, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.L3)), 
                // High Algae
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new ConditionalCommand(
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeHighStow), 
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                        () -> this.shooter.hasAlgae())
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HighAlgae, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.ReefPickup)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.HighAlgae)), 
                () -> this.isCoralMode);
        this.noSnapXCommand_L3_HighAlgae.addRequirements(escalator, algaePivot);

        // Command for L2 and Low Algae Reef Pickup, Stows if already at position

        this.noSnapBCommand_L2_LowAlgae = new ConditionalCommand(
                // L2
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.L2, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.L2)), 
                // Low Algae
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new ConditionalCommand(
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeHighStow), 
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                        () -> this.shooter.hasAlgae())
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.LowAlgae, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.ReefPickup)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.LowAlgae)), 
                () -> this.isCoralMode);
        this.noSnapBCommand_L2_LowAlgae.addRequirements(escalator, algaePivot);

        // Command for L2 and Processor, Stows if already at position

        this.noSnapACommand_L1_Processor = new ConditionalCommand(
                // L1
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.L1, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow)
                        ), 
                        () -> this.escalator.hasTarget(Escalator.Position.L1)), 
                // Processor
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new ConditionalCommand(
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeHighStow), 
                                        new PivotMoveCommand(
                                                this.algaePivot, 
                                                AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                        () -> this.shooter.hasAlgae())
                        ), 
                        new ParallelCommandGroup(
                                new MoveCommand(
                                        this.escalator, 
                                        Escalator.Position.HomeAndIntake, 
                                        this.shooter::shooterHasCoral, 
                                        this.shooter::escalatorClear),
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.ProcessorScore)
                        ), 
                        () -> this.algaePivot.hasTarget(AlgaePivot.AlgaePivotPosition.ProcessorScore)), 
                () -> this.isCoralMode);
        this.noSnapACommand_L1_Processor.addRequirements(escalator, algaePivot);

        // Command for stowing elevator and pivot manually

        this.goToStow = new ParallelCommandGroup(
                new MoveCommand(
                        this.escalator, 
                        Escalator.Position.HomeAndIntake, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear),
                new ConditionalCommand(
                        new PivotMoveCommand(
                                this.algaePivot,
                                AlgaePivot.AlgaePivotPosition.Stow), 
                        new ConditionalCommand(
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.AlgaeHighStow), 
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                () -> this.shooter.hasAlgae()), 
                        () -> this.isCoralMode)
        );
        this.goToStow.addRequirements(escalator, algaePivot);

        // Command for stowing elevator and pivot after scoring coral/algae

        this.goToStowAfterScore = new ParallelCommandGroup(
                new MoveCommand(
                        this.escalator, 
                        Escalator.Position.HomeAndIntake, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear),
                new ConditionalCommand(
                        new PivotMoveCommand(
                                this.algaePivot,
                                AlgaePivot.AlgaePivotPosition.Stow), 
                        new PivotMoveCommand(
                                this.algaePivot, 
                                AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                        () -> this.isCoralMode)
        );
        this.goToStowAfterScore.addRequirements(escalator, algaePivot);

        // Command for stowing elevator and pivot before intaking coral

        this.goToStowToCoralIntake = new ParallelCommandGroup(
                new MoveCommand(
                        this.escalator, 
                        Escalator.Position.HomeAndIntake, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear),
                new PivotMoveCommand(
                        this.algaePivot,
                        AlgaePivot.AlgaePivotPosition.Stow)
        );
        this.goToStowToCoralIntake.addRequirements(escalator, algaePivot);

        // Command for swapping robot mode and initializing pivot/shooter

        this.swapMode = new SequentialCommandGroup(
                new InstantCommand(() -> this.ChangeMode()),
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter), 
                                new InstantCommand(() -> this.shooter.setDutyCycle(Shooter.ShooterSpeeds.AlgaeHold), shooter),  
                                () -> this.isCoralMode),
                        new ConditionalCommand(
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.Stow), 
                                new PivotMoveCommand(
                                        this.algaePivot, 
                                        AlgaePivot.AlgaePivotPosition.AlgaeStow), 
                                () -> this.isCoralMode)
                )
        );

        // Command for intaking coral/algae

        this.intake = new ConditionalCommand(
                new ParallelCommandGroup(
                        new InstantCommand(() -> this.algaePivot.setPosition(AlgaePivot.AlgaePivotPosition.Stow), algaePivot),
                        this.shooter.runShooterIntake().onlyIf(() -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)), 
                        this.flapSystem.runIntake().onlyIf(() -> this.escalator.atPosition(Escalator.Position.HomeAndIntake))
                ), 
                new ConditionalCommand(
                        new InstantCommand(() -> this.shooter.setDutyCycle(Shooter.ShooterSpeeds.AlgaeIntake), shooter),
                        new InstantCommand(() -> this.shooter.setDutyCycle(Shooter.ShooterSpeeds.AlgaeHold), shooter),
                        (() -> !this.shooter.hasAlgae())
                ), 
                () -> this.isCoralMode);

        // Command for shooting coral/algae, coral shoot speeds controlled within shooter subsystem

        this.manualShoot = new ConditionalCommand(
                new InstantCommand(() -> this.shooter.coralShoot(this.escalator.getTarget()), shooter),
                new InstantCommand(() -> this.shooter.setDutyCycle(Shooter.ShooterSpeeds.Barge), shooter), 
                () -> this.isCoralMode);

        // Command for stopping shooter, stops if coral mode, static holds algae if not
        
        this.stopShooter = new ConditionalCommand(
                new InstantCommand(() -> this.shooter.setDutyCycle(0), shooter), 
                new InstantCommand(() -> this.shooter.setDutyCycle(Shooter.ShooterSpeeds.AlgaeHold), shooter), 
                () -> this.isCoralMode);

        // Command for driving swerve

        this.driveCommand = Drive(
            this.drivetrain, 
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX());
        configureBindings();
    }

    // Command for driving swerve

    public Command Drive(
            Drivetrain drivetrain,
            DoubleSupplier velocityX,
            DoubleSupplier velocityY,
            DoubleSupplier angularVelocity){
        
        return Commands.run(
            () -> {
                double controlX = velocityX.getAsDouble();
                double controlY = velocityY.getAsDouble();
                double controlAngularVelocity = angularVelocity.getAsDouble();
                double velX = controlX * MaxSpeed;
                double velY = controlY * MaxSpeed;
                double angularVel = controlAngularVelocity * MaxAngularRate;
                double throttleFieldFrame = RC.isRedAlliance.get() ? -velX : velX;
                double strafeFieldFrame = RC.isRedAlliance.get() ? -velY : velY;
                drivetrain.setControl(drivetrain.drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame).withRotationalRate(angularVel));
            }, drivetrain);
    }

    // Command for changing mode

    public void ChangeMode() {
        if (this.isCoralMode) {
                this.isCoralMode = false;
        } else {
                this.isCoralMode = true;
        }
    }

    /**
     * Pose to Pose autoAlign command that will drive to the closest scoring command.
     * It is two ProfiledPIDControllers, one for X and one for Y, that will close the robot pose to the target pose
     * We use SwerveRequest.FieldCentricFacingAngle so that we do not have to deal with rotation
     * First we wait for the headingController to point the robot at the right direction before moving.
     * This deals with issues of profiling turning with vx and vy which is not a simple control.
     * @return AutoAlign Command
     */
    public Command AutoSnapInline(){
        ProfiledPIDController vx = drivetrain.vxController;
        ProfiledPIDController vy = drivetrain.vyController;
        AtomicReference<Pose2d> target = new AtomicReference<>();

        return Commands.runOnce(
                // Sets everything up, reset the controllers because we have a non-zero kI
                () -> {
                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            this.drivetrain.getState().Speeds, this.drivetrain.getState().Pose.getRotation());
                    target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));
                    DogLog.log("AutoSnap/PoseTarget", target.get());
                    Translation2d translation2d = this.drivetrain.getState().Pose.getTranslation();
                    vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
                    vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
                }
        ).andThen(
                Commands.run(
                        // Points the drivetrain in the right direction
                        () -> this.drivetrain.setControl(
                                drivetrain.driveHeading.withTargetDirection(target.get().getRotation()))
                ).until(
                        // Wait until we are facing the right direction
                        this.drivetrain.driveHeading.HeadingController::atSetpoint
                ).andThen(
                       Commands.run(
                               () -> {
                                   ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                           this.drivetrain.getState().Speeds, this.drivetrain.getState().Pose.getRotation());
                                   Translation2d translation2d = this.drivetrain.getState().Pose.getTranslation();

                                   // Add the current setpoint velocity to the new velocity so that it is smoother.
                                   double outputX =
                                           vx.calculate(translation2d.getX(), target.get().getX()) +
                                                   vx.getSetpoint().velocity;
                                   double outputY =
                                           vy.calculate(translation2d.getY(), target.get().getY()) +
                                                   vy.getSetpoint().velocity;

                                   // If it is not scaled, then each controller can output speeds that
                                   // when added up to the final vector will exceed the maximum real speed of the robot.
                                   outputX *= 0.7;
                                   outputY *= 0.7;

                                   // Do some logging to help with debugging
                                   DogLog.log("AutoSnap/outputX", outputX);
                                   DogLog.log("AutoSnap/outputY", outputY);
                                   DogLog.log("AutoSnap/vxAtGoal", vx.atGoal());
                                   DogLog.log("AutoSnap/vyAtGoal", vy.atGoal());

                                   this.drivetrain.setControl(
                                           this.drivetrain.driveHeading
                                                   .withVelocityY(outputY)
                                                   .withVelocityX(outputX)
                                                   .withTargetDirection(target.get().getRotation())
                                   );
                               }
                       ).until(
                               () -> vx.atGoal() && vy.atGoal()
                       ).finallyDo(
                               () -> this.drivetrain.setControl(
                                       this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()))
                       )
                )
        );
    }

    private void configureBindings() {
        // Drive
        drivetrain.setDefaultCommand(driveCommand);

        // Zero odometry (front of robot facing you)
        joystick.start().onTrue(new InstantCommand(drivetrain::seedFieldCentric));

        // Zero elevator and pivot
        joystick.back().onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(this.escalator::zero),
                        new InstantCommand(this.algaePivot::zero)
                ));

        /* No Snap Setpoints */
        joystick.y().onTrue(noSnapYCommand_L4_Barge);

        joystick.x().onTrue(noSnapXCommand_L3_HighAlgae);

        joystick.b().onTrue(noSnapBCommand_L2_LowAlgae);

        joystick.a().onTrue(noSnapACommand_L1_Processor);

        /* Automated */
        //joystick.back().and(joystick.y()).whileTrue(snapScoreL4);

        //joystick.back().and(joystick.x()).whileTrue(snapScoreL3);

        //joystick.back().and(joystick.b()).whileTrue(snapScoreL2);

        // Force stow
        joystick.leftBumper().onTrue(goToStow);

        // Swap mode
        joystick.rightBumper().onTrue(swapMode);

        // Intake
        joystick.leftTrigger().and(() -> !this.shooter.shooterHasCoral()).whileTrue(intake).onFalse(stopShooter);

        // Auto stow for intake
        joystick.leftTrigger().and(() -> this.isCoralMode).onTrue(goToStowToCoralIntake);

        // Shoot
        joystick.rightTrigger().onTrue(manualShoot).onFalse(stopShooter);

        // Auto stow after shooting
        joystick.rightTrigger().onFalse(goToStowAfterScore);

        // Manually lower elevator
        joystick.povDown().whileTrue(this.escalator.goDown());

        // Manually raise elevator
        joystick.povUp().whileTrue(this.escalator.goUp());

        // Raise flap
        joystick.povLeft().whileTrue(
                new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

        // Lower flap
        joystick.povRight().whileTrue(
                new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.25), flapSystem))
                .onFalse(new InstantCommand(() -> this.flapSystem.stopFlapper(), flapSystem));

        // Backdrive (for unjamming)
        joystick.leftStick().onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> this.shooter.setDutyCycle(0.5)),
                        new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(-0.3))
                )).onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(() -> this.shooter.setDutyCycle(0)),
                        new InstantCommand(() -> this.flapSystem.setIntakeDutyCycle(0))
                ));

        // Start climb sequence
        joystick.rightStick().onTrue(new InstantCommand(() -> this.flapSystem.startClimb(), flapSystem));
    }
}
