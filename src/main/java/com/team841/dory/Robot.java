// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team841.dory;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.superstructure.AlgaePivot;
import com.team841.dory.superstructure.Escalator;
import com.team841.dory.superstructure.FlapSystemAndHang;
import com.team841.dory.superstructure.Shooter;
import com.team841.dory.superstructure.LED;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final SendableChooser<Command> autoChooser;

  private final Drivetrain drivetrain;
  private final Escalator escalator;
  private final Shooter shooter;
  private final FlapSystemAndHang flapSystem;
  private final AlgaePivot algaePivot;
//  private final LED led;

  private final Control control;

  public Robot() {

    this.drivetrain = new Drivetrain(TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
    
    this.escalator = new Escalator();
    this.shooter = new Shooter();
    this.flapSystem = new FlapSystemAndHang();
    this.algaePivot = new AlgaePivot();
//    this.led = new LED(this.shooter, this.flapSystem);
    
    this.control = new Control(drivetrain, escalator, algaePivot, shooter, flapSystem);
    
    DogLog.setOptions(new DogLogOptions().withLogExtras(true).withCaptureDs(true).withCaptureNt(true));

    DogLog.log("ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("GitDate", BuildConstants.GIT_DATE);
    DogLog.log("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
          DogLog.log("GitDirty", "All changes committed");
          break;
      case 1:
          DogLog.log("GitDirty", "Uncomitted changes");
          break;
      default:
          DogLog.log("GitDirty", "Unknown");
          break;
    }

    SignalLogger.enableAutoLogging(false);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    Threads.setCurrentThreadPriority(true, 5);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putBoolean("IsCoralMode", this.control.isCoralMode);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
