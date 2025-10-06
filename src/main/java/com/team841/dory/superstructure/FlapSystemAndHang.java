package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.dory.constants.SC;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class FlapSystemAndHang extends SubsystemBase{

    public TalonFX intakeMotor = new TalonFX(SC.flapSystem.intakeMotor, "rio");
    public TalonFX flapMotor = new TalonFX(SC.flapSystem.flapMotor, "rio");
    public CANrange canrange = new CANrange(SC.flapSystem.intakeCanRangeId, "rio");
    public TalonFX hangMotor = new TalonFX(SC.flapSystem.hangMotor, "rio");
    public TalonFX grabMotor = new TalonFX(SC.flapSystem.grabMotor, "rio");
    public CANcoder throughbore = new CANcoder(SC.flapSystem.throughbore, "rio");

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private Timer timer;
    private boolean deployDebounce;
    private boolean reverseHang;

    public String hangState;

    StatusCode latestStatusCode;

    public FlapSystemAndHang(){
        this.intakeMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        this.flapMotor.getConfigurator().apply(
                SC.flapSystem.configs.withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(5)
                                .withSupplyCurrentLimitEnable(true)));
        this.flapMotor.setNeutralMode(NeutralModeValue.Brake);

        this.canrange.getConfigurator().apply(SC.flapSystem.CanrangeConfigs);

        this.hangMotor.getConfigurator().apply(SC.flapSystem.climbConfigs);
        this.hangMotor.setNeutralMode(NeutralModeValue.Brake);

        this.grabMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.grabMotor.setNeutralMode(NeutralModeValue.Brake);

        this.hangState = "Stowed";
        this.deployDebounce = false;
        this.reverseHang = false;
        this.timer = new Timer();
        this.timer.reset();

        // this.hangMotor2.setControl(new Follower(SC.flapSystem.hangMotor, false));
    }

    public void periodic() {
        DogLog.log("FlapSystemAndHang/HangAngle", this.getHangAngle());
        DogLog.log("FlapSystemAndHang/HangState", this.hangState);

        if (this.hangState.equals("Deploying")) {
            this.setHangDutyCycle(SC.flapSystem.hangDeployingDutyCycle);
            this.setGrabDutyCycle(SC.flapSystem.grabDutyCycle);
            if (this.getHangAngle() < SC.flapSystem.hangDeployedAngle - 0.01) {
                this.deployDebounce = true;
            }
            if (this.getHangAngle() > SC.flapSystem.hangDeployedAngle && this.deployDebounce == true) {
                this.setHangState("Deployed");
            }
        } else if (this.hangState.equals("Deployed")) {
            this.setHangDutyCycle(0);
            if (this.grabMotor.getStatorCurrent().getValueAsDouble() > SC.flapSystem.grabMotorCurrentTrigger) {
                this.timer.start();
                if (this.timer.hasElapsed(SC.flapSystem.grabMotorCurrentTriggerDelay)) {
                    this.setHangState("Hanging");
                }
            }
        } else if (this.hangState.equals("Hanging")) {
            this.setHangDutyCycle(SC.flapSystem.hangHangingDutyCycle);
            if (this.getHangAngle() > SC.flapSystem.hangHungAngle) {
                this.setHangState("Hung");
            }
        } else {
            if (this.reverseHang == false) {
                this.setGrabDutyCycle(0);
                this.setHangDutyCycle(0);
            }
        }
    }

    public Command runIntake() {
        return new RunCommand(
                () -> {
                    this.setIntakeDutyCycle(0.2);
                }
        ).withName("runShooterIntakeCommand")
                .finallyDo(this::stopIntake);
    }

    public void startClimb() {
        if (DriverStation.getMatchTime() < 20) {
            if (this.hangState.equals("Stowed")) {
                this.setHangState("Deploying");
                this.timer.reset();
            }
            if (this.hangState.equals("Deployed")) {
                this.setHangState("Hanging");
            }
        }
    }

    public void reverseClimb() {
        this.reverseHang = true;
        this.setHangState("Reversing");
        this.setHangDutyCycle(-0.8);
        this.setGrabDutyCycle(0);
        this.deployDebounce = false;
    }

    public void stopClimb() {
        this.setHangDutyCycle(0);
        this.reverseHang = false;
        this.setHangState("Stowed");
        this.deployDebounce = false;
    }
    
    private void setHangState(String state) {
        this.hangState = state;
    }

    public boolean flapHasCoral() {
        return this.canrange.getDistance().getValue().magnitude() < 0.05;
    }

    public double getHangAngle() {
        return this.throughbore.getAbsolutePosition().getValueAsDouble();
    }

    public void setIntakeDutyCycle(double output) {
        this.latestStatusCode = setControlIntake(dutyCycle.withOutput(output));
    }

    public void setFlapperDutyCycle(double output) {
        this.latestStatusCode = setControlFlapper(dutyCycle.withOutput(output));
    }

    public void setHangDutyCycle(double output) {
        this.latestStatusCode = setControlHang(dutyCycle.withOutput(output));
    }

    public void setGrabDutyCycle(double output) {
        this.latestStatusCode = setControlGrab(dutyCycle.withOutput(output));
    }

    public StatusCode setControlIntake(DutyCycleOut control) {
        return this.intakeMotor.setControl(control);
    }

    public StatusCode setControlFlapper(DutyCycleOut control) {
        return this.flapMotor.setControl(control);
    }

    public StatusCode setControlHang(DutyCycleOut control) {
        return this.hangMotor.setControl(control);
    }

    public StatusCode setControlGrab(DutyCycleOut control) {
        return this.grabMotor.setControl(control);
    }

    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }

    public void stopFlapper() {
        this.flapMotor.stopMotor();
    }

    public void stopHang() {
        this.hangMotor.stopMotor();
    }
}
