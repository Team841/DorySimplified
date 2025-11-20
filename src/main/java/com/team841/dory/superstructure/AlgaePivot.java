package com.team841.dory.superstructure;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.dory.constants.SC;

import dev.doglog.DogLog;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {
    
    TalonFX pivotMotor = new TalonFX(SC.AlgaePivot.pivotMotorId, "rio");

    MotionMagicExpoVoltage pivotControl = new MotionMagicExpoVoltage(0).withSlot(0);
    
    DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode[] latestStatus;

    AlgaePivotPosition pivotTargetPosition = AlgaePivotPosition.Stow;

    public AlgaePivot() {
        this.pivotMotor.getConfigurator().apply(SC.AlgaePivot.pivotConfigs);
        this.pivotMotor.setNeutralMode(NeutralModeValue.Brake);
        this.zero();
    }

    @Override
    public void periodic() {
        Angle pivotMotorPos = pivotMotor.getPosition().getValue();
        DogLog.log("AlgaePivot/AtStow", this.atPosition(AlgaePivotPosition.Stow));
        DogLog.log("AlgaePivot/TargetPosition", this.pivotTargetPosition.toString());
        DogLog.log("AlgaePivot/PositionRadian", pivotMotorPos.in(Units.Rotation));
    }

    public void setPosition(AlgaePivotPosition position) {
        this.latestStatus = setControl(pivotControl.withPosition(position.getPosition()));
        this.pivotTargetPosition = position;
    }

    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[]{this.pivotMotor.setControl(control)};
    }

    public void resetPositions(double position) {
        this.pivotMotor.setPosition(position);
    }

    public void zero() {
        this.resetPositions(0);
    }

    public AlgaePivotPosition getTarget() {
        return this.pivotTargetPosition;
    }

    public boolean hasTarget(AlgaePivotPosition position) {
        return this.pivotTargetPosition == position;
    }

    public boolean atPosition(AlgaePivotPosition position) {
        return Math.abs(pivotMotor.getPosition().getValue().in(Units.Rotation) - position.getPosition()) < 0.5;
    }

    public enum AlgaePivotPosition {
        ReefPickup(6),
        GroundPickup(4),
        BargeScore(32),
        ProcessorScore(5),
        Stow(0),
        AlgaeStow(3),
        AlgaeHighStow(32);

        private final double rotationsPosition;

        AlgaePivotPosition(double rotationsPosition) {
            this.rotationsPosition = rotationsPosition;
        }

        public double getPosition() {
            return rotationsPosition;
        }
    }
}
