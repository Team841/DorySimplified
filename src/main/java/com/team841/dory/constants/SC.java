package com.team841.dory.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

/**
 * SC: Superstructure Constants
 */
public class SC {

    public static class Escalator {
        public static int left = 17;
        public static int right = 36;
        public static int CANRange = 0;
        // with out coral
        public static Slot0Configs slot0Configs =
                new Slot0Configs()
                        .withKP(2)
                        .withKD(0)
                        .withKS(0)
                        .withKA(0.12)
                        .withKG(0.288)
                        .withGravityType(GravityTypeValue.Elevator_Static)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        // public static double FF = 12.2;
        private static final TalonFXConfiguration configs =
                new TalonFXConfiguration()
                        .withSlot0(slot0Configs)
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                        .withFeedback(
                                new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(175)
                                        .withMotionMagicCruiseVelocity(100)
                                        .withMotionMagicJerk(50)
                                        .withMotionMagicExpo_kA(0.1)
                                        .withMotionMagicExpo_kV(0.12))
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake))
                        .withSoftwareLimitSwitch(
                                new SoftwareLimitSwitchConfigs()
                                        .withForwardSoftLimitEnable(true)
                                        .withReverseSoftLimitEnable(true)
                                        .withForwardSoftLimitThreshold(42)
                                        .withReverseSoftLimitThreshold(0));
        public static final TalonFXConfiguration leftConfigs = configs;
        public static final TalonFXConfiguration rightConfigs = configs.withMotorOutput(
                configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
        ); 
    }

    public static class AlgaePivot {
        public static int pivotMotorId = -1;

        // without algae
        public static Slot0Configs slot0Configs =
                new Slot0Configs()
                        .withKP(125)
                        .withKD(0)
                        .withKS(0.24274)
                        .withKA(0)
                        .withKV(0)
                        .withKG(0.66177)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        // with algae
        public static Slot1Configs slot1Configs =
                new Slot1Configs()
                        .withKP(125)
                        .withKD(0)
                        .withKS(0.65347)
                        .withKA(0)
                        .withKV(0)
                        .withKG(2.0762)
                        .withGravityType(GravityTypeValue.Arm_Cosine)
                        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        public static final TalonFXConfiguration pivotConfigs =
                new TalonFXConfiguration()
                        .withSlot0(slot0Configs)
                        .withSlot1(slot1Configs)
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimitEnable(true))
                        .withFeedback(
                                new FeedbackConfigs()
                                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                                        .withRotorToSensorRatio(87.8571428571)
                                        .withFeedbackRotorOffset(0))
                        .withMotionMagic(
                                new MotionMagicConfigs()
                                        .withMotionMagicAcceleration(2)
                                        .withMotionMagicCruiseVelocity(5))
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withNeutralMode(NeutralModeValue.Brake))
                        //.withSoftwareLimitSwitch(
                        //        new SoftwareLimitSwitchConfigs()
                        //                .withForwardSoftLimitEnable(true)
                        //                .withReverseSoftLimitEnable(true)
                        //                .withForwardSoftLimitThreshold(23)
                        //                .withReverseSoftLimitThreshold(0))
                        ;
    }

    public static class Shooter {
        public static int MotorId = 35;
        public static int CanRangeIdFront = 6;
        public static int CanRangeIdBack = 2;

        public static double manualShootDutyCycleL4 = 0.5;
        public static double manualShootDutyCycleL3L2 = 0.3;
        public static double manualShootDutyCycleL1 = 0.17;

        public static double algaeCurrentMinimum = -1;

        public static TalonFXConfiguration configs =
                new TalonFXConfiguration()
                        .withCurrentLimits(
                                new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(60)
                                        .withStatorCurrentLimitEnable(true)
        );


        private static final FovParamsConfigs fovParamsConfigs =
                new FovParamsConfigs()
                        .withFOVCenterX(-6.75)
                        .withFOVCenterY(6.75)
                        .withFOVRangeX(6.75)
                        .withFOVRangeY(6.75);

        private static final ProximityParamsConfigs proximityParamsConfigs =
                new ProximityParamsConfigs()
                        .withProximityThreshold(0.4)
                        .withProximityHysteresis(0.01)
                        .withMinSignalStrengthForValidMeasurement(2500);

        private static final ToFParamsConfigs toFParamsConfigs =
                new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRange100Hz)
                        .withUpdateFrequency(50);

        public static final CANrangeConfiguration CanrangeConfigs =
                new CANrangeConfiguration()
                        .withFovParams(fovParamsConfigs)
                        .withProximityParams(proximityParamsConfigs)
                        .withToFParams(toFParamsConfigs);
    }

    public static class flapSystem {
        public static int flapMotor = 9;
        public static int intakeMotor = 14;
        public static int intakeCanRangeId = 3;
        public static int hangMotor = 12;
        public static int grabMotor = 16;
        public static int throughbore = 37;

        public static double hangDeployedAngle = -0.125;
        public static double hangHungAngle = 0.118;
        public static double grabMotorCurrentTrigger = 30;
        public static double grabMotorCurrentTriggerDelay = 0.5;
        
        public static double hangDeployingDutyCycle = 1;
        public static double hangDeployingDutyCycle2 = 0.4;
        public static double hangHangingDutyCycle = 1;
        public static double grabDutyCycle = -1;

        public static TalonFXConfiguration configs =
                new TalonFXConfiguration()
                        .withCurrentLimits(
                                new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(60)
                                        .withStatorCurrentLimitEnable(true)
        );

        public static TalonFXConfiguration climbConfigs =
                new TalonFXConfiguration()
                        .withCurrentLimits(
                                new CurrentLimitsConfigs()
                                        .withStatorCurrentLimit(120)
                                        .withStatorCurrentLimitEnable(true)
        );

        private static final FovParamsConfigs fovParamsConfigs =
                new FovParamsConfigs()
                        .withFOVCenterX(0)
                        .withFOVCenterY(0)
                        .withFOVRangeX(27)
                        .withFOVRangeY(27);

        private static final ProximityParamsConfigs proximityParamsConfigs =
                new ProximityParamsConfigs()
                        .withProximityThreshold(0.4)
                        .withProximityHysteresis(0.01)
                        .withMinSignalStrengthForValidMeasurement(2500);

        private static final ToFParamsConfigs toFParamsConfigs =
                new ToFParamsConfigs()
                        .withUpdateMode(UpdateModeValue.ShortRange100Hz)
                        .withUpdateFrequency(50);

        public static final CANrangeConfiguration CanrangeConfigs =
                new CANrangeConfiguration()
                        .withFovParams(fovParamsConfigs)
                        .withProximityParams(proximityParamsConfigs)
                        .withToFParams(toFParamsConfigs);
    }
}
