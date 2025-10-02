package com.team841.dory.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;

/**
 * RC: Robot Constants, this is for general constants.
 */
public class RC {
    public static final RunType robotType = RunType.COMP;

    public static final CANBus canivoreCANBus = TunerConstants.kCANBus;

    public static Mode getMode() {
        return switch (robotType) {
            case DEV, COMP -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIM -> Mode.SIM;
            case REPLAY -> Mode.REPLAY;
        };
    }

    public static final double LOOKBACK_TIME = 1.0;

    public static Supplier<Boolean> isRedAlliance = () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    };

    public static class Vision {
        public static final double LOOKBACK_TIME = 1.0;
        // charlie is Right, gamma is left
        public static String LimelightCharlieName = "limelight-charlie";
        public static String LimelightGammaName = "limelight-gamma";
        // x y z roll pitch yaw
        public static double[] CharliePose = {0.1593, 0.225377, 0.2214, 0.0, 12.76, 0.5};
        public static double[] GammaPose = {0.1593, -0.225377, 0.2214, 0.0, 12.76, 0.2};
    }

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public enum RunType {
        SIM, // Simulation
        DEV, // Developer-tuning mode
        COMP, // Comp code, real robot code
        REPLAY
    }
}
