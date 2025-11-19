package com.team841.dory.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.function.BooleanSupplier;

import com.team841.dory.constants.SC.flapSystem;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class LED extends SubsystemBase{
    
    private Shooter shooter;
    private Timer timer;
    private FlapSystemAndHang flapSystemAndHang;
    private BooleanSupplier isCoralModeSupplier;
    private boolean isCoralMode;

    private final AddressableLED LED = new AddressableLED(0);
    private final AddressableLEDBuffer Buffer = new AddressableLEDBuffer(61);
    private final AddressableLEDBufferView BufferLeft = Buffer.createView(0, 28);
    private final AddressableLEDBufferView BufferRight = Buffer.createView(29, 60).reversed();
    
    Color yellow = new Color(255, 200, 0);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 255, 0);
    Color blue = new Color(0, 0, 255);
    Color blueGreen = new Color(0, 225, 225);
    Color purple = new Color(255, 0, 255);
    Distance ledSpacing = Meters.of(1 / 120.0);

    LEDPattern baseYellowScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, yellow);
    LEDPattern patternYellowScroll = baseYellowScroll.scrollAtRelativeSpeed(Percent.per(Second).of(60));

    LEDPattern baseYellowRedScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, red, yellow);
    LEDPattern patternYellowRedScroll = baseYellowRedScroll.scrollAtRelativeSpeed(Percent.per(Second).of(60));

    LEDPattern baseBlueScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, blue);
    LEDPattern patternBlueScroll = baseBlueScroll.scrollAtRelativeSpeed(Percent.per(Second).of(130));

    LEDPattern basePurpleScroll = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, purple);
    LEDPattern patternPurpleScroll = basePurpleScroll.scrollAtRelativeSpeed(Percent.per(Second).of(100));

    LEDPattern baseRedBreathe = LEDPattern.solid(red);
    LEDPattern patternRedBreathe = baseRedBreathe.breathe(Second.of(0.25));

    LEDPattern baseBlueBreathe = LEDPattern.solid(blue);
    LEDPattern patternBlueBreathe = baseBlueBreathe.breathe(Second.of(0.15));

    LEDPattern baseBlueGreenScroll = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, blue, green);
    LEDPattern patternBlueGreenScroll = baseBlueGreenScroll.scrollAtRelativeSpeed(Percent.per(Second).of(30));

    LEDPattern baseBlueGreenBreathe = LEDPattern.solid(blueGreen);
    LEDPattern patternBlueGreenBreathe = baseBlueGreenBreathe.breathe(Second.of(0.4));

    LEDPattern patternYellowSolid = LEDPattern.solid(yellow);
    LEDPattern patternRedSolid = LEDPattern.solid(red);
    LEDPattern patternGreenSolid = LEDPattern.solid(green);
    LEDPattern patternBlueSolid = LEDPattern.solid(blue);

    public LED(Shooter shooter, FlapSystemAndHang flapSystemAndHang, BooleanSupplier isCoralModeSupplier) {
        this.shooter = shooter;
        this.flapSystemAndHang = flapSystemAndHang;
        this.isCoralModeSupplier = isCoralModeSupplier;
        this.isCoralMode = isCoralModeSupplier.getAsBoolean();
        this.timer = new Timer();
        LED.setLength(Buffer.getLength());
        patternRedSolid.applyTo(BufferLeft);
        patternRedSolid.applyTo(BufferRight);
        LED.setData(Buffer);
        LED.start();
    }

    public void periodic() {
        
        this.isCoralMode = isCoralModeSupplier.getAsBoolean();

        if (DriverStation.getMatchTime() < 21 && DriverStation.getMatchTime() > 20) {
            this.timer.reset();
            this.timer.start();
        }

        /*
        LED States & Colors:
        Booting up - Solid Red
        Booted up, driverstation disconnected - Yellow & Red Scrolling
        Booted up, driverstation connected, disabled - Yellow Scrolling
        Teleop enabled - Yellow Solid
        Autonomous enabled - Purple Scrolling
        Coral in robot, incorrect position - Red Flashing
        Coral in robot, correct position - Green Solid
        Endgame (Last 20 seconds) - Blue Scrolling (for 3 seconds, overrides other conditions), then Blue Solid (doesn't override)
        Climb in deployed position - Blue Flashing
        */

        if (!DriverStation.isDSAttached()) {
            patternYellowRedScroll.applyTo(BufferLeft);
            patternYellowRedScroll.applyTo(BufferRight);
        } else if (DriverStation.isAutonomousEnabled()) {
            patternPurpleScroll.applyTo(BufferLeft);
            patternPurpleScroll.applyTo(BufferRight);
        } else if (this.flapSystemAndHang.hangState.equals("Deployed")) {
            patternBlueBreathe.applyTo(BufferLeft);
            patternBlueBreathe.applyTo(BufferRight);
        } else if (!this.timer.hasElapsed(3) && DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 0.01 && DriverStation.isTeleopEnabled()) {
            patternBlueScroll.applyTo(BufferLeft);
            patternBlueScroll.applyTo(BufferRight);
        } else if (!this.shooter.escalatorClear()) {
            patternRedBreathe.applyTo(BufferLeft);
            patternRedBreathe.applyTo(BufferRight);
        } else if (this.shooter.shooterHasCoral()) {
            patternGreenSolid.applyTo(BufferLeft);
            patternGreenSolid.applyTo(BufferRight);
        } else if (this.shooter.hasAlgae()) {
            patternBlueGreenBreathe.applyTo(BufferLeft);
            patternBlueGreenBreathe.applyTo(BufferRight);
        } else if (!this.isCoralMode) {
            patternBlueGreenScroll.applyTo(BufferLeft);
            patternBlueGreenScroll.applyTo(BufferRight);
        } else if (DriverStation.getMatchTime() < 20 && DriverStation.getMatchTime() > 0.01) {
            patternBlueSolid.applyTo(BufferLeft);
            patternBlueSolid.applyTo(BufferRight);
        } else if (DriverStation.isDisabled()) {
            patternYellowScroll.applyTo(BufferLeft);
            patternYellowScroll.applyTo(BufferRight);
        } else if (DriverStation.isTeleopEnabled()) {
            patternYellowSolid.applyTo(BufferLeft);
            patternYellowSolid.applyTo(BufferRight);
        }
        LED.setData(Buffer);
    }
}
