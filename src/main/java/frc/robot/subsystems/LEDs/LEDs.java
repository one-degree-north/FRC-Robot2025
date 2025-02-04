package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RegularConstants.LEDConstants;
import frc.robot.subsystems.AlgaeRollers.AlgaeInnerRollers;
import frc.robot.subsystems.CoralRollers.CoralRoller;
import frc.robot.subsystems.Superstructure.AlgaePivot;
import frc.robot.subsystems.Superstructure.CoralWrist;

public class LEDs extends SubsystemBase {
    private final AddressableLED m_leds;
    private static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.ledLength);
    private Alliance alliance = null;
    private LEDStates currentState = LEDStates.AUTON;

    private final AlgaePivot algaeMech;
    
    private static final double BREATH_DURATION = 1.0;
    private static final double WAVE_EXPONENT = 0.4;
    private static final double WAVE_CYCLE_LENGTH = 30.0;
    private static final double WAVE_CYCLE_DURATION = 0.25;

    public LEDs(AlgaePivot algaeMech) {
        m_leds = new AddressableLED(LEDConstants.ledID);
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.setData(m_ledBuffer);
        m_leds.start();
        this.algaeMech = algaeMech;
    }

    private LEDStates determineState() {
        if (!DriverStation.isFMSAttached() && !DriverStation.isDSAttached()) {
            return currentState;
        }
        if (DriverStation.isAutonomous()) return LEDStates.AUTON;
        boolean algaeIntaked = AlgaeInnerRollers.isAlgaeIntaked();
        boolean coralIntaked = CoralRoller.isCoralIntaked;

        if (algaeIntaked && coralIntaked) return LEDStates.BOTHINTAKED;
        if (algaeIntaked) return LEDStates.ALGAEINTAKED;
        if (coralIntaked) return LEDStates.CORALINTAKED;
        return LEDStates.IDLE;
    }

    private void handleLEDTransition() {
        switch (currentState) {
            case CORALINTAKED -> breath(LEDSections.FULL, Color.kGhostWhite, Color.kBlack, BREATH_DURATION);
            case ALGAEINTAKED -> breath(LEDSections.FULL, Color.kSeaGreen, Color.kWhite, BREATH_DURATION);
            case IDLE -> wave(LEDSections.FULL, Color.kAliceBlue, Color.kBlueViolet, WAVE_CYCLE_LENGTH, WAVE_CYCLE_DURATION, false);
            case BOTHINTAKED -> rainbow(LEDSections.FULL, WAVE_CYCLE_LENGTH, WAVE_CYCLE_DURATION);
            case AUTON -> {
                Color autonColor = (alliance == Alliance.Red) ? Color.kRed : Color.kBlue;
                solid(LEDSections.FULL, autonColor);
            }
        }
    }

    private void rainbow(LEDSections section, double cycleLength, double duration) {
        double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
        double xDiffPerLed = 180.0 / cycleLength;
        for (int i = section.start(); i < section.end(); i++) {
            m_ledBuffer.setHSV(i, (int) ((x + i * xDiffPerLed) % 180.0), 255, 255);
        }
    }

    private void wave(LEDSections section, Color c1, Color c2, double cycleLength, double duration, boolean reverse) {
        double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
        double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
        for (int i = section.start(); i < section.end(); i++) {
            double ratio = calculateWaveRatio(x + i * xDiffPerLed);
            m_ledBuffer.setLED(i, interpolateColor(c1, c2, ratio));
        }
    }

    private double calculateWaveRatio(double x) {
        double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
        return Double.isNaN(ratio) ? 0.5 : ratio;
    }

    private Color interpolateColor(Color c1, Color c2, double ratio) {
        double red = c1.red * (1 - ratio) + c2.red * ratio;
        double green = c1.green * (1 - ratio) + c2.green * ratio;
        double blue = c1.blue * (1 - ratio) + c2.blue * ratio;
        return new Color(red, green, blue);
    }

    private void solid(LEDSections section, Color color) {
        for (int i = section.start(); i < section.end(); i++) {
            m_ledBuffer.setLED(i, color);
        }
    }

    private void breath(LEDSections section, Color c1, Color c2, double duration) {
        double x = (Timer.getFPGATimestamp() % duration) / duration * 2.0 * Math.PI;
        double ratio = (Math.sin(x) + 1.0) / 2.0;
        solid(section, interpolateColor(c1, c2, ratio));
    }

    @Override
    public void periodic() {
        if (DriverStation.isFMSAttached() || DriverStation.isDSAttached()) {
            alliance = DriverStation.getAlliance().get();
        }
        currentState = determineState();
        handleLEDTransition();
    }

    public enum LEDStates {
        CORALINTAKED, ALGAEINTAKED, BOTHINTAKED, IDLE, AUTON
    }

    private enum LEDSections {
        FULL(0, LEDConstants.ledLength);

        private final int start, end;

        LEDSections(int start, int end) {
            this.start = start;
            this.end = end;
        }

        public int start() {
            return start;
        }

        public int end() {
            return end;
        }
    }
}
