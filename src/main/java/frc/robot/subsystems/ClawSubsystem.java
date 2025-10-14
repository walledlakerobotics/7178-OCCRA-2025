package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClawSubsystem {
    private final DoubleSolenoid m_rotateForwardDoubleSolenoid;
    private final DoubleSolenoid m_rotateBackwardDoubleSolenoid;
    private final DoubleSolenoid m_clampDoubleSolenoid;
    private final Compressor m_compressor;

    public ClawSubsystem() {
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);
        m_rotateForwardDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
        m_rotateBackwardDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
        m_clampDoubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);
        //set later
    }

    public void rotateForward() {
        m_rotateBackwardDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_rotateForwardDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void rotateBackward() {
        m_rotateBackwardDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        m_rotateForwardDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void clampToggle() {
        m_clampDoubleSolenoid.toggle();
    }
}