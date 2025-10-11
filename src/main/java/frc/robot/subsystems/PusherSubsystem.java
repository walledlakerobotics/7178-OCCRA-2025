package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PusherSubsystem {
    private final DoubleSolenoid m_doubleSolenoid;
    private final Compressor m_compressor;

    public PusherSubsystem() {
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2); //Set later
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    }

    public void setSolenoid(DoubleSolenoid.Value setValue) {
        m_doubleSolenoid.set(setValue);
    }

    public void dejectBlock() {
        setSolenoid(Value.kForward);
        try{
            Thread.sleep(500);
        }catch(InterruptedException e) {
            /* Do Nothing */
        }
        setSolenoid(Value.kReverse);
    }
}
