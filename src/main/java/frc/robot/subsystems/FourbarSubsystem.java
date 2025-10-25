package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class FourbarSubsystem {
     private final DoubleSolenoid m_doubleSolenoid;
     private final Compressor m_compressor;

     public FourbarSubsystem() {
         m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2); //Set later
         m_compressor = new Compressor(PneumaticsModuleType.REVPH);
     }

     public void setSolenoid(DoubleSolenoid.Value setValue) {
         m_doubleSolenoid.set(setValue);
     }
    
     public void toggleSolenid() {
         m_doubleSolenoid.toggle();
     }

}