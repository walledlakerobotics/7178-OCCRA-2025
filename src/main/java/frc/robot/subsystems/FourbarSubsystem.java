package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class FourbarSubsystem extends SubsystemBase{
     private final Solenoid m_Solenoid;

     public FourbarSubsystem() {
         m_Solenoid = new Solenoid(
                PneumaticsConstants.kPCMID, 
                PneumaticsModuleType.CTREPCM, 
                PneumaticsConstants.kFourBarChannel);
    }
    
    public Command toggle() {
        return runOnce( () -> {
            m_Solenoid.toggle(); 
    });
    }

}