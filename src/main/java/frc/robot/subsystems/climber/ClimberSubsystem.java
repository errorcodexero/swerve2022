package frc.robot.subsystems.climber;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class ClimberSubsystem extends MotorEncoderSubsystem {

    public ClimberSubsystem(Subsystem parent) throws Exception {
        super(parent, "climber", false);
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("position")) {
            v = new SettingsValue(getPosition()) ;
        }

        return v ;
    }
}