package frc.robot.subsystems.turret;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class TurretSubsystem extends MotorEncoderSubsystem {
    
    private boolean is_ready_to_fire_ ;
    
    public final static String SubsystemName = "turret" ;

    public TurretSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName, false) ;

        is_ready_to_fire_ = false ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("angle")) {
            v = new SettingsValue(getPosition()) ;
        }

        return v ;
    }

    public boolean isReadyToFire() {
        return is_ready_to_fire_ ;
    }

    public void setReadyToFire(boolean b) {
        is_ready_to_fire_ = b ;
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();
    }

    @Override
    protected double limitPower(double p) {
        return p ;
    }
}

