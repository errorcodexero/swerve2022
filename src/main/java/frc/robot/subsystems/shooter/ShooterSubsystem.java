package frc.robot.subsystems.shooter;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;

public class ShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem wheelSubsystem_;
    private MotorEncoderSubsystem hoodSubsystem_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        double ticks_per_rev = 42 ;
        double seconds_per_minute = 60 ;
        double motor_to_shooter_gear_ratio  = 1.25 ;
        double factor =  seconds_per_minute / ticks_per_rev * motor_to_shooter_gear_ratio ;

        wheelSubsystem_ = new MotorEncoderSubsystem(this, SubsystemName + "-wheel", false, 8);
        addChild(wheelSubsystem_) ;
        wheelSubsystem_.setVelocityConversion(factor);

        hoodSubsystem_ = new HoodMotorSubsystem(this) ;
        addChild(hoodSubsystem_) ;
    }

    public MotorEncoderSubsystem getWheelSubsystem(){
        return wheelSubsystem_;
    }

    public MotorEncoderSubsystem getHoodSubsystem() {
        return hoodSubsystem_;
    }

    public void stop() {
        wheelSubsystem_.cancelAction();
        wheelSubsystem_.setPower(0.0) ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("wheel-velocity")) {
            v = new SettingsValue(wheelSubsystem_.getVelocity()) ;
        }

        return v ;
    }
}