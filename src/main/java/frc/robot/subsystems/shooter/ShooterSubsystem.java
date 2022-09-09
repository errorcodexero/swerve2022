package frc.robot.subsystems.shooter;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.SettingsValue;
import org.xero1425.base.motors.MotorGroupController;

public class ShooterSubsystem extends Subsystem{
    private MotorEncoderSubsystem wheelMotor_;
    private MotorEncoderSubsystem hoodMotor_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        double ticks_per_rev = 42 ;
        double seconds_per_minute = 60 ;
        double motor_to_shooter_gear_ratio  = 2 / 1 ;
        double factor =  seconds_per_minute / ticks_per_rev * motor_to_shooter_gear_ratio ;

        wheelMotor_ = new MotorEncoderSubsystem(this, SubsystemName + "-wheel", false, 8);
        addChild(wheelMotor_) ;
        wheelMotor_.setVelocityConversion(factor);

        hoodMotor_ = new HoodMotorSubsystem(this) ;
        addChild(hoodMotor_) ;
    }

    public MotorEncoderSubsystem getWheelSubsystem(){
        return wheelMotor_;
    }

    public MotorEncoderSubsystem getHoodSubsystem() {
        return hoodMotor_;
    }

    public void stop() {
        wheelMotor_.cancelAction();
        wheelMotor_.setPower(0.0) ;
    }

    @Override
    public SettingsValue getProperty(String name) {
        SettingsValue v = null ;

        if (name.equals("wheel-velocity")) {
            v = new SettingsValue(wheelMotor_.getVelocity()) ;
        }

        return v ;
    }
}