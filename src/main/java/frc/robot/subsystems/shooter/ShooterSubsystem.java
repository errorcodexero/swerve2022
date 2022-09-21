package frc.robot.subsystems.shooter;

import org.xero1425.base.motors.MotorController;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class ShooterSubsystem extends Subsystem {
    private MotorEncoderSubsystem wheelSubsystem_;
    private HoodMotorSubsystem hoodSubsystem_;
    public static final String SubsystemName = "shooter";

    public ShooterSubsystem(Subsystem parent) throws Exception {
        super(parent, SubsystemName);

        wheelSubsystem_ = new MotorEncoderSubsystem(this, SubsystemName + "-wheel", false, 8, true);
        addChild(wheelSubsystem_) ;

        if (isSettingDefined("ramprate")) {
            MotorController ctrl = wheelSubsystem_.getMotorController() ;
            double rate = getSettingsValue("ramprate").getDouble() ;
            ctrl.setOpenLoopRampRate(rate);
        }
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
    public void run() {
        putDashboard("shvel", DisplayType.Always, wheelSubsystem_.getVelocity());
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