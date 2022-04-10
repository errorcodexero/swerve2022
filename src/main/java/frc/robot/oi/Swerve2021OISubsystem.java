package frc.robot.oi ;

import org.xero1425.base.Subsystem;
import org.xero1425.base.oi.Gamepad;
import org.xero1425.base.oi.OISubsystem;
import org.xero1425.base.oi.SwerveDriveGamepad;
import org.xero1425.base.swervedrive.SwerveDriveSubsystem;

public class Swerve2021OISubsystem extends OISubsystem {
    private SwerveDriveSubsystem db_ ;

    public final static String SubsystemName = "swerveoi";
    
    private final static String DriverGamepadHIDIndexName = "hw:driverstation:hid:driver" ;
    private final static String[] devices_ = { "driver" } ;

    public Swerve2021OISubsystem(Subsystem parent, SwerveDriveSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db) ;
        
        db_ = db ;
    }    


    public Gamepad createDevice(String name) throws Exception {
        Gamepad ret = null ;

        if (!name.equals(devices_[0]))
        {
            throw new Exception("OI HIDdevice '" + name + "' is not a valid device") ;            
        }

        //
        // This can throw an exception and it will be caught by the caller.  This type of error means
        // a permanent error that needs to be fixed.  For instance, if the index is missing from the params
        // file
        //
        int index = getSettingsValue(DriverGamepadHIDIndexName).getInteger() ;

        try {
            ret = new SwerveDriveGamepad(this, index, db_) ;
        }
        catch(Exception ex) {
            //
            // The exception during creation of the device may just mean that the driver station
            // has not connected to the robot yet.  We just return null so that the calling class
            // keeps trying
            //
            ret = null ;
        }

        return ret ;
    }
}
