package frc.robot.oi ;

import org.xero1425.base.DriveBaseSubsystem;
import org.xero1425.base.Subsystem;
import org.xero1425.base.oi.OISubsystem;

public class Swerve2021OISubsystem extends OISubsystem {
    public final static String SubsystemName = "swerveoi";

    public Swerve2021OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db) ;
    }
}
