package frc.robot.oi ;

import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.OISubsystem;

public class Swerve2021OISubsystem extends OISubsystem {
    public final static String SubsystemName = "swerveoi";

    public Swerve2021OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db) ;
    }
}
