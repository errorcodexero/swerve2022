package frc.robot.subsystems.oi ;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Swerve2022RobotSubsystem;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class Swerve2022OISubsystem extends OISubsystem {
    private Swerve2022OIDevice oipanel_ ;

    public final static String SubsystemName = "swerve2022oi";
    private final static String OIHIDIndexName = "panel:index";

    public Swerve2022OISubsystem(Subsystem parent, DriveBaseSubsystem db) {
        super(parent, SubsystemName, GamePadType.Swerve, db) ;

        int index ;
        MessageLogger logger = getRobot().getMessageLogger() ;

        //
        // Add the custom OI for zeke to the OI subsystem
        //
        try {
            index = getSettingsValue(OIHIDIndexName).getInteger() ;
        } catch (BadParameterTypeException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" exists, but is not an integer").endMessage();
            index = -1 ;
        } catch (MissingParameterException e) {
            logger.startMessage(MessageType.Error) ;
            logger.add("parameter ").addQuoted(OIHIDIndexName) ;
            logger.add(" does not exist").endMessage();
            index = -1 ;            
        }

        if (index != -1) {
            try {
                oipanel_ = new Swerve2022OIDevice(this, "OI", index) ;
                addHIDDevice(oipanel_) ;
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error) ;
                logger.add("OI HID device was not created - ") ;
                logger.add(ex.getMessage()).endMessage(); ;
            }
        }        
    }

    public void run() throws  Exception {
        super.run();

        Gamepad gamepad = getGamePad();
        if (gamepad.isBPressed() && gamepad.isYPressed()) {
            Swerve2022RobotSubsystem robotSubsystem = (Swerve2022RobotSubsystem) getRobot().getRobotSubsystem();
            robotSubsystem.getDB().setPose(new Pose2d()) ;
        }
    }
}
