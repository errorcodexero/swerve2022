package frc.robot.automodes;

import java.util.List;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class SwerveDriveRobotAutoController extends AutoController {

    private SwerveTestAutoMode test_mode_ ;

    public SwerveDriveRobotAutoController(XeroRobot robot) throws MissingParameterException, BadParameterTypeException, InvalidActionRequest {
        super(robot, "SwerveDriveAutoController");

        try {
            test_mode_ = new SwerveTestAutoMode(this) ;

            addAutoMode(new FourBallAuto(this)) ;
            addAutoMode(new TwoBallLeftAuto(this)) ;
            addAutoMode(new TwoBallRightAuto(this)) ;
        }
        catch(Exception ex) {
            MessageLogger logger = robot.getMessageLogger() ;
            logger.startMessage(MessageType.Error).add("Exception thrown creating automodes - ") ;
            logger.add(ex.getMessage()).endMessage();
            robot.logStackTrace(ex.getStackTrace());
        }
    }

    public void updateAutoMode(int mode, String gamedata) {

        AutoMode modeobj = null ;

        if (isTestMode()) {
            modeobj = test_mode_ ;
        }
        else {
            List<AutoMode> automodes = getAllAutomodes() ;
            if (mode >= 0 && mode < automodes.size()) {
                modeobj = automodes.get(mode) ;
            }
        }

        if (getAutoMode() != modeobj) {
            setAutoMode(modeobj) ;
        }
    }
}
