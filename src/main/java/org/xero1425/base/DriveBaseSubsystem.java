package org.xero1425.base;

import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.RomiGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class DriveBaseSubsystem extends Subsystem {
    private XeroGyro gyro_;

    public DriveBaseSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name);

        MessageLogger logger = getRobot().getMessageLogger();

        String gyrotype = getSettingsValue("hw:gyro:type").getString();
        double startup = getSettingsValue("hw:gyro:start-time").getDouble() ;

        if (gyrotype.equals("navx")) {
            gyro_ = new NavxGyro();
        } else if (gyrotype.equals("LSM6DS33")) {
            gyro_ = new RomiGyro();
        }
        else {
            String msg = "the gyro type '" + gyrotype + "' is not valid.  Only 'navx' and 'LSM6D33' are supported" ;
            throw new Exception(msg) ;
        }

        double start = getRobot().getTime();
        while (getRobot().getTime() - start < startup) {
            if (gyro().isConnected())
                break;
        }

        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error);
            logger.add("Gyro is not connected");
            logger.endMessage();

            String msg = "cannot connect to the gyro of type '" + gyrotype + "' in the start time" ;
            throw new Exception(msg) ;
        }
    }

    public static Pose2d segmentToPose(XeroPathSegment seg) {
        return new Pose2d(seg.getX(), seg.getY(), Rotation2d.fromDegrees(seg.getHeading())) ;
    }

    /// \brief returns true to indicate this is a drivebase
    /// \returns true to indicate this is a drivebase
    public boolean isDB() {
        return true;
    }

    protected XeroGyro gyro() {
        return gyro_ ;
    }
}