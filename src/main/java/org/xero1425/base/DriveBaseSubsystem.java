package org.xero1425.base;

import org.xero1425.base.gyro.NavxGyro;
import org.xero1425.base.gyro.RomiGyro;
import org.xero1425.base.gyro.XeroGyro;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public abstract class DriveBaseSubsystem extends Subsystem {
    private XeroGyro gyro_;

    public DriveBaseSubsystem(Subsystem parent, String name)
            throws BadParameterTypeException, MissingParameterException {
        super(parent, name);

        MessageLogger logger = getRobot().getMessageLogger();

        String gyrotype = getRobot().getSettingsSupplier().get("hw:gyro").getString();
        if (gyrotype.equals("navx")) {
            gyro_ = new NavxGyro();
        } else if (gyrotype.equals("LSM6DS33")) {
            gyro_ = new RomiGyro();
        }

        double start = getRobot().getTime();
        while (getRobot().getTime() - start < 3.0) {
            if (gyro().isConnected())
                break;
        }

        if (!gyro_.isConnected()) {
            logger.startMessage(MessageType.Error);
            logger.add("NavX is not connected - cannot perform tankdrive path following functions");
            logger.endMessage();
            gyro_ = null;
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

    protected boolean hasGyro() {
        return gyro_ != null ;
    }

    protected XeroGyro gyro() {
        return gyro_ ;
    }
}
