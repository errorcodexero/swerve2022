package org.xero1425.base.gyro;

import com.kauailabs.navx.frc.AHRS ;
import edu.wpi.first.wpilibj.SPI;

/// \file

/// \brief This class implements the XeroGyro interface for the NavX gyro
public class NavxGyro implements XeroGyro {

    // The instance of the NavX Gyro
    private AHRS navx_ ;

    private double offset_ ;

    /// \brief Create the Gyro class using the default NavX Port on the MXP bus
    public NavxGyro() {
        navx_ = new AHRS(SPI.Port.kMXP) ;
    }

    /// \brief Create the Gyro class using the supplied SPI port
    /// \param port the SPI port to use to talk to the NavX hardware
    public NavxGyro(SPI.Port port) {
        navx_ = new AHRS(port) ;
    }

    public void reset() {
        offset_ = -navx_.getYaw() ;
    }

    /// \brief Returns true if the NavX is connected
    /// \returns true if the NavX is connected, othewise false
    public boolean isConnected() {
        return navx_.isConnected() ;
    }

    /// \brief Returns the current effective YAW angle for the NavX.  This value will always be between
    /// -180 degrees and 180 degrees.
    /// \returns the current effective YAW angle for the NavX
    public double getYaw() {
        double ret = 0.0 ;

        ret = -navx_.getYaw() - offset_ ;
        return ret ;
    }

    /// \brief Returns the total angle for the NavX
    /// \returns the total angle for the NavX    
    public double getAngle() {
        return navx_.getAngle() ;
    }

    public double getGyroX() {
        return navx_.getRawGyroX() ;
    }
    public double getGyroY() {
        return navx_.getRawGyroY() ;
    }

    public double getGyroZ() {
        return navx_.getRawGyroZ() ;
    }
    
    public double getAccelX() {
        return navx_.getRawAccelX() ;
    }

    public double getAccelY() {
        return navx_.getRawAccelY() ;
    }

    public double getAccelZ() {
        return navx_.getRawAccelZ() ;
    }
}
