package org.xero1425.misc ;

import org.junit.*;

public class EncoderMapperUnitTest
{
    @Before
    public void init() {
    }

    @Test
    public void testRanseurTubArm() {
        EncoderMapper map = new EncoderMapper(180.0, -180.0, 5.0, 0.0) ;
        map.calibrate(90.0, 2.5) ;
        org.junit.Assert.assertEquals(map.toRobot(4.5), -126.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(3.5), 162.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(2.5), 90.0, 1e-6) ;    
        
        org.junit.Assert.assertEquals(map.toEncoder(-162), 4.0, 1e-6) ;   
        org.junit.Assert.assertEquals(map.toEncoder(54), 2.0, 1e-6) ; 
        org.junit.Assert.assertEquals(map.toEncoder(90), 2.5, 1e-6) ;                      
    }

    @Test
    public void testSimpleLinear() {
        EncoderMapper map = new EncoderMapper(100.0, 0.0, 5.0, 0.0) ;
        org.junit.Assert.assertEquals(map.toRobot(0.01), 0.2, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(-0.01), 0.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(2.5), 50.0, 1e-6) ;    
        org.junit.Assert.assertEquals(map.toRobot(4.99), 99.8, 1e-6) ;  

        org.junit.Assert.assertEquals(map.toEncoder(0), 0.0, 1e-6) ;   
        org.junit.Assert.assertEquals(map.toEncoder(50), 2.5, 1e-6) ; 
        org.junit.Assert.assertEquals(map.toEncoder(100), 5.0, 1e-6) ;                      
    }   
    
    @Test
    public void testSimpleLinearWrapped() {
        EncoderMapper map = new EncoderMapper(100.0, 0.0, 5.0, 0.0) ;
        map.calibrate(0.0, 3.0) ;
        org.junit.Assert.assertEquals(map.toRobot(0.0), 40.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(-0.01), 40.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(2.5), 90.0, 1e-6) ;    
        org.junit.Assert.assertEquals(map.toRobot(2.0), 80.0, 1e-6) ;  
        org.junit.Assert.assertEquals(map.toRobot(4.0), 20.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(5.0), 40.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(5.01), 40.0, 1e-6) ;    
        org.junit.Assert.assertEquals(map.toRobot(2.99), 99.8, 1e-6) ;         
        org.junit.Assert.assertEquals(map.toRobot(3.01), 0.2, 1e-6) ;   

        org.junit.Assert.assertEquals(map.toEncoder(80), 2.0, 1e-6) ;   
        org.junit.Assert.assertEquals(map.toEncoder(20), 4.0, 1e-6) ; 
    }   
    
    @Test
    public void testLinearFlipped() {
        EncoderMapper map = new EncoderMapper(100.0, 0.0, 0.0, 5.0) ;
        map.calibrate(50.0, 2.5) ;
        org.junit.Assert.assertEquals(map.toRobot(0.0), 100.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(-0.01), 100.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(2.5), 50.0, 1e-6) ;    
        org.junit.Assert.assertEquals(map.toRobot(4.9999999999), 0.0, 1e-6) ;  
        org.junit.Assert.assertEquals(map.toRobot(1.75), 65.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(3.25), 35.0, 1e-6) ;


        org.junit.Assert.assertEquals(map.toEncoder(50), 2.5, 1e-6) ;   
        org.junit.Assert.assertEquals(map.toEncoder(35), 3.25, 1e-6) ; 
    }

    @Test
    public void testLinearFlippedWrapped() {
        EncoderMapper map = new EncoderMapper(100.0, 0.0, 0.0, 5.0) ;
        map.calibrate(90.0, 2.5) ;
        org.junit.Assert.assertEquals(map.toRobot(0.0), 40.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(-0.01), 40.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(2.5), 90.0, 1e-6) ;    
        org.junit.Assert.assertEquals(map.toRobot(2.0), 0.0, 1e-6) ;  
        org.junit.Assert.assertEquals(map.toRobot(1.25), 15.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(3.75), 65.0, 1e-6) ;
        org.junit.Assert.assertEquals(map.toRobot(4.0), 60.0, 1e-6) ;

        org.junit.Assert.assertEquals(map.toEncoder(20), 1.0, 1e-6) ;   
        org.junit.Assert.assertEquals(map.toEncoder(80), 3.0, 1e-6) ; 
    }       
}