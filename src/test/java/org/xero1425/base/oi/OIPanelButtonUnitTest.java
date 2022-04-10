package org.xero1425.base.oi ;

import org.junit.*;

public class OIPanelButtonUnitTest
{
    @Before
    public void init() {
    }

    @Test
    public void testOILevelButton() {
        OIPanelButton button = new OIPanelButton(13, OIPanelButton.ButtonType.Level) ;

        Assert.assertEquals(button.getItemNumber(), 13) ;
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 1) ;
        Assert.assertEquals(button.getValue(), 1) ;
        
        button.setButtonValue(false);        
        Assert.assertEquals(button.getValue(), 0) ;
        Assert.assertEquals(button.getValue(), 0) ;        
    }

    @Test
    public void testOIInvLevelButton() {
        OIPanelButton button = new OIPanelButton(11, OIPanelButton.ButtonType.LevelInv) ;

        Assert.assertEquals(button.getItemNumber(), 11) ;
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 0) ;
        Assert.assertEquals(button.getValue(), 0) ;
        
        button.setButtonValue(false);        
        Assert.assertEquals(button.getValue(), 1) ;
        Assert.assertEquals(button.getValue(), 1) ;        
    }    

    @Test
    public void testOILowToHighButton() {
        OIPanelButton button = new OIPanelButton(8, OIPanelButton.ButtonType.LowToHigh) ;

        Assert.assertEquals(button.getItemNumber(), 8) ;
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(false);        
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 1) ;       
        Assert.assertEquals(button.getValue(), 1) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 0) ;       
        Assert.assertEquals(button.getValue(), 0) ;        
        
        button.setButtonValue(false);        
        Assert.assertEquals(button.getValue(), 0) ;
        Assert.assertEquals(button.getValue(), 0) ;   
        
        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 1) ;       
        Assert.assertEquals(button.getValue(), 1) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 0) ;       
        Assert.assertEquals(button.getValue(), 0) ;          
    }    

    @Test
    public void testOIHighToLowButton() {
        OIPanelButton button = new OIPanelButton(8, OIPanelButton.ButtonType.HighToLow) ;

        Assert.assertEquals(button.getItemNumber(), 8) ;
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(false);
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 0) ;       
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(false);
        Assert.assertEquals(button.getValue(), 1) ;       
        Assert.assertEquals(button.getValue(), 1) ;        
        
        button.setButtonValue(false);        
        Assert.assertEquals(button.getValue(), 0) ;
        Assert.assertEquals(button.getValue(), 0) ;   
        
        button.setButtonValue(true);
        Assert.assertEquals(button.getValue(), 0) ;       
        Assert.assertEquals(button.getValue(), 0) ;

        button.setButtonValue(false);
        Assert.assertEquals(button.getValue(), 1) ;
        Assert.assertEquals(button.getValue(), 1) ;    
        
        button.setButtonValue(false);
        Assert.assertEquals(button.getValue(), 0) ;
        Assert.assertEquals(button.getValue(), 0) ;           
    }     

} ;