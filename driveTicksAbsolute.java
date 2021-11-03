    public void driveTicksAbsolute(int targetXfieldPosition , int targetYfieldPosition,double xpower, double ypower,double targetAngle) {
      
      if (rot180){
          
        rot180XcompFactor=400;   
        rot180YcompFactor=1700;   
      }
      
        
        xticks = targetXfieldPosition - (actualXposition-rot180XcompFactor); 
        yticks = targetYfieldPosition - (actualYposition-rot180YcompFactor);       
    
        xMoveDone =false;
        yMoveDone =false;
        
        // We need a flag to be able to apply power in the correct direction
        // so we use a variable to change polarity
        
        if (xticks<0){
            
            xDirection = -1;
        }
        else xDirection = 1;
        
         if (yticks<0){
            
            yDirection = -1;
        }
        else yDirection = 1;
        
    // For either straight forward back or strafe moves when the X or Y axis 
    // is zero, we need to set the move in that direction to zero to start 
    // the loop
        
        
        if (xticks==0){
          
           xMoveDone =true;  
            
        }
      
        
         if (yticks==0){
            
           yMoveDone =true; 
        }
   
        
            
            velocityState = true;  // flag for setting the start velocity when doing the deceleration PID
            velocityStateY = true;  


            while ((!xMoveDone ||!yMoveDone) && opModeIsActive()) {
                
            if ((xticks* xDirection) < ((rightFront.getCurrentPosition()- actualXposition)* xDirection)){
                xMoveDone = true;
            }    
            if ((yticks * yDirection) < ((-rightBack.getCurrentPosition()- actualYposition)* yDirection)){
                yMoveDone = true;
            }    
                
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                  }
                off_angle = robotAngle- targetAngle ;
                remainingticks = (xticks * xDirection) - ((rightFront.getCurrentPosition()- actualXposition)* xDirection);
                remainingYticks = (yticks * yDirection) -  ((-rightBack.getCurrentPosition()- actualYposition)* yDirection);
    
   // Code to set up the ability to measure current velocity  
                
        newTicks = ((rightFront.getCurrentPosition()- actualXposition)* xDirection);
        newYTicks = ((-rightBack.getCurrentPosition()- actualYposition)* yDirection);
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaYTicks = oldYTicks-newYTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        currentYvelocity = (deltaYTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
        oldYTicks = newYTicks;
        

 
  if (!xMoveDone){
   
 // For short moves, under 10 inches
 
        if ((xticks * xDirection)  <= 1000){
            
          pfactor = (xpower/ (xticks * xDirection));
                xpwr = (((remainingticks * pfactor) + .2)* xDirection);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingticks >1500){   // When the robot is in the middle of long move, give full power
                 xpwr = ((xpower+.2) * xDirection);   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetVelocity = (remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                xpwr = ((((currentVelocity - targetVelocity)* -.25) +.20)* xDirection);   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if ((((rightFront.getCurrentPosition()- actualXposition)* xDirection) < 500) && (xpower > .3)) {
                        pfactor = (xpower/ 500);
                        xpwr = (((((rightFront.getCurrentPosition()- actualXposition)* xDirection) * pfactor) + .25)* xDirection);
                 
                        if ((xpwr* xDirection) > xpower){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          xpwr = (xpower* xDirection);  
                  
                        }
                 
                }       
 
        }
        
  }
 
   if (!yMoveDone){ 
   
        // For short moves, under 10 inches
 
        if ((yticks * yDirection) <= 500){
            
          pfactor = (ypower/ (yticks * yDirection));
                ypwr = (((remainingYticks * pfactor) + .2) * yDirection);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingYticks >500){   // When the robot is in the middle of long move, give full power
                 ypwr = ((ypower +.2) * yDirection);   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityStateY){                     // This flag is TRUE when the routine starts
                startYvelocity = currentYvelocity;        // Capture the current velocity at beginning of deceleration
                velocityStateY = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetYvelocity = (remainingYticks/500)*startYvelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                ypwr = ((((currentYvelocity - targetYvelocity)* -.2) +.25) * yDirection);   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if ((((-rightBack.getCurrentPosition()- actualYposition) * yDirection) < 500) && (ypower > .3)) {
                        pfactor = (ypower/ 500);
                        ypwr = ((((-rightBack.getCurrentPosition()- actualYposition) * yDirection) * pfactor) + .25);
                 
                        if (ypwr > ((ypower+.15) * yDirection)){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          ypwr = ((ypower +.15) * yDirection);  
                  
                        }
                 
                }       
 
        } 
        
   }    
        
   if (xMoveDone){

        currentXposition=  ((rightFront.getCurrentPosition()- actualXposition)* xDirection);
    
        if (currentXposition < ((xticks +25)* xDirection) && currentXposition > ((xticks -25)* xDirection)) {
            xpwr =0;
        }
            
        if (currentXposition >= ((xticks +25)* xDirection) || currentXposition <= ((xticks -25)* xDirection)){    
            xpwr = ((((xticks* xDirection) - currentXposition) * .0005)* xDirection);    
        }
    }
 
    if (yMoveDone){

        currentYposition=  ((-rightBack.getCurrentPosition()- actualYposition) * yDirection);
    
        if (currentYposition < ((yticks +25) * yDirection) && currentYposition > ((yticks -25) * yDirection)) {
            ypwr =0;
        }
            
        if (currentYposition >= ((yticks +25) * yDirection) || currentYposition <= ((yticks -25) * yDirection)){    
            ypwr = ((((yticks * yDirection) - currentYposition)  * .001) * yDirection);    
                      
        }
     
 }      
        
        
        
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
            
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = (off_angle * 0.015 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = (off_angle * 0.015 + 0);
            
                        }
                      
                }
               
              double r = Math.hypot(ypwr, xpwr);
                double robotAngle = Math.atan2(xpwr, ypwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle)+ rotation  ;
                double v2 = r * Math.sin(robotAngle)- rotation ;
                double v3 = r * Math.sin(robotAngle) + rotation ;
                double v4 = r * Math.cos(robotAngle) - rotation ;

                leftFront.setPower(v1/.7);
                rightFront.setPower(v2/.7);
                leftBack.setPower(v3/.7);
                rightBack.setPower(v4/.7);

                 newTime = runtime.milliseconds();
            
               telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", xpwr);
          telemetry.addData("ypwr:", ypwr);        

           telemetry.addData("current position:", rightFront.getCurrentPosition());
            telemetry.addData("pfactor:", pfactor);          
            
            telemetry.update();
                
            }
            
            setPowerZero();
            sleep(200);
            
            actualXposition =rightFront.getCurrentPosition();
            actualYposition = -rightBack.getCurrentPosition();
    }
