//**********FORWARD AND REVERSE DRIVING**************        

    public void driveInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        int ticks = inchesToTicks(inches);


        if (inches > 0) { // inches is greater than zero; positive
            
            velocityState = true;               // flag for setting the start velocity when doing the deceleration PID

            while ((ticks > rightFront.getCurrentPosition())&& opModeIsActive()) {
                
                
                
                
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = robotAngle- targetAngle ;
                remainingticks = ticks - rightFront.getCurrentPosition();
    
   // Code to set up the ability to measure current velocity  
                
        newTicks = rightFront.getCurrentPosition();
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
   
 
   
 // For short moves, under 20 inches, at low power
 
        if (ticks <= 1000){
            
          pfactor = (power/ ticks);
                pwr = ((remainingticks * pfactor) + .2);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher speeds       
        else {
                if (remainingticks >1500){   // When the robot is in the middle of long move, give full power
                 pwr = power +.2;   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }


                targetVelocity = (remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                pwr = ((currentVelocity - targetVelocity)* -.2) +.20;   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
// This is the acceleration PID method.  At applied power levels greater than .3, the wheels tend 
// to spin if you apply the power immediately.  This method applies a ramping up of power from 0 
// ticks to 1000 ticks.  The initial applied power starts at .25 and ramps up to the target power.
// 


                if ((rightFront.getCurrentPosition() < 1000) && (power > .3)) {
                        pfactor = (power/ 1000);
                        pwr = ((rightFront.getCurrentPosition() * pfactor) + .25);
                 
                        if (pwr > (power+.15)){ // the above code has the possibility of applying 
                                                // more than the target power, so this section 
                                                // limits the applied power to the target power
                   
                          pwr = power +.15;  
                  
                        }
                 
                }       
 
        }
 
                
   //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
                leftpwr = pwr;
                rightpwr = pwr;
                                     
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        
                        leftpwr = pwr + (off_angle * 0.025+0);
                        rightpwr = pwr - (off_angle * 0.025 + 0);
                        
                           
                        
                      
 
                }    
               
         
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
     


            }
            
            setPowerZero();
         

        }

     if (inches < 0) { //inches is less than zero 


            velocityState = true;               // flag for setting the start velocity when doing the deceleration PID
      
            while ((ticks < rightFront.getCurrentPosition())&& opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = robotAngle-targetAngle  ;              
                remainingticks = ticks - rightFront.getCurrentPosition();
               
         newTicks = rightFront.getCurrentPosition();
        newTime = runtime.milliseconds();
        deltaTicks = oldTicks-newTicks;
        deltaTime = oldTime -newTime;
        currentVelocity = (deltaTicks/deltaTime);
        oldTime = newTime;
        oldTicks = newTicks;
   
 
   
 // For short moves, under 20 inches, at low power. When you are moving slow and the distance is short,
 // the robot does not have a lot of inertia.  Therefore, we don't need to apply negative power in the
 // PID function to apply braking.  This PID is simply a proportional function with a small amount 
 // of feedforward.
 
        if (ticks >= -1000){
            
          pfactor = (power/ ticks);
                pwr = ((-remainingticks * pfactor) - .25);   
            
            
        }
        
 // This section is for longer moves, greater than 20 inches at higher power.  For these moves, the 
 // robot has a lot of inertia, and the PID method needs to apply reverse power to apply braking versus
 // simply decreasing applied power.  To accomplish this, the PID routine measures the velocity of the 
 // robot, calculates the target velocity, then applies power to have the robot follow the velocity target.
 
 
        else {
                if (remainingticks < -1500){   // When the robot is in the middle of long move, give full power
                 pwr = -power -.25;   
                } 

                else{            // This section sets up the PID deceleration at end of long move, last 15 inches
 
                 if(velocityState){                     // This flag is TRUE when the routine starts
                startVelocity = currentVelocity;        // Capture the current velocity at beginning of deceleration
                velocityState = false;                  // set the flag to FALSE so the routine won't capture current velocity again
                 }

                targetVelocity = -(remainingticks/1500)*startVelocity;   // As robot moves in final 1500 ticks, we first determine the ideal velocity 
                                                                        // at the current number of final ticks
           
                pwr = ((currentVelocity - targetVelocity)* -.2) -.25;   // The power applied in final deceleration is dependent on the current velocity.
                                                                        // If the robot is going faster than it should, then the applied power is in UnsupportedClassVersionError                                           
                                                                        // If robot is going slower than ideal, then power is applied in forward direction

                }
                  
                if ((rightFront.getCurrentPosition() > -1000) && (-power < -.3)) {
                        pfactor = (-power/ -1000);
                        pwr = ((rightFront.getCurrentPosition() * pfactor) - .25);
                 
                        if (pwr < (-power-.15)){
                   
                          pwr = -power -.15;  
                  
                        }
                 
                }       
 
        }
 
  
  
  
         telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightFront.getCurrentPosition());
            telemetry.addData("pfactor:", pfactor);          
 
            telemetry.update();

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.5 && off_angle > -0.5) {
                leftpwr = pwr;
                rightpwr = pwr;
             
             }
            
  //    Turn robot back to heading if the current heading is off from desired heading          
                if (off_angle >= 0.5 || off_angle <= -0.5) {
                        
                        leftpwr = pwr + (off_angle * 0.03 + 0);
                        rightpwr = pwr - (off_angle * 0.03 + 0);
                }    
                leftFront.setPower(leftpwr);
                rightFront.setPower(rightpwr);
                leftBack.setPower(leftpwr);
                rightBack.setPower(rightpwr);
            }
            
           setPowerZero();

        }
    
    }
        