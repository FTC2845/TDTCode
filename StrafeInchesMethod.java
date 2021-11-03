   //***************LEFT RIGHT STRAFING******************


    public void strafeInches(double inches, double power, double targetAngle) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = inchesToTicks2(inches);
        ticks = ticks * -1;

    //      double exponentTicks =0;
    //    double exponentEncoder =0; 
    //     double accelThresholdTicks = (ticks * .67);
    //    double accelCoefficient = 0;

    double bias = 0;
        if (inches > 0) { //positive
            while ((ticks < rightBack.getCurrentPosition())&& opModeIsActive()) {
 
                 angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = targetAngle - robotAngle;
           remainingticks = ticks - rightBack.getCurrentPosition();
        //       double pwr;

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = -(off_angle * 0.02 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = -(off_angle * 0.02 + 0);
            
                        }
                      
                }
                    
             //   exponentTicks = Math.pow (ticks,5);
             //   exponentEncoder = Math.pow (rightBack.getCurrentPosition(),5);
             //   double currentInchPosition = (rightBack.getCurrentPosition()* .00186);
             //   pwr=((((-power/exponentTicks)*exponentEncoder)+power)+.15);

                if (remainingticks < -500){
                 pwr = power +.2;   
                } 
                else{
               pfactor = (-power / -500);
                pwr = ((-remainingticks * pfactor) + .2);
         
                }  
                if ((rightBack.getCurrentPosition() > -500) && (power > .3)) {
                       pfactor = (-power / -500);            
                        pwr = ((-rightBack.getCurrentPosition() * pfactor) + .45);
                 
                        if (pwr > (power+.2)){
                   
                           pwr = power +.2;  
                  
                        }
                     
                 }
                 
        bias =  -(rightFront.getCurrentPosition()*.001);
        
        
        telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightBack.getCurrentPosition());  
            telemetry.addData("pfactor:", rightBack.getCurrentPosition()); 
            telemetry.update();

                
                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation + bias ;
                double v2 = r * Math.sin(robotAngle) - rotation + bias;
                double v3 = r * Math.sin(robotAngle) + rotation + bias;
                double v4 = r * Math.cos(robotAngle) - rotation + bias;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

 

            }
            setPowerZero();

        }

        if (inches < 0) { //negative

        while ((ticks > rightBack.getCurrentPosition())&& opModeIsActive()) {
            
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                robotAngle = angles.firstAngle;
                    if ((targetAngle ==180) && (robotAngle < 1)){
                    
                    robotAngle = (360+ robotAngle);  
                      
                      
                  }
                off_angle = targetAngle - robotAngle;
            remainingticks = ticks - rightBack.getCurrentPosition();
          //      double pwr;

 //     Hysteresis to prevent overshoot
              if (off_angle < 0.3 && off_angle > -0.3) {
            rotation =0;
                
             }
            
                if (off_angle >= 0.3 || off_angle <= -0.3) {
                        if (off_angle >=.3) {
                        rotation = -(off_angle * 0.02 + 0);
            
                        }
                        if (off_angle <=-.3) {
                       rotation = -(off_angle * 0.02 + 0);
            
                        }
                      
                }
    //   if (remainingticks >500){
    //              pwr = -power -.25;   
    //             } 
    //             else{
    //           pfactor = (power / 500);
    //             pwr = (-(remainingticks * pfactor) - .25);
         
    //             }  
    //             if ((rightBack.getCurrentPosition() < 500) && (power > .3)) {
    //                     pfactor = (power / 1000);
    //                     pwr = ((-rightBack.getCurrentPosition() * pfactor) - .20);
    //                     if (pwr < (-power-.15)){
    //                       pwr = -power -.15;  
    //                     }
                
    //              }
               pwr = -power;
               
                bias =  -(rightFront.getCurrentPosition()*.001);
        telemetry.addData("remaining ticks:", remainingticks);
         telemetry.addData("pwr:", pwr);      
         telemetry.addData("ticks:", ticks);
           telemetry.addData("current position:", rightBack.getCurrentPosition());  
            telemetry.addData("pfactor:", pfactor); 
            telemetry.update();


                double r = Math.hypot(pwr, -ycorrection);
                double robotAngle = Math.atan2(-ycorrection, pwr) - Math.PI / 4;;

                double v1 = r * Math.cos(robotAngle) + rotation + bias;
                double v2 = r * Math.sin(robotAngle) - rotation + bias;
                double v3 = r * Math.sin(robotAngle) + rotation + bias;
                double v4 = r * Math.cos(robotAngle) - rotation + bias;

                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);

 

            }
            setPowerZero();

        }

    }
    //automates shooting, shoots the amount of rings the user wants.
    public void shootRing (int j){
        
        for(int i = 0; i < j; i++){
         shooterServo.setPosition(sClose); //close
        sleep(300);
        shooterServo.setPosition(sOpen); //open
        sleep(300);
        }
        sleep(200);
      
        }