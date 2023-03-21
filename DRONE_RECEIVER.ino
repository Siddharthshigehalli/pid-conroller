/* 
                                                :DRONE RECEIVER:
 we are using arduino UNO R3 in drone .The important part of the program is: #esc calibration, 
#data rceiving ad sending,#potentiometer and #joystick programming.


            This program is written by
            Siddharth R. Shigehalli
             in 23/12/2019.
             And Reedited By
             Siddharth R. shigehalli
             on 19/05/2020.       

     /////////:-DRONE COMPONENTS:-///////////
                 * Arduino UNO R3                
                 * NRF24L01 Receiver Module
                 * MPU6050 6-axis Gyroscope
                 * Brushless motors[1800Kv]
                 * ESCs[30A]
                 * 12v 5200mAh battery.

   \\\\\\\\\\\\-:DRONE RECEIVER PINS EXPLAINED:-////////////
            1] Arduino communication pins:-
                Arduino nano    |  NRF24L01
                   * D11        |  MOSI
                   * D12        |  MISO
                   * D13        |  SCK
                   * D7         |  CE
                   * D8         |  CSN

            2] Arduino and brushless motors connections:-
                Arduino nano    |  ESCs
                   * D5         |  1st ESC[F_LEFT_ARM] M1
                   * D6         |  2nd ESC[F_RIGHT_ARM] M2
                   * D9         |  3rd ESC[B_LEFT_ARM] M3 
                   * D10        |  4th ESC[B_RIGHT_ARM] M4

           3] Arduino and MPU6050 6-axis Gyroscope connnections:-
                Arduino nano    |  MPU6050
                   * A5         |  SCL
                   * A4         |  SDA

           4] Arduino and Current sensing resitors:-
                 Arduino nano   |  Resistor
                   *A6          |   middle wireof 7.5k ohms and 5.1k ohms
           

        //------------------- ELUER'S FORMULA ----------------//
 
            // Accel_AngleY = atan((Accel_raw_Y / LSB Sensitivity) / squrt(pow((Accel_raw_X / LSB Sensitivity),2) + pow((Accel_raw_Z / LSB Sensitivity),2)))* Rad_To_Deg);
                
                          LSB Sensitivity = MPU6050 register map aceelerometer measurement
                               Rad_To_Deg = 180/3.141592654;
                                     atan = atan will calculate the arc tangent;


        //----------------- Toatal Angle ------------------//

            //  Total_angle_axis = 0.98 *(Total_angle_axis + Gyro_angle_axis*elapsedTime) + 0.02*Acceleration_angle_axis;

                       axis = same in all variable in one equation;
                 0.98, 0.02 - They are complimentary filters to get real total angle;
                 

       //----------------- PID CONTROLLER -----------------//
                    PID Stand for : PROPORTIONAL
                                    INTEGRAL
                                    DERIVATIVE
                   PID Controller is an instrument used in this project to 
                   regulate PWM signals & We Calculate The PWM Signal 
                   By using PID formula Total Angle data and Maped 
                   PWM signal which are received from MPU6050 Module
                   and Drone Transmitter Respectively.
           Formula:
                u(t)_p = kp*(e)
                u(t)_i = u(t)_i + (Ki*e)
                u(t)_d = Kd*((e - Previous_e) /t)
                
                  u(t) = u(t)_p + u(t)_i + u(t)_d
                 
                        u(t)_p, u(t)_i, u(t)_d - PID Proportional, Integral, Derivartive varable respectively
                        
                        Kp = Proportional constant
                        Ki = Integral constant
                        kd = derivative constant
                         t = elapsed or passed time
                         e = error variable or change in error
                      u(t) = PID varable
                       
                       Previous_e = previous error **


        //===================== Drone Axis =====================//
                   Drone Roll-AXis = The axis which passes frome tale of drone 
                                     to front face or nose of drone.--------
                                     In this case it is X-axis.
                                      
                  Drone Pitch-Axis = The axis which is paralell to drone wings 
                                     or which bisects the angle of any two 
                                     side wings of drone.----- In The case it 
                                     is Y-axis.


       //========================== Joystick Vaues ==========================//
                         Joystick_1: 
                         
                                 High Value = 255
                               Middle Value = 130 [R = 0 && L = 255]
                                  Low Value = 0

                         Joystick_2X:

                                High Value = 255
                              Middle Value = 132 [D = 0 && Up = 255]
                                 Low Value = 0

                        Joystick_2Y:

                               High Value = 255
                             Middle Value = 128 [L = 0 && R = 255]
                                Low Value = 0

                         Joystick_2X = Throttle[2] = Roll-axis
                         Joystick_2y = Throttle[3] = Pitch-axis
                         
                         Joystick_1  = Main Throttle Vaue = Throttle[1] = Yaw-axis                                 

*/

#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
#include<Servo.h>
#include<Wire.h> 

//Radio variables:
  RF24 radio(7, 8);                          //Assign CE and CSN pins
  const byte address[14] = "NRFTWENTYFOUR";  //Assign the constant byte address and store in array 
  int Throttle[6];  //Assign Throttle Variable To Store The Received Data
  float Thrust[6]; //Assign Thrust array to convert the received data into float

//Gyro Variables:
  float Gyro_raw_angleX, Gyro_raw_angleY, Gyro_raw_angleZ;
  float Gyro_raw_error_X, Gyro_raw_error_Y;
  float Gyro_AngleX, Gyro_AngleY;
  float   Gyro_error = 0;

//Accel Variables:
  float Accel_raw_angleX, Accel_raw_angleY, Accel_raw_angleZ;
  float Accel_angle_error_X, Accel_angle_error_Y;
  float Accel_AngleX, Accel_AngleY;
  float   Accel_error = 0;

//Total Angle Variables:
  float Total_AngleX;
  float Total_AngleY;
  
//Other Variables:
  float elapsedTime, timePrev, time;
  float Rad_To_Deg = 180/3.141592654;

//PWM float Variables:
  float R_F_raw_PWM;
  float R_B_raw_PWM;
  float L_F_raw_PWM;
  float L_B_raw_PWM;

//PWM int Variables:
  int R_F_PWM;
  int R_B_PWM;
  int L_F_PWM;
  int L_B_PWM;

//PID Variables For Roll Axis:
  float Roll_PID;
  float Roll_error, Roll_Previous_error;
  float Roll_PID_p = 0;
  float Roll_PID_i = 0;
  float Roll_PID_d = 0;

//Roll PID Constants ;
  float Roll_Kp = 0.7;
  float Roll_Ki = 0.00;
  float Roll_Kd = 1.2;
  float Roll_Desired_Angle;

//PID Variables For Pitch AXis:
  float Pitch_PID;
  float Pitch_error, Pitch_Precious_error;
  float Pitch_PID_p = 0;
  float Pitch_PID_i = 0;
  float Pitch_PID_d = 0;

//Pitch PID Constants:
  float Pitch_Kp = 0.72;
  float Pitch_Ki = 0.00;
  float Pitch_Kd = 1.2;
  float Pitch_Desired_Angle;
  

//===== Now we have to creat Servo objects =====//
    Servo R_F_ARM;
    Servo L_F_ARM;
    Servo L_B_ARM;
    Servo R_B_ARM;
    

void setup() {
  // put your setup code here, to run once:

//------- Now We Have To Cliberate All The ESCs -------//
    R_F_ARM.attach(5,1000,2000);
    L_F_ARM.attach(6,1000,2000);
    L_B_ARM.attach(9,1000,2000);
    R_B_ARM.attach(10,1000,2000);

    Wire.begin();
    SPI.begin();                        //Start the SPI communication
    radio.begin();                      //Start the radio communication
    radio.openReadingPipe(0,address); //Set Pip or Loop to receive the data
    radio.setPALevel(RF24_PA_MAX);    //Set the range of the radio signal in max mode[1Km] range
    radio.setDataRate(RF24_2MBPS);    //Set the baud rate
    radio.setChannel(0x4C);           //Set the channel
    radio.startListening();          //Set The Module As Receiver
    Serial.begin(9600);              //Start Serial Communication

    


   
 //================================================================== -: SECTION 01 :- ==================================================================//
                     //In this Section we calculate Acceleration, Gyro and Total angle


                          
                          ////////// Configure The MPU6050 Module //////////
          
   //Power Management:
     Wire.beginTransmission(0b1101000);   //This is the I2C address of MPU6050 (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
     Wire.write(0x6B);                   //Access the register 6B - Power Management (Sec. 4.28, "register data sheet")
     Wire.write(0b00000000);             // Setting Sleep register to 0.(Required; see Note on p. 9
     Wire.endTransmission();
    

   //Gyroscope Configeration: 
     Wire.beginTransmission(0b1101000);    //I2C address of MPU6050
     Wire.write(0x1B);                     //Accessing the register 1B
     Wire.write(0b00000000);             //Setting the gyro full scale range +/- 250 ./S [degrees per second]
     Wire.endTransmission();

    //Accelerometer Configeration:
      Wire.beginTransmission(0b1101000);  //I2C address of MPU6050
      Wire.write(0x1C);                   //Accessing the register 1C
      Wire.write(0b00010000);           //Setting the Accelerometer full scale range +/- 8g
      Wire.endTransmission();

  


                   //-------------------------- Calculate The Gyro_Raw_Error --------------------------//
                
                                 //Here We Calculate The Gyro Raw Data Before We Move To The Loop.
                                      //* We make mean of 200 values, to calculate the gyro raw error.

 if(Gyro_error==0)
  {
     for(int i=0; i<200; i++)
       {
        //recodGyroRegisters:
          Wire.beginTransmission(0b1101000);           //I2C address of MPU6050
          Wire.write(0x43);                           //Startig the ragister for gyro readings
          Wire.endTransmission();
          Wire.requestFrom(0b1101000,6);              //Request for all gyro registers (43 to 48)
          while(Wire.available() < 6);
          Gyro_raw_angleX = Wire.read()<<8|Wire.read(); //Store first two X-axis bytes into Gyro_raw_angleX
          Gyro_raw_angleY = Wire.read()<<8|Wire.read(); //Store middle two Y-axis bytes into Gyro_raw_angleY
      //    Gyro_raw_angleZ = Wire.read()<<8|Wire.read(); // Store last two Z-axis bytes into Gyro_raw_angleZ
         
       //Calculate Gyro Raw Error:
         Gyro_raw_error_X =  Gyro_raw_error_X + (Gyro_raw_angleX/131.0); //X-axis
         Gyro_raw_error_Y =  Gyro_raw_error_Y + (Gyro_raw_angleY/131.0); //Y-axis
         
    if(i==199)
    {
      Gyro_raw_error_X = Gyro_raw_error_X/200; //X-axis
      Gyro_raw_error_Y = Gyro_raw_error_Y/200; //Y-axis
      Gyro_error=1;
   }
  }
 }//End of Gyro error Calculation




    //------------------------------ Calculate The Accel_angle_Error ------------------------------//
    
                          //Here We Calculate The Gyro Raw Data Before We Move To The Loop.
                               //*  We make mean of 200 values, to calculate the gyro raw error.

 if(Accel_error==0)
 {
 for(int a=0; a<200; a++)
  {
    //record Acceleratopn registers:
      Wire.beginTransmission(0b1101000);              //I2C address of MPU6050
      Wire.write(0x3B);                               //Starting the register for accel readings
      Wire.endTransmission();
      Wire.requestFrom(0b1101000,6);                   //Request for all accel registers ( 3B to 40)
      while(Wire.available() < 6);
      Accel_raw_angleX = (Wire.read()<<8|Wire.read())/4096.0;  //Store first two X-axis bytes into Accel_raw_angle
      Accel_raw_angleY = (Wire.read()<<8|Wire.read())/4096.0; //Store middle two Y-axis bytes into Accel_raw_angleY
      Accel_raw_angleZ = (Wire.read()<<8|Wire.read())/4096.0;  // Store last two Z-axis bytes into Accel_raw_angleZ

   //Calculate Accel Angle Error:
     Accel_angle_error_X = Accel_angle_error_X + ((atan((Accel_raw_angleY)/sqrt(pow((Accel_raw_angleX),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg));  //X-axis
    
     Accel_angle_error_Y = Accel_angle_error_Y + ((atan(-1*(Accel_raw_angleX)/sqrt(pow((Accel_raw_angleY),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg));//Y-axis

   if(a==199)
     {
        Accel_angle_error_X = Accel_angle_error_X/200; //X-axis
        Accel_angle_error_Y = Accel_angle_error_Y/200; //Y-axis
        Accel_error=1;
  }
 }
}//End of Accel error Calculation

}
 
void loop() {
  // put your main code here, to run repeatedly:


        ////////// Calculate The Elapsed Time //////////
             timePrev = time;   // the previous time is stored before the actual time read  
                 time = millis();  //Actual Time Read in mili Seconds
          elapsedTime = (time - timePrev)/1000;     //Calculate elapsed Time     


      //------------------ Calculate The Gyro Angle :------------------//

     //Record Gyro Registers:
       Wire.beginTransmission(0b1101000);           //I2C address of MPU6050
       Wire.write(0x43);                           //Startig the ragister for gyro readings
       Wire.endTransmission();
       Wire.requestFrom(0b1101000,6);              //Request for all gyro registers (43 to 48)
       while(Wire.available() < 6);
       Gyro_raw_angleX = Wire.read()<<8|Wire.read(); //Store first two X-axis bytes into Gyro_raw_angleX
       Gyro_raw_angleY = Wire.read()<<8|Wire.read(); //Store middle two Y-axis bytes into Gyro_raw_angleY
  //     Gyro_raw_angleZ = Wire.read()<<8|Wire.read(); // Store last two Z-axis bytes into Gyro_raw_angleZ

    //Calculate The Gyro_Raw_Angle:
      Gyro_raw_angleX = (Gyro_raw_angleX/131.0) - Gyro_raw_error_X; //Gyro_Angle_X  
      Gyro_raw_angleY = (Gyro_raw_angleY/131.0) - Gyro_raw_error_Y; //Gyro_Angle_Y

    //Calculate The Gyro Angle:
            //Now we integrate the raw value in degrees per second in order to obtain the gyro angle.
               //* If we multiplied degrees/second * degrees we get "degrees".
               
      Gyro_AngleX = Gyro_raw_angleX*elapsedTime; //X-axis
      
      Gyro_AngleY = Gyro_raw_angleY*elapsedTime; //Y-axis


     //------------------------- Calculate The Accel Angle :-------------------------//

         //record Accel Registers:
            Wire.beginTransmission(0b1101000);              //I2C address of MPU6050
            Wire.write(0x3B);                               //Starting the register for accel readings
            Wire.endTransmission();
            Wire.requestFrom(0b1101000,6);                   //Request for all accel registers ( 3B to 40)
            while(Wire.available() < 6);
            Accel_raw_angleX = (Wire.read()<<8|Wire.read())/4096.0; //Store first two X-axis bytes into Accel_raw_angleX
            Accel_raw_angleY = (Wire.read()<<8|Wire.read())/4096.0; //Store middle two Y-axis bytes into Accel_raw_angleY
            Accel_raw_angleZ = (Wire.read()<<8|Wire.read())/4096.0; // Store last two Z-axis bytes into Accel_raw_angleZ


       //Calculate The Accel Angle:
               //Now in order to get Accel Angles we use Euler's formula with Acceleration values.
               //*And we substract the error value.

         Accel_AngleX = (atan((Accel_raw_angleY)/sqrt(pow((Accel_raw_angleX),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg) - Accel_angle_error_X;     //X-axis

         Accel_AngleY = (atan(-1*(Accel_raw_angleX)/sqrt(pow((Accel_raw_angleY),2) + pow((Accel_raw_angleZ),2)))*Rad_To_Deg) -  Accel_angle_error_Y; //Y-axis
    
               
         //--------------------- Total Angle :--------------------------//

                //Calculate The Total Angle:
                  Total_AngleX = 0.98 *(Total_AngleX + Gyro_AngleX) + 0.02*Accel_AngleX; //Xaxis
   
                  Total_AngleY = 0.98 *(Total_AngleY + Gyro_AngleY) + 0.02*Accel_AngleY; //y-axis


                  
              //--------------------- Now Print The Data On Serial Monitor ------------------//
                         /*
                            Serial.print(" Gyro Data (deg)= ");
                            Serial.print(" Gyro_AngleX = ");
                            Serial.print(Gyro_AngleX);
                            Serial.print(" Gyro_AngleY = ");
                            Serial.println(Gyro_AngleY);
                         
                            Serial.print("  Accel Data (deg)= ");
                            Serial.print(" Accel_AngleX = ");
                            Serial.print(Accel_AngleX);
                            Serial.print(" Accel_AngleY = ");
                            Serial.println(Accel_AngleY);                    

                          */  
                            Serial.print("  Total Angle Data (deg)= ");                        
                            Serial.print("  Total_AngleX = ");                    
                            Serial.print(Total_AngleX); 
                            Serial.print("  Total_AngleY = ");                    
                            Serial.println(Total_AngleY);              
                         


                         

  //======================================== xxxxxxxxxxxxxxxxxxxxxxx -: END OF SECTION 01 :- xxxxxxxxxxxxxxxxxxxxxxx ========================================//
        
      



  
   //==================================================================== -: SECTION 02 :- ==================================================================//
                       //In this Section we receive and read all transmitted joystick values:
          

    ////////// Read The Incoming data //////////
      radio.read(&Throttle,sizeof(Throttle));      //Read The Available Data And Store In Throttle Variable

   //2nd Received data test running:
     /*
      if(ButtonState[5] == HIGH){
        Serial.println("yes");
      } else {
        Serial.println("no");
      }
         */
   //-------------- Now Print Data On Serial Monitor --------------//
          /*
            Serial.print(" INCOMINGDATA LIST ()=");
            Serial.print(" Joy_1=");
            Serial.print(Throttle[1]);
            Serial.print(" Joy_2_X=");
            Serial.print(Throttle[2]);
            Serial.print(" Joy_2_Y=");
            Serial.println(Throttle[3]);
          */

      
    //========================================== XXXXXXXXXXXXXXXXXXXXX -: END OF SECTION 02 :- XXXXXXXXXXXXXXXXXXXXX ==============================================//






 //====================================================================== -: SECTION 03 :- ======================================================================//
                                                    //In This Section We Regulate The PWM Signals For Auto Stability Of The Drone
                                                             //** Using Data From Section 1 & 2.


  ///////////////// Map The Roll Desired Angle /////////////////
                if(Throttle[2] == 132) {
                    Roll_Desired_Angle = 0;
                    
                } else if(Throttle[2] < 120) {
                    Roll_Desired_Angle = -10;
                  
                } else if(Throttle[2] > 140) {
                   Roll_Desired_Angle = 10;
                }
      
 ///////////////// Map The Pitch Desired Angle /////////////////
               if(Throttle[3] == 128) {
                   Pitch_Desired_Angle = 0;
                   
               } else if(Throttle[3] < 125) {
                   Pitch_Desired_Angle = -10;
                   
               } else if(Throttle[3] > 140) {
                   Pitch_Desired_Angle = 10;
               }

   //------------------- Now Print The Maped Derired Angle Data On Serial Monitor -----------------//
                  /*
                     Serial.print(" ROll_Desired_Angle=");
                     Serial.print(Roll_Desired_Angle);

                     Serial.print("  Pitch_Desired_Angle=");
                     Serial.println(Pitch_Desired_Angle);
                  */
                
//----------------------------- PID CONTROLLER -----------------------------//

     //First We Calculate The error:
       Roll_error = Total_AngleX - Roll_Desired_Angle;   //Calculate the roll error 
       Pitch_error = Total_AngleY - Pitch_Desired_Angle; //Calculate the pitch error

    //Now Calculate The Proportional Value of Pitch and Roll PID:
       Roll_PID_p = Roll_Kp*(Roll_error);   // Now Multiply Proportional **
       Pitch_PID_p = Pitch_Kp*(Pitch_error); //** constant with the error

   //Now Calculate The Intigral Value of Pitch and Roll PID:
    if(-3 < Roll_error < 3) {
      Roll_PID_i =  Roll_PID_i + (Roll_Ki*Roll_error);   // To intigrate we just sum the previous intigral Value with the error
    }
     if(-3 < Pitch_error < 3) {
      Pitch_PID_i = Pitch_PID_i + (Pitch_Ki*Pitch_error); //**multiplied by intgral constant & it will increase the value of Intigral PID value. 
     }

 //Now Calculate The Derivative Value of Roll and Pitch PID:
  /*The Last part is derivative control works on how fast or speed the error is 
   * changing.As we know the speed is the amount of error produced in a certain 
   * amount of time and divided by that time.The Formula is:
   * Speed (s) = (e)/ elapsed time, where (e) is actual error which is 
   * obtained by subtracting the error by previous error*/
     Roll_PID_d = Roll_Kd*((Roll_error - Roll_Previous_error)/elapsedTime);
     Pitch_PID_d = Pitch_Kd*((Pitch_error - Pitch_Precious_error)/elapsedTime);


  //Now Calculate The Final PID Value:
    /*Final PID value is the sum of Proportioal Intigral
     * and Derivative part of PID*/
     Roll_PID = Roll_PID_p + Roll_PID_i + Roll_PID_d;
     Pitch_PID = Pitch_PID_p + Pitch_PID_i + Pitch_PID_d;


  /*Befor we move forward we know that we can supply PWM signals to the ESCs from 0 to 255.
   * But PID Value can oscilate from -2000us to 2000us,So during PWM signal Calculation
   * it turned out to be an error that's why we need to adjust the PID value as -+400, because this
   * is the approximate  value we get when the drone angle is from -10 degrees to 10 degrees*/
    //For ROLL PID:            
      if(Roll_PID <= -400) 
          { Roll_PID = -400;}   
                           
      if(Roll_PID >= 400)  
           { Roll_PID = 400;} 
                                  
    //For PITCH PID:                                 
      if(Pitch_PID <= -400) 
          { Pitch_PID = -400;}
          
      if(Pitch_PID >= 400)  
         { Pitch_PID = 400;}


 
  //======================================== XXXXXXXXXXXXXXXXXXXXX -: END OF SECTION 03 :- XXXXXXXXXXXXXXXXXXXXX ========================================//





//====================================================================== -: SECTION 04 :- ======================================================================//
                                                              //In This Section we calculate the PWM width & send the signal to ESCs.

  //Now we need convert the PWM int values into float values:
    /*Because it is necessity for calculating raw_pwm_value*/
      Thrust[1] = (float)Throttle[1];
     // Serial.println(Throttle[2]);
  

   //Now Calculate The PWM Width:
     /*Now we need to Calculate the PWM width by using PID and Throttle values
      * For that I've made a simple calculation,in this calculation
      * we just need add and subtract the PID values according
      * to the MPU6050 axes graph*/
       R_F_raw_PWM = Thrust[1] - Roll_PID + Pitch_PID; //Because R_F_PWM lies in 2nd quadrant
       R_B_raw_PWM = Thrust[1] + Roll_PID + Pitch_PID; //Because R_B_PWM lies in 1st quadrant
       L_F_raw_PWM = Thrust[1] - Roll_PID - Pitch_PID; //Because L_F_PWM lies in 3rd quadrant
       L_B_raw_PWM = Thrust[1] + Roll_PID - Pitch_PID; //Because L_B_PWM lies in 4th quadrant


   //Now we need to convert the PWM float values into integer values:
     /* Because PWM is an integer it cannot take floatint pont values*/
       R_F_PWM = (int)R_F_raw_PWM;
       R_B_PWM = (int)R_B_raw_PWM;
       L_F_PWM = (int)L_F_raw_PWM;
       L_B_PWM = (int)L_B_raw_PWM;       

    /*We know that we can supply minimum PWM signal is o and maximum is 255
     * but after the PWM width calculation PWM signal is exceeding the maximum
     * value so we need to adjust it and we also need to adjust the minimum value 
     * because it is not accurate */
       if(R_F_PWM >= 255)
         { R_F_PWM = 255;}
       if(R_F_PWM < 5)
         { R_F_PWM = 0;}

       if(R_B_PWM >= 255)
         { R_B_PWM = 255;}
       if(R_B_PWM < 5)
         { R_B_PWM = 0;}

       if(L_F_PWM >= 255)
         { L_F_PWM = 255;}
       if(L_F_PWM < 5)
         { L_F_PWM = 0;}

       if(L_B_PWM >= 255)
         { L_B_PWM = 255;}
       if(L_B_PWM < 5)
         { L_B_PWM = 0;}
       


    // Now Send The Signal To The ESCs:
       /*In this last part of the coding we send the
        * calculated PWM width signal to all the ESCs.
        * we send the signal to the ESCs in CCW order 
        * of the arms & it starts from R_F_ARM */

    /* We made some modifications in code for better control by triggering the PWM
     *  means if the push button is on send the Calculated PWM signal to the esc
     *  otherwise send low value to turn off he motor.
     *  Here Throttle[5] is ButtonState */
        if(Throttle[5] == HIGH){     
           R_F_ARM.write(R_F_PWM);
           L_F_ARM.write(L_F_PWM);   
           L_B_ARM.write(L_B_PWM);
           R_B_ARM.write(R_B_PWM);
           
     } else {   
            R_F_ARM.write(0);
            L_F_ARM.write(0);
            L_B_ARM.write(0);
            R_B_ARM.write(0);
          }
          
// Now we store the error in previous error because it nedded for calculation process.  
       Roll_Previous_error = Roll_error;
       Pitch_Precious_error = Pitch_error;
          
     //--------------------- Now Print The Calculted PWM Width On Serial Monitor ---------------------//
                /*
                  Serial.print(" R_F_PWM=");
                  Serial.print(R_F_PWM);
                  Serial.print("  R_B_PWM=");
                  Serial.print(R_B_PWM);
                  Serial.print("  L_F_PWM=");
                  Serial.print(L_F_PWM);
                  Serial.print("  L_B_PWM=");
                  Serial.println(L_B_PWM);
                */


        

 //======================================== XXXXXXXXXXXXXXXXXXXXX -: END OF SECTION 04 :- XXXXXXXXXXXXXXXXXXXXX /========================================//
 

}

 
   
