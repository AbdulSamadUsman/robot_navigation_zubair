#include "mbed.h"       //this if for inporting the neccessary library components.

//the following three lines is for visualisation of the IR edge detection sensors. 
DigitalOut led1(LED1); //used for center IR sensor detection
DigitalOut led2(LED2);  //used for left IR sensor detection
DigitalOut led3(LED3);  //used for right IR detection 

//The following lines are for declaring the required IO for the dual H-Bridge module.

DigitalOut Left_Motor_forward(D8);                                          //D8    PF_13
DigitalOut Left_Motor_reverse(D6);                                          //D6    PE_9
PwmOut Left_Motor_Enable(D10);                                              //D10   PD_14
DigitalOut Right_Motor_forward(D7);                                         //D7    PF_13
DigitalOut Right_Motor_reverse(D5);                                         //D5    PE_11
PwmOut Right_Motor_Enable(D9);                                              //D9    PD_15


// the following lines are for declaring the required IO for the US sensor module.
DigitalOut USTrigger(D14);                                                  //D14   PB_9
DigitalIn USEcho(D15);                                                      //D15   PB_8

//The following line is for declaring the required IO for the Servomotor component.
PwmOut ServoMotorLocation(D3);

//The following line is for declaring the required IO for the IR sensor component. 
InterruptIn LeftIR(A1);                                                    
InterruptIn CenterIR(A2);    
InterruptIn RightIR(A3);

// Since we are using IR sensors for proximity detection so these lines are not required
InterruptIn LeftIR_line(A4);
InterruptIn CenterIR_line(A5);
InterruptIn RightIR_line(A6);
/*--------------------------------------------------------------------------------------------------------*/

/*          function prototyping statments*/
void Initialise();
void Stop();
void Move_Forward(float speed);
void Move_Backward(float speed);
void Rotate_Right(float speed);
void Rotate_Left(float speed);
void Move_Forward(float speed, int forward_time);
void Move_Backward(float speed, int backword_time);
void Rotate_Right(float speed, int Rotate_Right_time);
void Rotate_Left(float speed, int Rotate_Left_time);
void InterruptLeftIRFall();
void InterruptLeftIRRise();
void InterruptRightIRFall();
void InterruptRightIRRise();
void ServoMotorScan();
void USDistanceMeassure();
bool DecideOnUSRightOrLeft();


//New functions for center IR sensors interrupt routines
void InterruptCenterIRFall();
void InterruptCenterIRRise();



/*the following is for declaring variables from meassurments. */
int loopCounter=0;

int EchoTimeus=0;               //US echo time.... ~340 m/sec which relates to distance.....
float fUSDistance=0.0f;         //Variable that takes the previous variable and calculates the distance....
float fDistanceArray[11];       //This is for creating an array for the US readings..... radar array. 




/*This where your programme starts.....*/
int main()
{
    //Following statments if for runtime debugging...
    printf("This is the bare metal robot running on Mbed OS %d.%d.%d.\n", MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);
    printf("Starting the robot programme!!!!!!!\r\n");
    printf("Muhammad Zubair\r\n");
    printf("School of Physics Engineering and Computer Science\r\n");
    printf("University of Hertfordshire\r\n");
    printf("Robot bare metal Version 2021\r\n");




    printf("Stop the robot\r\n");
    Stop();                 //Stop the Robot



    printf("\r\n Initialize the Hbridge and servomotor");
    Initialise();

    
/*Do a sweep to "see" what is infront of the robotic platform.*/
    ServoMotorScan();
    for(int i=0; i<11; i++)
        printf("The distance array is %f\r\n", fDistanceArray[i]);//Report meassurements on the Monitor
    






// You can select if you want to implement:
// A) the edge IR detection (line follower), or
// B) the proximity IR detection (obstacle detection) 
//The code template does B

    LeftIR.fall(&InterruptLeftIRFall);
    LeftIR.rise(&InterruptLeftIRRise);
    RightIR.fall(&InterruptRightIRFall);
    RightIR.rise(&InterruptRightIRRise);
    CenterIR.fall(&InterruptCenterIRFall);   
    CenterIR.rise(&InterruptCenterIRRise);   


    //LeftIR_line.fall(&function_name4LeftLinefall);
    //LeftIR_line.rise(&function_name4LeftLinerise);
    //CenterIR_line.fall;
    //CenterIR_line.rise;
    //RightIR_line.fall;
    //RightIR_line.rise;





//Test drive to check if the motors move in the way I wrote the code or if direction needs adjustment.

    printf("Testing dc motor Moving Forward\r\n");
    Move_Forward(0.8f);
    thread_sleep_for(1000);
    printf("Testing dc motor Rotate Left\r\n");
    Rotate_Left(0.8f);
    thread_sleep_for(1000);
    printf("Testing dc motor Rotate Right\r\n");
    Rotate_Right(0.8f);
    thread_sleep_for(1000);
    printf("Testing dc motor Moving Backwords\r\n");
    Move_Backward(0.8f);
    thread_sleep_for(1000);
    Stop();

// From here on you can write your own algorithm.....    

    while (1)
    {
        printf("while loop iteration begins");
        thread_sleep_for(50);   //This is for sleeping for 50 msec;
        if (led1 == 1) //If an obstacle is present in front of Centeral IR sensor
        {
            Stop();
            Move_Backward(0.8, 200);
            ServoMotorScan();
            if (DecideOnUSRightOrLeft())
            {
                Rotate_Right(0.8, 200);
            }
            else
            {
                Rotate_Right(0.8, 200);
            }

        }

        else if (led1 && led2 && led3 == 1) //Obstacles on all 3 sides left, right and center
        {
            Stop();
            Move_Backward(0.8, 200);
            ServoMotorScan();
            if (DecideOnUSRightOrLeft())
            {
                Rotate_Right(0.8, 200);
            }
            else
            {
                Rotate_Right(0.8, 200);
            }
        }

        else if (led1 && led2 == 1)  //obstacle on left and front
        {
            Stop();
            Move_Backward(0.8, 200);
            Rotate_Right(0.8, 700);
            ServoMotorScan();
            if (DecideOnUSRightOrLeft())
            {
                Rotate_Right(0.8, 200);
            }
            else
            {
                Rotate_Right(0.8, 200);
            }
        }

        else if (led1 && led3 == 1) //obstacle on right and front
        {
            Stop();
            Move_Backward(0.8, 200);
            Rotate_Left(0.8, 700);
            ServoMotorScan();
            if (DecideOnUSRightOrLeft())
            {
                Rotate_Right(0.8, 200);
            }
            else
            {
                Rotate_Right(0.8, 200);
            }
        }

        else 
        {
            Move_Forward(0.8, 200);
        }
        
    }
}

//Use the US sensor and decide if you need to turn right or left....

bool DecideOnUSRightOrLeft()  //very simple algorithm to decide if we turn right or left.
{
//the following is a very simple algorithm to decide if we are turnging Right or Left.
int iRightSide=0;
int iLeftSide=0;
for(int i=0; i<5; i++)
    iRightSide+=fDistanceArray[i];
for(int i=5; i<10; i++)
    iLeftSide+=fDistanceArray[i];    

if(iRightSide>=iLeftSide)
    return true;
else
    return false;
}



void InterruptLeftIRFall()      //Obstacle on the left detected
{
    led2=1;
    return;
}
void InterruptLeftIRRise()      //Obstacle on the left is cleared
{
    led2=0;
    return;
}

void InterruptRightIRFall()      //Obstacle on the Right detected
{
    led3=1;
    return;
}
void InterruptRightIRRise()      //Obstacle on the Right is cleared
{
    led3=0;
    return;
}

void InterruptCenterIRFall()  //Obstacle on the front is detected
{
    led1 = 1;
    return;
}

void InterruptCenterIRRise()  //Obstacle on the front is cleared
{
    led1 = 0;
    return;
}



/*      This is for stoping the DC motors....*/

void Stop()
{
    Left_Motor_Enable=0.0f;
    Right_Motor_Enable=0.0f;
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=0;

    printf("Robot stopped\r\n");
    return;
}






/************************************************************************************
*           Setup the microcontroller peripheral units.                             *
*           This is for the setup of the H-Bridge and servomotor of the US modules. *
*************************************************************************************/
void Initialise()
{
    Right_Motor_Enable.period_us(50);               //DC Right Motor PWM base Period = 50 usec-> 20 kHz
    Left_Motor_Enable.period_us(50);                //DC Left Motor PWM base Period = 50 usec -> 20 kHz
    ServoMotorLocation.period_us(20000);            //ServoMotor PWM base Period = 20 msec ->50 Hz
    return;
}


void ServoMotorScan()
{
    for(int i=0;i<11;i++)
    {
        ServoMotorLocation=((float)i*0.2f+0.5f)/20.0f;
        USDistanceMeassure();  
        fDistanceArray[i]= fUSDistance;
        thread_sleep_for(150);
    }

    ServoMotorLocation=1.5f/20.0f;                  //Leave the US sensor pointing forward.
    thread_sleep_for(200);
    return;
}

void USDistanceMeassure()
{
        USTrigger=0;
        wait_us(2);
        USTrigger=1;
        wait_us(10);
        USTrigger=0;
        EchoTimeus=0;
        while(USEcho==0)
            EchoTimeus=0;
        while(USEcho==1)
        {
            wait_us(1);
            EchoTimeus++;
        }
        fUSDistance=(float)EchoTimeus/29.0f/2.0f;
        return;
}





void Move_Forward(float speed)
{
    printf("Robot moving forward\r\n");
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=1;
    Left_Motor_forward=1;
    return;
}



void Move_Backward(float speed)
{
    printf("Robot Move Backwards\r\n");
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=1;
    Left_Motor_reverse=1;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    return;
}

void Rotate_Right(float speed)
{
    printf("Robot Rotate Right\r\n");
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=0;
    Left_Motor_reverse=1;
    Right_Motor_forward=1;
    Left_Motor_forward=0;
    return;
}

void Rotate_Left(float speed)
{
    printf("Robot Rotate Left\r\n");
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=1;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=1;
    return;
}


void Move_Forward(float speed, int forward_time)
{
    printf("Robot Move Forward for %d msec\r\n", forward_time);
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=1;
    Left_Motor_forward=1;
    thread_sleep_for(forward_time*1);
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    return;
}

void Move_Backward(float speed, int backward_time)
{
    printf("Robot Move Backwards for %d msec\r\n", backward_time);
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    Right_Motor_reverse=1;
    Left_Motor_reverse=1;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    thread_sleep_for(backward_time*1);
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    return;
}

void Rotate_Right(float speed, int Rotate_Right_time)
{
    printf("Robot Rotate Right for %d msec\r\n", Rotate_Right_time);
    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    
    Right_Motor_reverse=0;
    Left_Motor_reverse=1;
    Right_Motor_forward=1;
    Left_Motor_forward=0;
    thread_sleep_for(Rotate_Right_time*1);
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    return;
}

void Rotate_Left(float speed, int Rotate_Left_time)
{
    printf("Robot Rotate Left for %d msec\r\n", Rotate_Left_time);

    Left_Motor_Enable=speed;
    Right_Motor_Enable=speed;
    
    Right_Motor_reverse=1;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=1;
    thread_sleep_for(Rotate_Left_time*1);
    Right_Motor_reverse=0;
    Left_Motor_reverse=0;
    Right_Motor_forward=0;
    Left_Motor_forward=0;
    return;
}
