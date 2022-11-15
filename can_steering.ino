#include <mcp_can.h>
#include <SPI.h>

#define PWM_L 6
#define PWM_R 5


#define EN1 3
#define EN2 4

#define ERROR_THRESH 10

struct pidVar{
  float kp;
  float kd;
  float ki;

  int sum;
  int lastError;

  int hardLimitMax;
  int hardLimitMin;

};
//Leftmost 8 | RightMost 909
struct pidVar pidPot = {50,10,0.1,0,0,1024,-1024};

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

int steeringFeedback = 0;
int error = 0;

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10


float outputActuator = 0;
int steeringCommand = 500;

float pidController(struct pidVar *pid, int error, int position)
{
  pid->sum +=error;
  // pid->sum = pid->sum > pid->hardLimitMax ? pid->hardLimitMax : pid->sum;
  // pid->sum = pid->sum < pid->hardLimitMin ? pid->hardLimitMin : pid->sum;
  float finalVal = pid->kp * error + pid->ki * pid->sum + pid->kd * (error - pid->lastError);
  pid->lastError = error;
  if(finalVal > pid->hardLimitMax)
    return pid->hardLimitMax;
  else if(finalVal < pid->hardLimitMin)
    return pid->hardLimitMin;
  return finalVal;
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(PWM_L,OUTPUT);
  pinMode(PWM_R,OUTPUT);

  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);

  digitalWrite(EN1,HIGH);
  digitalWrite(EN2,HIGH);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input

}

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if(rxId == 0x36)
    {
        steeringFeedback = rxBuf[0] | (rxBuf[1] << 8);
        // Serial.print("New Feedback: ");
        Serial.print(steeringFeedback);
        error = steeringCommand - steeringFeedback;
        if(abs(error) <= ERROR_THRESH)
          error = 0;
        outputActuator = pidController(&pidPot, (steeringCommand - steeringFeedback), steeringFeedback);
        outputActuator = (float)outputActuator/1024*255;
        Serial.print(", ");
        Serial.print(outputActuator);
        Serial.print(", ");
        Serial.println(steeringCommand);
    }  
    if(rxId == 0x69)
    {
      // Serial.print("New Commnand: ");
      steeringCommand = rxBuf[0] | (rxBuf[1] << 8);
      // Serial.print(steeringCommand);
      // Serial.print(" | OUTPUT: ");
      // Serial.println(outputActuator);
    }

    if(outputActuator > 0)
    {
      analogWrite(PWM_R,int(outputActuator));
      analogWrite(PWM_L,0);
    }
    else if(outputActuator < 0)
    {
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,int(abs(outputActuator)));
    }
    else{
      analogWrite(PWM_R,0);
      analogWrite(PWM_L,0);
    }
  }
}