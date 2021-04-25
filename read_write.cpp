/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

//
// *********     Read and Write Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 2.0
// This example is tested with a Dynamixel PRO 54-200, and an USB2DYNAMIXEL
// Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h> 

#include "dynamixel_sdk.h"                                 // Uses Dynamixel SDK library

// Control table address
#define ADDR_OPERATING_MODE             11                 // Control table address is different in Dynamixel model
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2                   // Dynamixel ID: 2
#define DXL_ID3                         3                   // Dynamixel ID: 3
#define DXL_ID4                         4                   // Dynamixel ID: 4
#define DXL_ID5                         5                   // Dynamixel ID: 5
#define DXL_ID6                         6                   // Dynamixel ID: 6
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define DXL_MOVING_STATUS_THRESHOLD     5                   // Dynamixel moving status threshold
#define EXT_POSITION_CONTROL_MODE       4                   // Value for extended position control mode (operating mode)

#define ESC_ASCII_VALUE                 0x1b
#define SPACE_ASCII_VALUE               0x20

#define M1o             2900
#define M2o             750
#define M3o             3200
#define M4o             470
#define M6o             1100
#define M5o             1400
#define M5c             500
#define	PI		3.14159265358979323846
#define	P		26
#define	Theta0		(63.43494882*PI)/180

float u11, u12, u13, u21, u22, u23, u31, u32, u33, px, py, pz;
float Px, Py, Pz, U11, U12, U13, U23, U31, U32, U33;
float Thetax, Thetay, Thetaz;
float degreex, degreey, degreez;
int M1, M2, M3, M4, M5, M6;

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

//////////////////////////////////////////
//////////* Forward Kinematics *//////////
//////////////////////////////////////////

/* Given angles Thetax, Thetay, Thetaz calculate position and orientation u11, u12, u13, u21, u22, u23, u31, u32, u33, px, py, pz */
void ForwardKinematics(float Thetax, float Thetay, float Thetaz) {

	u11 = cos(Thetaz)*cos(Thetax)*cos(Thetay)-sin(Thetaz)*sin(Thetay);
	u12 = -cos(Thetay)*sin(Thetaz)-cos(Thetaz)*cos(Thetax)*sin(Thetay);
	u13 = cos(Thetaz)*sin(Thetax);
	u21 = cos(Thetaz)*sin(Thetay)+cos(Thetax)*cos(Thetay)*sin(Thetaz);
	u22 = -cos(Thetax)*sin(Thetaz)*sin(Thetay)+cos(Thetaz)*cos(Thetay);
	u23 = sin(Thetaz)*sin(Thetax);
	u31 = -cos(Thetay)*sin(Thetax);
	u32 = sin(Thetax)*sin(Thetay);
	u33 = cos(Thetax);
	px = (26*cos(Thetaz)*sin(Thetax));
	py = (26*sin(Thetaz)*sin(Thetax));
	pz = (26*cos(Thetax));

}

//////////////////////////////////////////
//////////* Inverse Kinematics *//////////
//////////////////////////////////////////

/* Given position of end effector Px, Py, Pz calculate angles Thetax, Thetay, Thetaz */
void PositionGiven(float Px, float Py, float Pz) {  /* possible positions px=py=(-0.9,0.9), pz=(11.2,11.5)*/

	if(Pz==0){
		Thetax = PI/2;
    		Thetay = 0;
    		Thetaz = atan2(Py,Px);
	}
	else {
		Thetax = atan2(sqrt(pow(Px,2)+pow(Py,2)), Pz);
    		Thetaz = atan2(Px, Py);
    		Thetay = 0;
	}

}

/* Given orientation of end effector U13, U23, U31, U32, U33 calculate angles Thetax, Thetay, Thetaz */
void OrientationGiven(float U11, float U12, float U13, float U23, float U31, float U32, float U33) {

	if(U33==1 || U33==-1){
		Thetax = 0;
    		Thetay = 0;
    		Thetaz = atan2(U11,-U12);
	}	
	else {
		Thetax = atan2((U33),(sqrt(1-pow(U33,2))));
		if(sin(Thetax>0)){
    			Thetaz = atan2(U13,U23);
    			Thetay = atan2(-U31,U32);
		}
		else {
    			Thetaz = atan2(-U13,-U23);
    			Thetay = atan2(U31,-U32);
		}

	}

}

//////////////////////////////////////////
///////* Kinematic Transformation *///////
//////////////////////////////////////////

/* Given angles Thetax, Thetay, Thetaz calculate tendons lengths Lx, Ly, Lz */
void CalculateLengths(float Thetax, float Thetay, float Thetaz) {

	Thetax = (degreex*PI)/180;
	Thetay = (degreey*PI)/180;

    float Lxx, Lxy, Lyx, Lyy, Dxx, Dxy, Dyx, Dyy, Lx1, Lx2, Ly1, Ly2;
    Lxx=sqrt(pow((9-(5*cos(Theta0-fabs(Thetax)))),2)+pow(((5*sin(Theta0-fabs(Thetax)))-2.5),2))-0.045573;
    Dxx=fabs(Lxx-7);
    Lxy=sqrt(pow((9.25-2.25*cos(fabs(Thetay))),2)+pow((2.25*sin(fabs(Thetay))),2));
    Dxy=fabs(Lxy-7);
    Lyy=sqrt(pow((9-(5*cos(Theta0-fabs(Thetay)))),2)+pow(((5*sin(Theta0-fabs(Thetay)))-2.5),2))-0.045573;
    Dyy=fabs(Lyy-7);
    Lyx=sqrt(pow((9.25-2.25*cos(fabs(Thetax))),2)+pow((2.25*sin(fabs(Thetax))),2));
    Dyx=fabs(Lyx-7);
    Lx1=Dxy+Dxx;
    Lx2=Dxy-Dxx;
    Ly1=Dyx+Dyy;
    Ly2=Dyx-Dyy;
    
/* Given tendons lengths Lx, Ly, Lz calculate motor rotations M1, M2, M3 */

	float k1 = 2.0;
	float k2 = 2.0;
	float k3 = 2.0;
	float k4 = 2.0;
	float rmotor = 4;
	float umotor1, umotor2, umotor3, umotor4, umotor6;
	umotor1 = (Lx1*180)/(PI*rmotor);
	umotor3 = (Lx2*180)/(PI*rmotor);
	umotor2 = (Ly1*180)/(PI*rmotor);
	umotor4 = (Ly2*180)/(PI*rmotor);
	umotor6 = degreez;
	printf("To open the gripper press 2 \n To close the gripper press 3 \n To continue press any other key\n");
	int gripper;
	scanf ("%d",&gripper);
	if(gripper == 2)
		M5 = M5o;
	else
		if(gripper == 3)
			M5 = M5c;
		else
			M5 = 1000;
	if(Thetax>0)
	{
		k1 = 2.30;
		k2 = 2.30;
		if(Thetax>20)
		{
			k1 = 2.80;
			k2 = 2.80;
		}
		M1 = M1o - static_cast<int>(k1*(4096*umotor1)/360);
		M3 = M3o - static_cast<int>(k2*(4096*umotor3)/360);		
	}
	else
	{
		k1 = 1.98;
		k2 = 2.00;
		if(Thetax<-20)
		{
			k1 = 2.35;
			k2 = 2.30;
		}
		M1 = M1o + static_cast<int>(k1*(4096*umotor1)/360);
		M3 = M3o + static_cast<int>(k2*(4096*umotor3)/360);
	}
	if(Thetay>0)
	{
		k3 = 2.30;
		k4 = 2.30;
		if(Thetay>20)
		{
			k3 = 2.90;
			k4 = 2.90;
		}
		M2 = M2o + static_cast<int>(k3*(4096*umotor2)/360);
		M4 = M4o + static_cast<int>(k4*(4096*umotor4)/360);
	}
	else
	{
		k3 = 2.25;
		k4 = 2.25;
		if(Thetay>20)
		{
			k3 = 2.90;
			k4 = 2.90;
		}
		M2 = M2o - static_cast<int>(k4*(4096*umotor2)/360);
		M4 = M4o - static_cast<int>(k3*(4096*umotor4)/360);
	}
	M6 = M6o + static_cast<int>((4096*umotor6)/360);
	
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  CalculateLengths(0, 0, 0);
  
  int index = 1;
  int dxl_comm_result1 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result2 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result3 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result4 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result5 = COMM_TX_FAIL;             // Communication result
  int dxl_comm_result6 = COMM_TX_FAIL;             // Communication result

  int dxl_goal_position1 = M1;         // Goal position
  int dxl_goal_position2 = M2;         // Goal position
  int dxl_goal_position3 = M3;         // Goal position
  int dxl_goal_position4 = M4;         // Goal position
  int dxl_goal_position5 = M5;         // Goal position
  int dxl_goal_position6 = M6;         // Goal position

  uint8_t dxl_error1 = 0;                          // Dynamixel error
  uint8_t dxl_error2 = 0;                          // Dynamixel error
  uint8_t dxl_error3 = 0;                          // Dynamixel error
  uint8_t dxl_error4 = 0;                          // Dynamixel error
  uint8_t dxl_error5 = 0;                          // Dynamixel error
  uint8_t dxl_error6 = 0;                          // Dynamixel error

  int32_t dxl_present_position1 = 0;               // Present position
  int32_t dxl_present_position2 = 0;               // Present position
  int32_t dxl_present_position3 = 0;               // Present position
  int32_t dxl_present_position4 = 0;               // Present position
  int32_t dxl_present_position5 = 0;               // Present position
  int32_t dxl_present_position6 = 0;               // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set operating mode to extended position control mode
  dxl_comm_result1 = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error1);
  dxl_comm_result2 = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error2);
  dxl_comm_result3 = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error3);
  dxl_comm_result4 = packetHandler->write1ByteTxRx(portHandler, DXL_ID4, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error4);
  dxl_comm_result5 = packetHandler->write1ByteTxRx(portHandler, DXL_ID5, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error5);
  dxl_comm_result6 = packetHandler->write1ByteTxRx(portHandler, DXL_ID6, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error6);
  if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
  }
  else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
  }
  else
  {
    printf("Operating mode changed to extended position control mode. \n");
  }

  // Enable Dynamixel Torque
  dxl_comm_result1 = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error1);
  dxl_comm_result2 = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error2);
  dxl_comm_result3 = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error3);
  dxl_comm_result4 = packetHandler->write1ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error4);
  dxl_comm_result5 = packetHandler->write1ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error5);
  dxl_comm_result6 = packetHandler->write1ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error6);
  if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
  {
    // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
  }
  else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
  {
    // printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  while(1)
  {
    printf("Press any key to continue! (or press ESC to quit!)\n");
    if (getch() == ESC_ASCII_VALUE)
      break;

    // Write goal position
    dxl_comm_result1 = packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION, dxl_goal_position1, &dxl_error1);
    dxl_comm_result2 = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_goal_position2, &dxl_error2);
    dxl_comm_result3 = packetHandler->write4ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_GOAL_POSITION, dxl_goal_position3, &dxl_error3);
    dxl_comm_result4 = packetHandler->write4ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_GOAL_POSITION, dxl_goal_position4, &dxl_error4);
    dxl_comm_result5 = packetHandler->write4ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_GOAL_POSITION, dxl_goal_position5, &dxl_error5);
    dxl_comm_result6 = packetHandler->write4ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_GOAL_POSITION, dxl_goal_position6, &dxl_error6);
    if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
    {
      printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
    }
    else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
    {
      printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
    }

    do
    {
      // Read present position
      dxl_comm_result1 = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position1, &dxl_error1);
      dxl_comm_result2 = packetHandler->read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position2, &dxl_error2);
      dxl_comm_result3 = packetHandler->read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position3, &dxl_error3);
      dxl_comm_result4 = packetHandler->read4ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position4, &dxl_error4);
      dxl_comm_result5 = packetHandler->read4ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position5, &dxl_error5);
      dxl_comm_result6 = packetHandler->read4ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position6, &dxl_error6);
      if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
      }
      else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
      }

      printf("[ID1:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID1, dxl_goal_position1, dxl_present_position1);
      printf("[ID2:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID2, dxl_goal_position2, dxl_present_position2);
      printf("[ID3:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID3, dxl_goal_position3, dxl_present_position3);
      printf("[ID4:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID4, dxl_goal_position4, dxl_present_position4);
      printf("[ID5:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID5, dxl_goal_position5, dxl_present_position5);
      printf("[ID6:%03d] GoalPos1:%03d  PresPos1:%03d\n", DXL_ID6, dxl_goal_position6, dxl_present_position6);

if (kbhit())
      {
        char c = getch();
        if (c == SPACE_ASCII_VALUE)
        {
          printf("\n  Stop & Clear Multi-Turn Information! \n");

          // Write the present position to the goal position to stop moving
          dxl_comm_result1 = packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION, dxl_present_position1, &dxl_error1);
          dxl_comm_result2 = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_present_position2, &dxl_error2);
          dxl_comm_result3 = packetHandler->write4ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_GOAL_POSITION, dxl_present_position3, &dxl_error3);
          dxl_comm_result4 = packetHandler->write4ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_GOAL_POSITION, dxl_present_position4, &dxl_error4);
          dxl_comm_result5 = packetHandler->write4ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_GOAL_POSITION, dxl_present_position5, &dxl_error5);
          dxl_comm_result6 = packetHandler->write4ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_GOAL_POSITION, dxl_present_position6, &dxl_error6);
          if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
          {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
          }
          else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
          {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
          }

          usleep(300);

          // Clear Multi-Turn Information
          dxl_comm_result1 = packetHandler->clearMultiTurn(portHandler, DXL_ID1, &dxl_error1);
          dxl_comm_result2 = packetHandler->clearMultiTurn(portHandler, DXL_ID2, &dxl_error2);
          dxl_comm_result3 = packetHandler->clearMultiTurn(portHandler, DXL_ID3, &dxl_error3);
          dxl_comm_result4 = packetHandler->clearMultiTurn(portHandler, DXL_ID4, &dxl_error4);
          dxl_comm_result5 = packetHandler->clearMultiTurn(portHandler, DXL_ID5, &dxl_error5);
          dxl_comm_result6 = packetHandler->clearMultiTurn(portHandler, DXL_ID6, &dxl_error6);
          if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
          {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
          }
          else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
          {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
          }

          // Read present position
          dxl_comm_result1 = packetHandler->read4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position1, &dxl_error1);
          dxl_comm_result2 = packetHandler->read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position2, &dxl_error2);
          dxl_comm_result3 = packetHandler->read4ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position3, &dxl_error3);
          dxl_comm_result4 = packetHandler->read4ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position4, &dxl_error4);
          dxl_comm_result5 = packetHandler->read4ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position5, &dxl_error5);
          dxl_comm_result6 = packetHandler->read4ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position6, &dxl_error6);
          if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
          {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
          }
          else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
          {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
          }

              printf("  Present Position has been reset. : %03d \n", dxl_present_position1);
	      printf("  Present Position has been reset. : %03d \n", dxl_present_position2);
	      printf("  Present Position has been reset. : %03d \n", dxl_present_position3);
	      printf("  Present Position has been reset. : %03d \n", dxl_present_position4);
	      printf("  Present Position has been reset. : %03d \n", dxl_present_position5);
	      printf("  Present Position has been reset. : %03d \n", dxl_present_position6);

          break;
        }
        else if (c == ESC_ASCII_VALUE)
        {
          printf("\n  Stopped!! \n");

          // Write the present position to the goal position to stop moving
          dxl_comm_result1 = packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_GOAL_POSITION, dxl_present_position1, &dxl_error1);
          dxl_comm_result2 = packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_GOAL_POSITION, dxl_present_position2, &dxl_error2);
          dxl_comm_result3 = packetHandler->write4ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_GOAL_POSITION, dxl_present_position3, &dxl_error3);
          dxl_comm_result4 = packetHandler->write4ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_GOAL_POSITION, dxl_present_position4, &dxl_error4);
          dxl_comm_result5 = packetHandler->write4ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_GOAL_POSITION, dxl_present_position5, &dxl_error5);
          dxl_comm_result6 = packetHandler->write4ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_GOAL_POSITION, dxl_present_position6, &dxl_error6);
          if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
          {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
          }
          else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
          {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
          }

          break;
        }
      }

    }while((abs(dxl_goal_position1 - dxl_present_position1) > DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position2 - dxl_present_position2) > DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position3 - dxl_present_position3) > DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position4 - dxl_present_position4) > DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position5 - dxl_present_position5) > DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position6 - dxl_present_position6) > DXL_MOVING_STATUS_THRESHOLD));

    // Change goal position
    if (index<=10)
    {
    	if ((abs(dxl_goal_position1 - dxl_present_position1) < DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position2 - dxl_present_position2) < DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position3 - dxl_present_position3) < DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position4 - dxl_present_position4) < DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position5 - dxl_present_position5) < DXL_MOVING_STATUS_THRESHOLD) && (abs(dxl_goal_position6 - dxl_present_position6) < DXL_MOVING_STATUS_THRESHOLD))
		{
			index++;
			printf("Give goal position! \n");
			printf("Thetax = ");
			scanf ("%f", &degreex);
			printf(" \n Thetay = ");
			scanf ("%f", &degreey);
			printf(" \n Thetaz = ");
			scanf ("%f", &degreez);
			CalculateLengths(degreex, degreey, degreez);
			dxl_goal_position1 = M1;         // Goal position
  			dxl_goal_position2 = M2;         // Goal position
  			dxl_goal_position3 = M3;         // Goal position
  			dxl_goal_position4 = M4;         // Goal position
  			dxl_goal_position5 = M5;         // Goal position
  			dxl_goal_position6 = M6;         // Goal position
		}
	}
    
  }

  // Disable Dynamixel Torque
  dxl_comm_result1 = packetHandler->write1ByteTxRx(portHandler, DXL_ID1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error1);
  dxl_comm_result2 = packetHandler->write1ByteTxRx(portHandler, DXL_ID2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error2);
  dxl_comm_result3 = packetHandler->write1ByteTxRx(portHandler, DXL_ID3, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error3);
  dxl_comm_result4 = packetHandler->write1ByteTxRx(portHandler, DXL_ID4, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error4);
  dxl_comm_result5 = packetHandler->write1ByteTxRx(portHandler, DXL_ID5, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error5);
  dxl_comm_result6 = packetHandler->write1ByteTxRx(portHandler, DXL_ID6, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error6);
  if (dxl_comm_result1 != COMM_SUCCESS && dxl_comm_result2 != COMM_SUCCESS && dxl_comm_result3 != COMM_SUCCESS && dxl_comm_result4 != COMM_SUCCESS && dxl_comm_result5 != COMM_SUCCESS && dxl_comm_result6 != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result1));
  }
  else if (dxl_error1 != 0 || dxl_error2 != 0 || dxl_error3 != 0 || dxl_error4 != 0 || dxl_error5 != 0 || dxl_error6 != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error1));
  }

  // Close port
  portHandler->closePort();

  return 0;
}
