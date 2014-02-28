/*
When running this controller in the real robot
do not interface via SSh client (i,e, PuTTY)
The result will be on a segmentation fault error.

Instead interface with the robot via remote desktop
(i.e. VNC). Open a terminal window, compile controller
and run.
*/
#include "walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>
#include "LinuxDARwIn.h"
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#define  BUFF_SIZE   4

// IP ------------------------
// #include "stdafx.h"
 
 #include <stdio.h>
 
 #include <unistd.h>
 #include <netdb.h>
 #include <string.h>
 #include <sys/socket.h>
 #include <netinet/in.h>
 #include <arpa/inet.h>
 #include <sys/ioctl.h>
 #include <net/if.h>
// IP -----------------------------

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

char  szIp[16]; // IP address

Walk::Walk():
    Robot()
{
  mTimeStep = getBasicTimeStep();
  LinuxActionScript::PlayMP3("./init_1.mp3");
  walkType = -1;//jh new
  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  printf("11");
  getGyro("Gyro")->enable(mTimeStep);
  
  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(mTimeStep);
  }
  
  //keyboardEnable(mTimeStep);
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
  
}

Walk::~Walk() {
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void* Walk::t_function(void* data){
  Walk* self = (Walk*) data;
  return (void *)(self->real_thread());
}

int* Walk::real_thread(){
  
  while(isWalking==true){
    checkIfFallen();
    mGaitManager->setXAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    switch(walkType){
      case 0://stop
        //cout << "Receive : Walking stop //" << endl;
        mGaitManager->stop(); 
        break;
      case 1://start
        //cout << "Receive : Walking ready //" << endl;
        mGaitManager->start(); 
        break;
      case 2://forward
        mGaitManager->setXAmplitude(1.0);
        break;
      case 3://back
        mGaitManager->setXAmplitude(-1.0);
        break;
      case 4://left
        mGaitManager->setAAmplitude(0.5);
        break;
      case 5://right
        mGaitManager->setAAmplitude(-0.5);
        break;
      default:
        mGaitManager->stop(); 
        walkType = 0;
        break;
    }
    mGaitManager->step(mTimeStep);
    myStep();
  }
  mGaitManager->stop(); 
  mGaitManager->step(mTimeStep);
  // step
  myStep(); 
  return 0;
}

void Walk::searchIP(){
   struct ifconf ifc;
   
   int s = socket( AF_INET, SOCK_DGRAM, 0 );
   int len = 100 * sizeof(struct ifreq);
   
   char buf[ len ];
   
   ifc.ifc_len = len;
   ifc.ifc_buf = buf;
   
   int e = ioctl(s,SIOCGIFCONF,&ifc);
   char *ptr = buf;
   int tl = ifc.ifc_len;
   //char  szIp[16];
   
   while ( tl > 0 )
   {
     struct ifreq* ifr = (struct ifreq *)ptr;
     int si;
     
     si = sizeof(ifr->ifr_name) + sizeof(struct sockaddr);
 
     tl -= si;
     ptr += si;
     
     struct ifreq ifr2;
     ifr2 = *ifr;
     
     e = ioctl(s,SIOCGIFADDR,&ifr2);
     if( e == -1 )
     {
        continue;
     }
     
     struct sockaddr a = ifr2.ifr_addr;
     struct sockaddr_in* addr = (struct sockaddr_in*) &a;
     
     unsigned int ai = ntohl( addr->sin_addr.s_addr );
     // 127.0.0.1 주소는 출력하지 않는다.
     if( int((ai>>24)&0xFF) == 127 ) continue;
 
     snprintf( szIp, sizeof(szIp), "%d.%d.%d.%d", int((ai>>24)&0xFF)
       , int((ai>>16)&0xFF)
       , int((ai>> 8)&0xFF) 
       , int((ai    )&0xFF) );
     //printf( "[%s]\n", szIp );
   }
   
   //close(s);
}

void Walk::conSocket(){
  int   client_socket;
  struct sockaddr_in   server_addr;
  int   buff_rcv;
  char   org_buff_rcv[BUFF_SIZE+5];
  //bool walkType = -1;
  pthread_t p_thread;
  int status;
  int thr_id;
  client_socket  = socket( PF_INET, SOCK_STREAM, 0);
  printf("22");
  if( -1 == client_socket){
    printf( "socket initialize fale\n");
    exit( 1);
  }else{
    printf("socket initialize complete\n");
  }
  memset( &server_addr, 0, sizeof( server_addr));
  server_addr.sin_family     = AF_INET;
  server_addr.sin_port       = htons( 8888 );
  server_addr.sin_addr.s_addr= inet_addr( "128.210.171.227");

  if( connect( client_socket, (struct sockaddr*)&server_addr, sizeof( server_addr) ) == -1 ){
    printf( "Access fail\n");
    exit( 1);
  }else{
    printf("Access Server complete\n");
  }
  int recv_flag = true;
  isWalking = true; 
  // First step to update sensors values
  myStep();
  // play the hello motion
  mMotionManager->playPage(9); // init position
  thr_id = pthread_create(&p_thread, NULL, t_function, this);
  if (thr_id < 0){
    perror("thread create error : ");
    exit(0);
  }else{
    printf("thread create success : \n");
  }
    searchIP();	
    write( client_socket, &szIp, sizeof(szIp));          // +1: To NULL
    // In order to send to Server.
  
  stime = 0;
  while(recv_flag>-1){
    printf("waiting...\n");
    read ( client_socket, &org_buff_rcv, BUFF_SIZE);
    getLED("HeadLed")->set(0x1DDB16);
    getLED("EyeLed")->set(0x1DDB16);
    if( ((int)org_buff_rcv[0]-48)>-1 && ((int)org_buff_rcv[0]-48) <6)
      buff_rcv = (int)org_buff_rcv[0]-48;
    else buff_rcv = -1;
    printf( "receive: %d - ", buff_rcv);
    recv_flag = buff_rcv;
    if(buff_rcv==-1){
      
      getLED("HeadLed")->set(0x0100FF);
      getLED("EyeLed")->set(0x0100FF);
      cout << "Receive : System end //" << endl;
      mGaitManager->setXAmplitude(0.0);
      mGaitManager->setAAmplitude(0.0);
      isWalking=false;
      recv_flag=-1;
    }//end process
    else{
	  if(stime==0){
	   stime++;
	   LinuxActionScript::PlayMP3("./init_2.mp3");
	   wait(1200);
	  }
      walkType = buff_rcv;
      getLED("HeadLed")->set(0xFFBB00);
      getLED("EyeLed")->set(0xFF0000);
    } 
    write( client_socket, &buff_rcv, sizeof(buff_rcv));          // +1: To NULL
  }
  pthread_join(p_thread, (void **)&status);
  printf("socket close\n");
  close( client_socket); 
}//socket function



void Walk::checkIfFallen() {
  static int fup = 0;
  static int fdown = 0;
  static const double acc_tolerance = 80.0;
  static const double acc_step = 100;
  
  // count how many steps the accelerometer
  // says that the robot is down
  const double *acc = mAccelerometer->getValues();
  if (acc[1] < 512.0 - acc_tolerance)
    fup++;
  else
    fup = 0;
  
  if (acc[1] > 512.0 + acc_tolerance)
    fdown++;
  else
    fdown = 0;
  
  // the robot face is down
  if (fup > acc_step) {
    mMotionManager->playPage(10); // f_up
    mMotionManager->playPage(9); // init position    
    fup = 0;
  }
  // the back face is down
  else if (fdown > acc_step) {
    mMotionManager->playPage(11); // b_up
    mMotionManager->playPage(9); // init position
    fdown = 0;
  }
}
