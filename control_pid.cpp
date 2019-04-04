#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h" //x,y,theta float 64
#include </opt/ros/kinetic/include/geometry_msgs/Point.h>//x,y,z loat 64
#include "iostream"

#define clkw        0
#define c_clkw      1
#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG       1

/**----------------pid position calculator------------------------------------------**/
/* 
 * pid for motor:
 * period time: 20 ms
 * peek time: 400 ms
 * max speed (setting): value: 25/0.02 = 1250 pulses/s = 981.75 mm/s  <vm = vp*65*pi/wheels_encoder>
 * smallest speed (runable speed): value: 5/0.02= 250 pulses/s = 196.35mm/s
*/


/**--------------------Control signal variable-----------------------------------------------------------------------------------**/
volatile int ang_vel=0,lin_vel=0;
double l_v,l_vt;
double r_v,r_vt; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir=clkw, r_dir=clkw;
/**-----------------------pid velocity calculation-------------------------------------------**/
volatile double  l_error=0.0,l_pre_error=0.0,l_integral=0.0,l_derivative=0.0,l_Ppart=0.0,l_Ipart=0.0,l_Dpart=0.0,l_out,l_set,l_ms,l_pre_out=0;
double const l_kP = 0.72, l_kI=25.205 ,l_kD = 0.005;
volatile double  r_error=0.0,r_pre_error=0.0,r_integral=0.0,r_derivative=0.0,r_Ppart=0.0,r_Ipart=0.0,r_Dpart=0.0,r_out,r_set,r_ms,r_pre_out=0;
double const r_kP = 0.95, r_kI=25.23,r_kD = 0.008;
/**--------------------------car parameter-----------------------------------------------**/
const double pi=3.1415;
const double sampletime = 0.02, inv_sampletime = 1/sampletime,timer_set=65535-sampletime*250000;
const double wheels_distance = 207, wheels_radius = 31, wheels_diameter=62,wheels_encoder = 430;// mm
const double wheel_ticLength = wheels_diameter*pi/wheels_encoder;
const bool l_motor=1,r_motor=0;
/*--------------------------position calculation----------------------------*/
double p_org[]={0.0,0.0,0.0}, p_now[]= {0.0,0.0,0.0}; //{x,y,phi}
int l_p=0,r_p=0;
double l_d=0,r_d=0;

//time period for connecting
//int pos_sampleTime=50;
//long int setting_millis=millis()+pos_sampleTime;

/*--------------------------------------------define function------------------------------------------*/
void calculate_position(double,double , double);
void motion(double,double);
double PID_cal(double,double,double,double,double,double,double,double,double,double);
void pwmOut(int,int,bool,bool);
void encoder_1();
void encoder_2(); 

/*----------------------Subscriber callBack-----------------------------------------------*/
void messageCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  lin_vel=vel->linear.x*1000;
  ang_vel=vel->angular.z*180/pi;
}
/**/
/*----------------------------------------main function--------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_pid");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/turtle1/cmd_vel",10, messageCallback );
  geometry_msgs::Pose2D *postef;
  geometry_msgs::Point *lin_ang;
  ros::Publisher pos_temp  = nh.advertise<geometry_msgs::Pose2D>("position",10);
  ros::Publisher test_temp = nh.advertise<geometry_msgs::Point>("velocity",10);
  while(ros::ok())
  {
    postef->x     = p_now[0];
    postef->y     = p_now[1];
    postef->theta = p_now[2]; 
    pos_temp.publish(*postef);
      
    lin_ang->x = r_v;
    lin_ang->y = l_v;
    test_temp.publish(*lin_ang);
       
    //setting_millis=millis() + pos_sampleTime;  
    //delay(pos_sampleTime/2);
  }
  ros::spinOnce();
  return 0;
}
/*-------------------encoder interrupt 1 ---------------------------------------*/
void encoder_1()
{
  if(!l_dir) l_p ++;
  else l_p--;
}
/*----------------------encoder interrupt 2 ------------------------------------*/
void encoder_2()
{  
  if(!r_dir) r_p ++;
  else r_p--;
}
/*--------------------generarte pwm-----------------------------------*/
void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir)
{
  if(Lpwm==0 && Rpwm==0)
  { 
   // analogWrite(M1_p,0); digitalWrite(M1_l,0);
   // analogWrite(M2_p,0); digitalWrite(M2_l,0);
  }
  else if(Ldir==clkw && Rdir==clkw)
  {
   // analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
   // analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }

  else if(Ldir==c_clkw && Rdir==c_clkw)
  {
    // analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
   // analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0) ;  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
   // analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
   // analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
   // analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
   // analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}
/*-------------------calculate_position------------------------------------*/
void calculate_position(double xt,double yt, double pht)
{
  l_d = l_p*wheel_ticLength;
  r_d = r_p*wheel_ticLength;
  double c_d = (l_d + r_d )/2;

  xt += c_d*cos(pht);
  yt += c_d*sin(pht);
  pht += atan2((r_d-l_d),wheels_distance);

  //update position
  p_now[0]=xt;
  p_now[1]=yt;
  p_now[2]=pht;
}
/*----------Calculate from angle and linear to motion of 2 wheels--------------------------------------*/
void motion(double lin, double phi )
{
  r_v = (2*lin - phi*wheels_distance)/(2.0); //speed of right wheels  
  l_v = (2*lin + phi*wheels_distance)/(2.0);  //speed of left wheels
  //to l_vt and r_vt
  l_vt = l_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  r_vt = r_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;

  if (l_vt>=0) l_dir = clkw;//go ahead
  else l_dir =c_clkw;       //backhead
  if (r_vt>=0) r_dir = clkw;
  else r_dir =c_clkw;
  
  l_set=abs(l_vt);
  r_set=abs(r_vt);
  if (l_set>40) l_set=40;
  else if (l_set<4 && l_set>0.5) l_set=4 ;
  if (r_set>40) r_set=40;
  else if (r_set<4 && r_set>0.5) r_set =4;
}
/**-------------Calculate PID----------------------------------------------------------------------**/
double PID_cal(double error,double pre_error,double _integral,double _derivative,double Ppart,double Ipart,double Dpart,double Kp,double Ki, double Kd)
{
    //PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
    //PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
    //PID_cal(ang_error,ang_pre_error,ang_integral,ang_derivative,ang_Ppart,ang_Ipart,ang_Dpart,p_kP,p_kI,p_kD);
    //PID_cal(lin_error,lin_pre_error,lin_integral,lin_derivative,lin_Ppart,lin_Ipart,lin_Dpart,p_kP,p_kI,p_kD);
    double PID_output;
    Ppart = Kp*error;

    _integral += error * sampletime;
    Ipart = Ki*_integral;

    _derivative  = (error  - pre_error )*inv_sampletime;
    Dpart  = Kd *_derivative ;

    PID_output = Ppart  + Ipart  + Dpart  ;
    pre_error  = error ;
    
    return PID_output;
}
/**-------------------------------------------------------------**/