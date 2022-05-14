#define _USE_MATH_DEFINES

#include "../matplotlibcpp.h"
#include <vector>
#include <math.h>
#include <cmath>

//Introduction to Self-Driving Cars
//Module 4: Vehicle Dynamic Modeling
//Ex Kinematic Bicycle Model Part 1

namespace plt = matplotlibcpp;



class Bicycle_State_Machine
{
  public:
  Bicycle_State_Machine();
  void robot_state(double vel,double roc_steer);
  void init(double wheel_base,double w_max,double lr);
  void reset();
  void set_delta(double steering_angle);
  void set_sample_time(double sample_time);
  double get_global_x(); //get global x
  double get_global_y(); //get global y
  void debugger_printing();


  private:
  double _xc_dot; //next state x,y coordinates
  double _yc_dot;
  double _theta_dot =0; 
  double _delta_dot =0; //steering angle of vehicle next step
  

  //Self
  double _xc_self;
  double _yc_self;
  double _theta_self;
  double _delta_self; //steering angle of vehicle
  double _beta_self;

  //vehicle properties
  double _wheel_base;
  double _lr;   //length to rear axle
  double _w_max; //maxiumum steering angle
  double _vel;
  double _roc_steer; //rate of angular changes

  //timer
  double _sample_time_self =0; 

};

Bicycle_State_Machine::Bicycle_State_Machine() //wheel base length 2m , maximum turning radius 1.22rad/s , length of 1.2m to center of mass rear axle
{

}

void Bicycle_State_Machine::robot_state(double vel,double roc_steer)
{
  //modify to python code
  //projecting future state
  _vel = vel;
  _xc_dot = _vel *cos(_theta_self + _beta_self); //next state
  _yc_dot = _vel *sin(_theta_self + _beta_self);
  _theta_dot= (_vel *cos(_beta_self) * tan(_delta_self))/_wheel_base;
  _delta_dot = std::max(-_w_max,std::min(_w_max,_roc_steer));
  //updating to current state
  _xc_self += _xc_dot * _sample_time_self;
  _yc_self += _yc_dot * _sample_time_self;
  _theta_self += _theta_dot * _sample_time_self; 
  //_delta_self = _delta_dot * self.sample_time; //rate of change of steering / sampling time 
  _delta_self = _delta_dot;  //preset permanamet steering angle to drive in a circle
  _beta_self = atan2((_lr * tan(_delta_self)),_wheel_base);

}

void Bicycle_State_Machine::reset()
{
  _xc_self =0;
  _yc_self =0;
  _theta_self =0;
  _delta_self =0;
  _beta_self =0;

}

void Bicycle_State_Machine::init(double wheel_base,double w_max,double lr)
{
  _wheel_base = wheel_base;
  _w_max = w_max;
  _lr = lr;
  _vel = 0;
}

void Bicycle_State_Machine::set_delta(double steering_angle)
{
  _roc_steer = steering_angle;
}


double Bicycle_State_Machine::get_global_x()
{
    return _xc_self ;
}

double Bicycle_State_Machine::get_global_y()
{
    return _yc_self ;
}

void Bicycle_State_Machine::set_sample_time(double sample_time)
{
    _sample_time_self = sample_time;

}

void Bicycle_State_Machine::debugger_printing()
{
  std::cout<<"*******************************"<<std::endl;
  std::cout<<"_vel "<<_vel<<std::endl;
  std::cout<<" _xc_self :"<<_xc_self<<std::endl;
  std::cout<<" _yc_self :"<<_yc_self<<std::endl;
  std::cout<<" _theta_self :"<<_theta_self<<std::endl;
  std::cout<<" _delta_self :"<<_delta_self<<std::endl;
  std::cout<<" _beta_self :"<<_beta_self<<std::endl;
  std::cout<<"_theta_dot "<<_theta_dot<<std::endl;
  std::cout<<"_delta_dot "<<_delta_dot<<std::endl;
  std::cout<<" _beta_self "<<_beta_self<<std::endl;
  std::cout<<"*******************************"<<std::endl;
}


int main() {


  Bicycle_State_Machine bicycle;
  bicycle.reset();
  bicycle.init(2.0,1.22,1.2); //w_max can be changed
  //1 step
  bicycle.set_delta(0.1974);
  bicycle.set_sample_time(0.01);
  

  

  //setting time and speed
  


  /*Example*/

  // Prepare data.
    int n = 2000 ; //20s/0.01s = 20000 cycle
   std::vector<double> x(n), y(n), z(n), w(n,2);
    for(int i=0; i<n; ++i) {
        x.at(i) = bicycle.get_global_x();   //get x,y coordinates
        y.at(i) = bicycle.get_global_y(); 
        //bicycle.debugger_printing();
        bicycle.robot_state(M_PI,0);
        
    }

    plt::figure_size(1200, 780);
    plt::plot(x, y);
    plt::title("Bicycle Kinematic model");
    plt::show();


}
