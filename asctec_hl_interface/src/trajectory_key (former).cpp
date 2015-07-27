#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sys/time.h>


#define KEYCODE_a  0x61
#define KEYCODE_b  0x62
#define KEYCODE_c  0x63  
#define KEYCODE_q  0x71


class Desired
{
public:
  Desired();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double h_a_, h_b_, h_c_, xy_max_, z_max_, mav_x, mav_y, mav_z;
  ros::Publisher ctrl_pub_;
  
};

Desired::Desired():
  h_a_(1.0),
  h_b_(2.0),
  h_c_(0.5),
  mav_x(0),
  mav_y(0),
  mav_z(0),
  xy_max_(0.2),
  z_max_(0.2)
{
  nh_.param("heigh_take_off", h_a_, h_a_);
  nh_.param("heigh_trajectory", h_b_, h_b_);
  nh_.param("heigh_landing", h_c_, h_c_);

  ctrl_pub_ = nh_.advertise<asctec_hl_comm::mav_ctrl>("/fcu/control", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Desired_trajectory");
  Desired mav1;

  signal(SIGINT,quit);

  mav1.keyLoop();
  
  return(0);
}


void Desired::keyLoop()
{
  char c;
  struct  timeval timeout={0,10000};
  int state;
  fd_set readfds;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use a,b,c,q !!!");


  mav_x=mav_y=mav_z=0;

  for(;;)
  {
    // get the next event from the keyboard  
    // there seems to be some error in timeout & timeout........
    FD_SET(kfd, &readfds);
    state = select(kfd+1, &readfds, NULL, NULL, &timeout);
    switch(state)
    {
      case -1 : ROS_DEBUG("error"); break;

      case 0 :  ROS_DEBUG("keep going"); break;

      default:
        read(kfd, &c, 1);
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
        // command for position
        case KEYCODE_a:
          ROS_DEBUG("a : take off");
          puts("a : take off");
          mav_x = 0;
          mav_z = h_a_;
          break;
        case KEYCODE_b:
          ROS_DEBUG("b : trajectory");
          puts("b : trajectory");
          mav_x = 0.5;
          mav_z = h_b_;
          break;
        case KEYCODE_c:
          ROS_DEBUG("c : landing");
          puts("c : landing");
          mav_x = 0;
          mav_z = h_c_;
          break;
        case KEYCODE_q:
          ROS_DEBUG("q :quit");
          puts("q :quit");
          mav_x = 0;
          mav_z = 0;
          break;
        default:
          puts("!!!!!!!!press a,b,c,q!!!!!!!!");
          break;
        }
      break; 
    }
    usleep(50000);//20Hz
   

    asctec_hl_comm::mav_ctrl ctrl;
    ctrl.header.frame_id ="/world";
    ctrl.header.stamp = ros::Time::now();

    ctrl.type=3;// type1:acc // type2:vel // type3:pos

    ctrl.x=mav_x;
    ctrl.y=0;
    ctrl.z=mav_z;
    ctrl.yaw=0;
    ctrl.v_max_xy=xy_max_;
    ctrl.v_max_z = z_max_;
    ctrl_pub_.publish(ctrl);
  }


  return;
}
