#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42)
        {
          *c = KEYCODE_B;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43)
        {
          *c = KEYCODE_C;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
        {
          *c = KEYCODE_E;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46)
        {
          *c = KEYCODE_F;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47)
        {
          *c = KEYCODE_G;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
        {
          *c = KEYCODE_R;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
        {
          *c = KEYCODE_T;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56)
        {
          *c = KEYCODE_V;
          return;
        }
	else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x73)
        {
          *c = KEYCODE_S;
          return;
        }
	else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x77)
        {
          *c = KEYCODE_W;
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_x_,linear_y_,linear_z_, angular_, l_scale_, a_scale_;
  //ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_x_(0),
  linear_y_(0),
  linear_z_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  //twist_pub_ = nh_.advertise<geometry_msgs::Twist>("navigate_map_key/cmd_vel", 1);
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigate_map_key_node");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  quit(0);
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle. 'q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }
    double linear_x_ant = linear_x_ant; 
    double linear_y_ant = linear_y_ant;
    double linear_z_ant = linear_z_ant;

    linear_x_ = 0; 
    linear_y_= 0;
    angular_ = angular_;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        linear_y_ += 0.05;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        linear_y_ += -0.05;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("FORWARD");
        linear_x_ += 0.05;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("BACK");
        linear_x_ += -0.05;
        dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("UP");
        linear_z_ant += 0.05;
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("DOWN");
        linear_z_ant += -0.05;
        dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("ROTATE+");
        angular_ += 0.01;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("ROTATE-");
        angular_ += -0.01;
        dirty = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("quit");
        return;
    }
   

    /*geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_x_;
    twist.linear.y = l_scale_*linear_y_;
    twist.linear.z = l_scale_*linear_z_;*/

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    double x_comp_ant = x_comp_ant;
    double y_comp_ant = y_comp_ant;
    double x_proy = std::cos(angular_)*(linear_x_) - std::sin(angular_)*(linear_y_);
    double y_proy = std::sin(angular_)*(linear_x_) + std::cos(angular_)*(linear_y_);
    double x_comp = x_proy + linear_x_ant;
    double y_comp = y_proy + linear_y_ant;
    if(linear_x_ == 0 && linear_y_ == 0){
   	 transform.setOrigin(tf::Vector3(x_comp_ant,y_comp_ant,linear_z_ant));
    }
    else{
    transform.setOrigin(tf::Vector3(x_comp,y_comp,linear_z_ant));
    x_comp_ant = x_comp;
    y_comp_ant = y_comp;
    linear_x_ant += x_proy;
    linear_y_ant += y_proy; 
    }
    tf::Quaternion q;
    q.setRPY(0,0,angular_);
    transform.setRotation(q);

    if(dirty ==true)
    {
      /*twist_pub_.publish(twist);    
      dirty=false;*/
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "navigation_frame"));
    }
  }


  return;
}
