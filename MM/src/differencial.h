#define DEF_SPEED 50
#define MAX_PWM 255

struct diff_motors {
  int a;
  int b;
};



struct diff_motors get_diff(int pidout) {

  struct diff_motors retval ;

  retval.a = DEF_SPEED + pidout;

  retval.b = DEF_SPEED + pidout;


  if (retval.a > MAX_PWM) {
    retval.a = MAX_PWM;
  } else if (retval.a < -MAX_PWM){
    retval.a = -MAX_PWM;
  }

  if (retval.b > MAX_PWM) {
    retval.b = MAX_PWM;
  } else if (retval.b < -MAX_PWM){
    retval.b = -MAX_PWM;
  }


  return retval;
}
