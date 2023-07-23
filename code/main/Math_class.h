//************************************
//         Math_class.h    
//************************************
#ifndef MATH_CLASS_H
#define MATH_CLASS_H

#include "Arduino.h"

class PID_class {
  public:
    void initial(float P_constant, float I_constant, float D_constant, double delta_t, int max_offset);
    void PID_output(int current_value);
    void reset_values();

    double _offset_value;
    int _desired_value;
    
  private:
    double _P_constant;
    int _I_constant;
    int _D_constant;

    double _delta_t;
    double _old_error;
    double _error_intergral;
    int _max_offset;
};


class Location_class {
  public:
    void initial(void);
    void calc_location(void);

    double _angle;

  private:
    double _x_dist;
    double _y_dist;
    double _av_dist;
    double _prev_dist;
};

#endif //MATH_CLASS_H
