#ifndef CUB_COMMANDER__JOYPAD_BASE_HPP_
#define CUB_COMMANDER__JOYPAD_BASE_HPP_

class JoypadBase{
public:

  virtual ~JoypadBase(){};
  virtual bool up(){return false;};
  virtual bool down(){return false;};
  virtual bool left(){return false;};
  virtual bool right(){return false;};

  virtual bool buttonUp(){return false;};     // general
  // virtual bool triangle();     // PlayStation
  // virtual bool y();            // XBox

  virtual bool buttonDown(){return false;};   // general
  // virtual bool cross();        // PlayStation
  // virtual bool a();            // XBox

  virtual bool buttonLeft(){return false;};   // general
  // virtual bool square();       // PlayStation
  // virtual bool x();            // XBox

  virtual bool buttonRight(){return false;};  // general
  // virtual bool circle();       // PlayStation
  // virtual bool b();            // XBox

  virtual bool optionLeft(){return false;};   // general
  // virtual bool create();       // PlayStation
  // virtual bool view();         // XBox

  virtual bool optionRight(){return false;};  // general
  // virtual bool option();       // PlayStation
  // virtual bool menu();         // XBox

  virtual bool optionCenter(){return false;}; // general
  // virtual bool touchpad();     // PlayStation
  // virtual bool share();        // XBox

  virtual bool logo(){return false;};         // general
  // virtual bool ps();           // PlayStation
  // virtual bool xbox();         // XBox

  virtual bool l1(){return false;};           // general, PlayStation
  // virtual bool l();            // XBox

  virtual bool r1(){return false;};           // general, PlayStation
  // virtual bool r();            // XBox

  virtual bool l2(){return false;};           // general, PlayStation
  // virtual bool lt();           // XBox

  virtual bool r2(){return false;};           // general, PlayStation
  // virtual bool rt();           // XBox

  virtual bool l3(){return false;};           // general, PlayStation, XBox

  virtual bool r3(){return false;};           // general, PlayStation, XBox

  virtual double lx(){return 0;};
  virtual double ly(){return 0;};
  virtual double rx(){return 0;};
  virtual double ry(){return 0;};

  virtual double l2f(){return 0.0;};        // general, PlayStation
  // double ltf();        // XBox

  virtual double r2f(){return 0.0;};        // general, PlayStation
  // double ltf();        // XBox

  virtual void print(){
    printf("joy: ");
    printf("lx:% 1.2lf, ly:% 1.2lf, rx:% 1.2lf, ry:% 1.2lf, l2f:% 1.2lf, r2f:% 1.2lf",lx(),ly(),rx(),ry(),l2f(),r2f());
    if(up()) printf(", up");
    if(down()) printf(", down");
    if(left()) printf(", left");
    if(right()) printf(", right");
    if(buttonUp()) printf(", buttonUp");
    if(buttonDown()) printf(", buttonDown");
    if(buttonLeft()) printf(", buttonLeft");
    if(buttonRight()) printf(", buttonRight");
    if(optionLeft()) printf(", optionLeft");
    if(optionRight()) printf(", optionRight");
    if(optionCenter()) printf(", optionCenter");
    if(logo()) printf(", logo");
    if(l1()) printf(", l1");
    if(r1()) printf(", r1");
    if(l2()) printf(", l2");
    if(r2()) printf(", r2");
    if(l3()) printf(", l3");
    if(r3()) printf(", r3");
    printf("\n");
    fflush(stdout);
  };



};

#endif  // CUB_COMMANDER__JOYPAD_BASE_HPP_