#define TOUCH_SDA  33
#define TOUCH_SCL  32
#define TOUCH_INT  21
#define TOUCH_RST 25
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 320

#define SWIPE_LEFT  10
#define SWIPE_RIGHT 11
#define SWIPE_DOWN  12
#define SWIPE_UP    13
#define LONG_PRESS   2
#define CLICK        1

#define LONG_PRESS_INTERVAL 300

struct Touch_struct {
  int x0, x1, y0, y1;
  uint32_t TimestampTouched;
  uint32_t TimestampReleased;
  int Gesture;
};