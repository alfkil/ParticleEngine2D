#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
#include "../Service/vector2d.hpp"

int deltamousex;
int deltamousey;
int lastx, lasty;
bool firsttimer;

int rotateleft, rotateright;

class Input {
    private:
        
    public:
        static void KeyboardPress(unsigned char key, int x, int y) {
            switch(key) {
              case '\x1B':
      exit(EXIT_SUCCESS);
      break;
            }
        }
        static void SpecialKeyPress(int key, int x, int y)
        {
            switch(key)
            {
                case GLUT_KEY_LEFT:
                    //BatNewAngle(mybat.angle-M_PI/20.0);
                    rotateleft = 1;
                    break;
                case GLUT_KEY_RIGHT:
                    //BatNewAngle(mybat.angle+M_PI/20.0);
                    rotateright = 1;
                    break;
            }
        }
        static void MouseMove(int x, int y)
        {
            if (!firsttimer) {
                deltamousex = x - lastx;
                deltamousey = y - lasty;
            }
            firsttimer = 0;
            lastx = x; lasty = y;
        }

        Input()
        {
            deltamousex = 0; deltamousey = 0;
            firsttimer = 1;
            glutPassiveMotionFunc(MouseMove);
            glutSpecialFunc(SpecialKeyPress);
            glutKeyboardFunc(KeyboardPress);
        }

        Vector2D deltaMouse() {
            return Vector2D (deltamousex, deltamousey);
        }

        int rotate() {
            int result = 0;
            if(rotateright) result = 1;
            if (rotateleft) result = -1 ;
            rotateleft = 0; rotateright = 0;
            return result;
        }
};
