
#ifndef RENDERER_GLUT_hpp
#define RENDERER_GLUT_hpp

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif
#include <math.h>

#include <vector>
#include <cstdio>

#include "../Service/colorrgb.hpp"
#include "../Service/vector2d.hpp"
#include "../Border/renderer.hpp"
#include "../Entity/particles.hpp"

using namespace std;

class GLUTParticleRenderer : public ParticleRenderer {
    private:
        int argc;
        char **argv;
        static ParticleSystem *world;
        
    public:
        GLUTParticleRenderer (int _argc, char **_argv) :
            argc(_argc),
            argv(_argv)
        {
            initialize();
        }

    private:
        void initialize() {
            glutInit(&argc, argv);
            glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
            glutInitWindowSize(640, 480);
            glutCreateWindow("Swingbal2D Preview");

            glutDisplayFunc(display);

            //init for 2d painting
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-3.2, 3.2, 2.4, -2.4, 0, 1); //playfield is 6.40x4.80 meters
            glMatrixMode(GL_MODELVIEW);
            glDisable(GL_DEPTH_TEST);
        }

        static void display () {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            drawWorld ();
            glutSwapBuffers();
        }

        static void drawWorld () {
            //make sure we have a world
            if (!world) {
                return;
            }

            glMatrixMode(GL_MODELVIEW);
            glClear(GL_COLOR_BUFFER_BIT);

            //sticks
            vector<Stick *> &sticks = world->getSticks();
            for (int i = 0; i < sticks.size(); i++) {
                Stick *stick = sticks[i];
                drawStick (stick->position().x(), stick->position().y(), stick->radius(), ColorRGB(ColorRGB::COLOR_BLUE));
            }

            //springs (rope)
            vector<Spring *> &springs = world->getSprings();
            for (int i = 0; i < springs.size(); i++) {
                Spring *spring = springs[i];
                drawSpring (spring->position1().x(), spring->position1().y(),
                            spring->position2().x(), spring->position2().y(),
                            ColorRGB(ColorRGB::COLOR_GREEN));
            }

            //balls
            vector<Ball *> &balls = world->getBalls();
            for (int i = 0; i < balls.size(); i++) {
                Ball *ball = balls[i];
                drawBall (ball->position().x(), ball->position().y(), ball->radius(), ball->angle(), ColorRGB(ColorRGB::COLOR_RED));
            }

            //sticks
            vector<Bat *> &bats = world->getBats();
            for (int i = 0; i < bats.size(); i++) {
                Bat *bat = bats[i];
                drawBat (bat->position().x(), bat->position().y(), bat->angle(), bat->width(), ColorRGB(ColorRGB::COLOR_WHITE));
            //debug
            Vector2D end1 = bat->endpoint1();
            Vector2D end2 = bat->endpoint2();
            Vector2D dist1 = end1 - bat->position();
            Vector2D dist2 = end2 - bat->position();
            Vector2D eigen1 = dist1.eigen();
            Vector2D eigen2 = dist2.eigen();
            eigen1.scale(onto1);
            eigen2.scale(onto2);
            if(onto1 > dist1.length()) {
                glBegin(GL_LINES);
                    glColor3f(1.0, 0.0, 0.0);
                    glVertex2f(bat->position().x(), bat->position().y());
                    glVertex2f(bat->position().x() + eigen1.x(), bat->position().y() + eigen1.y());
                    // glVertex2f(end1.x(), end1.y());
                glEnd();
            }
            if(onto2 > dist2.length()) {
                glBegin(GL_LINES);
                    glColor3f(0.0, 1.0, 0.0);
                    glVertex2f(bat->position().x(), bat->position().y());
                    glVertex2f(bat->position().x() + eigen2.x(), bat->position().y() + eigen2.y());
                    // glVertex2f(end2.x(), end2.y());
                glEnd();
            }
            // switch(debugFlag) {
            //     case 0:
            //     glColor3f(0.2, 0.2, 0.2);
            //     break;
            //     case 1:
            //     glColor3f(1.0, 0.0, 0.0);
            //     break;
            //     case 2:
            //     glColor3f(0.0, 1.0, 0.0);
            //     break;
            //     case 3:
            //     glColor3f(0.0, 0.0, 1.0);
            //     break;
            //     case 4:
            //     glColor3f (1.0, 1.0, 1.0);
            //     break;
            // }
            // glBegin(GL_LINES);
            // glVertex2f(end1.x(), end1.y());
            // glVertex2f(end2.x(), end2.y());
            // glEnd();
            }
        }

    private:
        static void drawStick (double x, double y, double radius, ColorRGB color) {
            //draw stick in the middle
            glColor3f(color.red(), color.green(), color.blue());

            glBegin(GL_TRIANGLE_FAN);

            for (double angle = 0.0; angle <= 2*M_PI; angle += M_PI/10.0) {
                glVertex2f(x + sin(angle) * radius, y + cos(angle) * radius);
            }

            glEnd();
        }

        static void drawRope (vector<Vector2D> &ropePoints, ColorRGB color) {
            //draw rope
            glColor3f(color.red(), color.green(), color.blue());

            glLineWidth(1.0f);
            glBegin(GL_LINE_STRIP);

            for(int i = 0; i < ropePoints.size(); i++) {
                Vector2D point = ropePoints[i];
                glVertex2f(point.x(), point.y());
            }

            glEnd();

            //draw points on rope
            glColor3f(1.0, 0.0, 0.0);
            glLineWidth(1.0f);

            glBegin(GL_POINTS);
            for (int i = 1; i < ropePoints.size() - 1; i++) {
                Vector2D point = ropePoints[i];
                glVertex2f(point.x(), point.y());
            }
            glEnd();
        }

        static void drawSpring (double x1, double y1, double x2, double y2, ColorRGB color) {
            //draw spring
            glColor3f(color.red(), color.green(), color.blue());

            glLineWidth(1.0f);
            glBegin(GL_LINES);

            glVertex2f(x1, y1);
            glVertex2f(x2, y2);

            glEnd();

            //draw point
            glColor3f(1.0, 0.0, 0.0);
            glLineWidth(1.0f);

            glBegin(GL_POINTS);
            glVertex2f(x1, y1);
            glVertex2f(x2, y2);
            glEnd();
        }

        static void drawBall (double x, double y, double radius, double rotation, ColorRGB color) {
            //draw ball
            glColor3f(color.red(), color.green(), color.blue());

            glBegin(GL_TRIANGLE_FAN);
            for (double angle = 0.0; angle <= 2*M_PI; angle += M_PI/10.0) {
                glVertex2f(x + sin(angle) * radius, y + cos(angle) * radius);
            }
            glEnd();

            //draw stripe
            glColor3f(1.0, 1.0, 1.0);
            glLineWidth(2.0);

            glBegin(GL_LINES);
            glVertex2f(x + cos(rotation) * radius, y - sin(rotation) * radius);
            glVertex2f(x - cos(rotation) * radius, y + sin(rotation) * radius);
            glEnd();
        }

        static void drawBat (double x, double y, double angle, double width, ColorRGB color) {
            //draw bat
            Vector2D plane(-sin(angle), cos(angle)); //normal rotated 90 degrees counter clockwise
            Vector2D pos(x, y);
            Vector2D p1 = pos + plane * width / 2;
            Vector2D p2 = pos - plane * width / 2;

            glColor3f(color.red(), color.green(), color.blue());

            glLineWidth(2.0f);
            glBegin(GL_LINES);
            glVertex2f(p1.x(), p1.y());
            glVertex2f(p2.x(), p2.y());
            glEnd();
        }

    public:
        void ignite () {
            glutPostRedisplay();
            glutMainLoopEvent();
        }

        void setWorld (ParticleSystem *_world) {
            world = _world;
        }
};

ParticleSystem *GLUTParticleRenderer::world = 0;
#endif
