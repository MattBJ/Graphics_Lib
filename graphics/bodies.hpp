#ifndef 	__BODIES_H__
#define		__BODIES_H__
// override the '+' operator
// what does + mean for a vector?
	// need to add eachs element relative to its accessor
#include <vector>
#include <stdint.h>
#include <cmath> // math.h ?
#include <iostream>

#include <limits> // std::numeric_limits for data types
#include <optional> // c++17
#include <tuple> // c++11
#include <set>

#include <Eigen/Dense> // eigen file path

using namespace Eigen;


// const --> doesnt change but can be evaluated at run time
// constexpr --> doesnt change but IS EVALUATED AT COMPILE TIME

extern const uint16_t PPM; // -------------- VERY IMPORTANT
extern const float MPS; // meters per second velocity
extern const double pi; // important constant
extern const float dt;


// sadly, can't use enum class because it creates its own type.. and eigen expects integer arguments
// 0,1,2
enum coordinate {X,Y,Z}; // enum class rip
// 0,1,2,3
enum border {MIN_X,MAX_X,MIN_Y,MAX_Y}; // used for painter object in rectangle

// quantization of real world vectors
class Pixel { // quantization
public: // no need for initializer (excluded from initializer lists)
	std::vector<uint16_t> x;
	std::vector<uint16_t> y;
	void newPixel(uint16_t _x, uint16_t _y);
	void clear();
	Pixel copy();
};

// stores both real world vectors and quantized position only vector
class Ball {
public:
	Vector2f pos;
	Vector2f vel;
	Vector2f acc;

	float theta,thetaDot,theta2Dot;

	Pixel painter;
	float radius;
	int32_t min_x, min_y, max_x, max_y;
	// all in meters, 16 pixels per meter
	Ball(float _radius, float _xCenter, float _yCenter);
	void move();
	// add this next
	// void move(Vector2f jerk);
	void bounce(uint16_t _width, uint16_t _height);
	void update();
	Ball copy();
	void capVector(uint16_t _width, uint16_t _height);
};

// https://www.uio.no/studier/emner/matnat/ifi/nedlagte-emner/INF3320/h03/undervisningsmateriale/lecture3.pdf

// going to use Eigen library for my linear algebra from here on!


class Rectangle {
public:
	float length; // magnitude vectors
	float width; // magnitude vectors

	Vector2f pos; // referenceFrame
	Vector2f vel;
	Vector2f acc;

	Vector2f corners[4];

	// attitude: angle, angular velocity, angular acceleration
	float theta, thetaDot, theta2Dot;

	// shape vectors
	// PVector lVec;
	// PVector wVec;

	Pixel painter;
	int32_t min_x,max_x,min_y,max_y;
	// for now, rotational frame is not configurable by developer
	Rectangle(float _length, float _width, float _posX, float _posY);
	// generates pixels (and X/Y ranges)
	void update();
	bool lineCheck(Vector2f p1, Vector2f p2, Vector2f pIn, bool under);
	// move updates reference frame position, velocity, and acceleration
	// generates all 4 corners
	void move(); // default value
	void bounce(uint16_t _width, uint16_t _height);
};

int32_t roundHelper(float in);
std::optional<std::set<Vector2f*>> checkCopies(Vector2f ** in);

#endif	//__BODIES_H__