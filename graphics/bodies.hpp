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
	void move(bool angular = false);
	// add this next
	// void move(Vector2f jerk);
	void bounce(uint16_t _width, uint16_t _height);
	void update();
	Ball copy();
	void capVector(uint16_t _width, uint16_t _height);
};

/*
	coordinate systems, frames of reference

	want to CHANGE THE BASIS (unit) VECTORS from one frame of reference to another

	vector = n1 * v1 + n2 * v2 + n3 * v3 --> v1-3 are the unit vectors

	u1 = a11 * v1 + a12 * v2 + a13 * v3
	u2 = a21 * v1 + a22 * v2 + a23 * v3
	u3 = a31 * v1 + a32 * v2 + a33 * v3

	3x3 'change of basis' matrix: M = [a11, a12, a13], [a21, a22, a23], [a31, a32, a33]

	2 representations of a given vector v:
		v = aT(v1,v2,v3) and v = bT(u1,u2,u3)
		
		a = (n1,n2,n3)T and b = (m1,m2,m3) .. the scalar quantities that correspond to each unit vector

	implication:
		a = MTrans * b
		b = inverse(MTrans) * a

	Matrix M could be used to ROTATE and SCALE vectors --> CAN'T DEAL WITH POINTS in graphics
			we want to TRANSLATE points (and objects)

	arbitrary transformation can be achieved by:
		mul by 3x3 matrix
		shift by a vector
	IN GRAPHICS WE PREFER:
		use FRAMES to achieve same thing

	Frame is a richer coordinate system
		reference point P0
		3 linearly independent basis vectors (v1,v2,v3)
		represent vectors v and points P differently:
			v = n1 * v1 + n2 * v2 + n3 * v3
			P = P0 + v (!!!!)

	v = [n1, n2, n3, 0]*[v1,v2,v3,P0]T
	P = [n1, n2, n3, 1]*[v1,v2,v3,P0]T

	coefficients ^^^ --> homogeneous coordinates of v and P respectively

	change of frames

	from frame (v1,v2,v3,P0) to new frame (u1,u2,u3,Q0)
		u1 = a11*v11 + a12*v2 + a13*v3
		u2 = ..
		u3 = ..
		Q0 = a41*v1 + a42*v2 + a43*v3 + P0

	4x4 matrix:
		M = [[a11,a12,a13,0],[a21,a22,a23,0],[a31,a32,a33,0],[a41,a42,a43,1]]

	a and b are homogeneous representations of same point/vector with respect to 2 frames:
		a = [v1,v2,v3,P0],b=[u1,u2,u3,Q0] = btrans * M [v1,v2,v3,P0]

	implies:
		a = Mtrans * b
		b = inverse(MTrans) * a

	Affine transformation
		transposed matrix:
			MTrans = 	|a11,	a21,	a31,	a41 |
							|a12,	a22,	a32,	a42 |
							|a13,	a23,	a33,	a43 |
							|0,		0,		0,		1	|
		-- an arbitrary AFFINE TRANSFORMATION
			12 degrees of freedom
				9 DOF as the 3x3 matrix
				+ components of a VECTOR SHIFT

	most important affine transformations:
		rotation
		scaling
		translation
	all affine transformations can be expressed as a combination of all 3 of those
*/


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
	void move(bool angular = false); // default value
	void bounce(uint16_t _width, uint16_t _height);
};

#endif	//__BODIES_H__