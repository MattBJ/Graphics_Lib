#include "bodies.hpp"

constexpr uint16_t PPM = 4; // 16 pixels per meter
constexpr float dt = (1.0/60); // 60 fps
constexpr float MPS = 1.0; // 1 meter / second == x / dt.. just use dt

constexpr double pi = 3.141592653589793;

void Pixel::newPixel(uint16_t _x, uint16_t _y){
	x.push_back(_x);
	y.push_back(_y);
	return;
}

void Pixel::clear(){
	x.clear();
	y.clear();
}

Pixel Pixel::copy(){
	Pixel out;
	out.x = x;
	out.y = y;
	return out;
}

Ball::Ball(float _radius, float _xCenter, float _yCenter) : 
pos(_xCenter,_yCenter), vel((MPS*10)*dt,(MPS*5)*dt), acc((MPS/2)*dt,(MPS/2)*dt) // just for now
{
	radius = _radius;
	theta = 0;
	thetaDot = 0;
	theta2Dot = 0;

	// update() initializes:
		// min's and max's, as well as the corners
		// painter class
	update();
	return;
}

void Ball::update(){
	painter.clear();
	min_x = static_cast<int32_t>((pos(coordinate::X) - radius)*PPM + 0.5);
	min_y = static_cast<int32_t>((pos(coordinate::Y) - radius)*PPM + 0.5);
	max_x = static_cast<int32_t>((pos(coordinate::X) + radius)*PPM + 0.5);
	max_y = static_cast<int32_t>((pos(coordinate::Y) + radius)*PPM + 0.5);
	uint16_t i, j;
	// iterate over square (bottom left: position x/y - radius.. converted to PPM)
	// adding 0.5 for rounding, it gets floored
	uint16_t quantizedR = static_cast<uint16_t>(radius * PPM + 0.5);
	std::cout << "Center pixel: (" 	<< static_cast<uint16_t>(pos(coordinate::X)*PPM+0.5) << ","
									<< static_cast<uint16_t>(pos(coordinate::Y)*PPM + 0.5) << ")\n";
	
	std::cout <<
	"\tmin_x:\t" << min_x << std::endl << 
	"\tmax_x:\t" << max_x << std::endl << 
	"\tmin_y:\t" << min_y << std::endl << 
	"\tmax_y:\t" << max_y << std::endl <<
	"\tvelX:\t" << vel(coordinate::X) << std::endl <<
	"\tvelY:\t" << vel(coordinate::Y) << std::endl;

	// @TODO: add clipping range on max side as well --> WINDOW_WIDTH/HEIGHT
	for(i= (min_x>=0)? min_x : 0;i<max_x;i++){ // x axis pixels
		for(j=(min_y>=0)? min_y : 0;j<max_y;j++){ // y axis pixels
			// now figure out distance from center
			Vector2f checkPoint(static_cast<float>(i)/PPM,static_cast<float>(j)/PPM);
			uint16_t distPix = static_cast<uint16_t>(
						((checkPoint-pos).norm()*PPM)+0.5 // difference vector is in meters, convert, add 0.5 for floor rounding
						);
			if( distPix <= quantizedR){
				painter.newPixel(i,j);
			}
		}
	}
	return;
}

// void Ball::move(Vector2f jerk){ // change in acceleration
// 	acc = acc + jerk;
// 	move();
// }

 // constant acceleration

void Ball::move(bool angular){ // default = false
	// for now let's always use angular
	if(angular){
		Matrix3f velAf;
		// cos,sin
		// -sin,cos --> counterclockwise orientation
		velAf <<	cos(thetaDot),		sin(thetaDot),	vel(coordinate::X),
					-sin(thetaDot),		cos(thetaDot),	vel(coordinate::Y),
					0,					0,				1;
		Matrix3f accAf;
		accAf <<	cos(theta2Dot),		sin(theta2Dot),	acc(coordinate::X),
					-sin(theta2Dot),	cos(theta2Dot),	acc(coordinate::Y),
					0,					0,				1;
		Vector3f tmpPos;
		tmpPos << pos, 1; // concatenates a 1 on the end of position vector and gets assigned to tmpPos
		Vector3f tmpVel;
		tmpVel << vel, 1;
		// Vector3f tmpAcc(acc(coordinate::X),acc(coordinate::Y),1);

		tmpPos = velAf * tmpPos;
		tmpVel = accAf * tmpVel;
		pos(coordinate::X) = tmpPos(coordinate::X);
		pos(coordinate::Y) = tmpPos(coordinate::Y);

		vel(coordinate::X) = tmpVel(coordinate::X);
		vel(coordinate::Y) = tmpVel(coordinate::Y);

		// acc(coordinate::X) = tmpAcc(coordinate::X);
		// acc(coordinate::Y) = tmpAcc(coordinate::Y);
		theta += thetaDot;
		thetaDot += theta2Dot;
		return;
	} else{
		vel += acc;
		pos += vel;
		return;
	}
}

void Ball::bounce(uint16_t _width, uint16_t _height){ // max width/height of window
	uint8_t bitField = 0;
	// (max_x > _width) || (min_x < 0)
	Matrix3f Affine;
	if((bitField = 		(max_x > _width)? 1 : 
						(min_x < 0)? 2 : 0)){
		// 1 means reflect from right side
		// 2 means reflect from y axis

		// case 1: Changing reference frame from origin to right border axis
			// shift reference frame for vector
			// do Affine reflection PLUS the shift back --> back to original ref frame

		Vector3f afVec;
		afVec << pos, 1; //pos(0) aka pos(coordinate::X) is going to be replaced

		// reference frame: the rvalue of a subtraction
			// case 1: diff vec points from max border to the position vector
		afVec(coordinate::X) = (bitField == 1)? pos(coordinate::X) - (static_cast<float>(_width)/PPM - radius): 
												pos(coordinate::X) - (radius - 0);

		Affine << 	-1,0,0, // need to edit this dX component
					0,1,0,
					0,0,1;
		Affine(0,2) = (bitField == 1)? (static_cast<float>(_width)/PPM - radius) : (radius - 0);

		afVec = Affine * afVec;
		pos(coordinate::X) = afVec(coordinate::X);
		pos(coordinate::Y) = afVec(coordinate::Y);
		// reflection operation complete
		// update velocity/acceleration after performing the proper reflection
		vel(coordinate::X) *= -1;
		acc(coordinate::X) *= -1;
		// not doing any angular stuff here..
		bitField = 0; // i think this is redundant
		update(); // redraw the pixels
	}
	// (max_y > _height) || (min_y < 0)
	if((bitField = 		(max_y > _height)? 1 : 
						(min_y < 0)? 2 : 0)){
		// 1 means reflect from top axis
		// 2 means reflect from x axis

		Vector3f afVec;
		afVec << pos,1; //pos(1) aka pos(coordinate::Y) is going to be replaced

		afVec(coordinate::Y) = (bitField == 1)? pos(coordinate::Y) - (static_cast<float>(_height)/PPM - radius) :
												pos(coordinate::Y) - (radius - 0);

		Affine <<	1,0,0,
					0,-1,0, // need to edit this dY component
					0,0,1;

		Affine(1,2) = (bitField == 1)? (static_cast<float>(_height)/PPM - radius) : (radius - 0);
		afVec = Affine * afVec;
		pos(coordinate::X) = afVec(coordinate::X);
		pos(coordinate::Y) = afVec(coordinate::Y);

		vel(coordinate::Y) *= -1;
		acc(coordinate::Y) *= -1;
		bitField = 0;
		update();
	}
	return;
}

Rectangle::Rectangle(float _length, float _width, float _posX, float _posY) :
	vel((MPS*10)*dt,(MPS*5)*dt), acc((MPS/10)*dt,(MPS/10)*dt) // arbitrary stuff
	// rPos is going to be recalculated
	// rPos(0,0), rVel(0,0), rAcc(0,0)
{
	// want to track the referenceFrame
		// 'standard graphics uses the corner'...
	// position will be the body's reference frame from origin

	theta = 0;
	length = _length;
	width = _width;
	// first generate all corners by referencing the generator corner
	corners[0] << _posX, _posY; // generator
	corners[1] << sin(theta)*length,cos(theta)*length;
	corners[1] += corners[0];
	corners[2] << cos(theta)*width,sin(theta)*width;
	corners[2] += corners[0];
	corners[3] = (corners[1]-corners[0]) + (corners[2]-corners[0]) + corners[0];

	// second create the actual reference frame --> here on out all corners will use this
	// reference frame lies on the base of the rectangle: between corners 0 and 2, halfway

	pos = corners[2] - corners[0]; // referencing the generator
	pos = ((width/2)*pos)/pos.norm();
	pos += corners[0]; // now this reference frame references the ground frame


	thetaDot = 0;
	theta2Dot = 0;

	update();
	return;
}

// creates painter pixels
// borders (X and Y ranges)
void Rectangle::update(){
	painter.clear();

	// figure out the borders
	// pos represents one corner

	std::cout << "\ncorners[0]:\n" << corners[0].transpose() << std::endl;
	std::cout << "\ncorners[1]:\n" << corners[1].transpose() << std::endl;
	std::cout << "\ncorners[2]:\n" << corners[2].transpose() << std::endl;
	std::cout << "\ncorners[3]:\n" << corners[3].transpose() << std::endl;

	max_x = max_y = std::numeric_limits<int32_t>::min();
	min_x = min_y = std::numeric_limits<int32_t>::max();

	for(uint8_t i = 0; i < 4; i++){
		min_x =	(static_cast<int32_t>(corners[i](coordinate::X)*PPM + 0.5) < min_x)? static_cast<int32_t>(corners[i](coordinate::X)*PPM + 0.5) : min_x;
		max_x = (static_cast<int32_t>(corners[i](coordinate::X)*PPM + 0.5) > max_x)? static_cast<int32_t>(corners[i](coordinate::X)*PPM + 0.5) : max_x;
		min_y = (static_cast<int32_t>(corners[i](coordinate::Y)*PPM + 0.5) < min_y)? static_cast<int32_t>(corners[i](coordinate::Y)*PPM + 0.5) : min_y;
		max_y = (static_cast<int32_t>(corners[i](coordinate::Y)*PPM + 0.5) > max_y)? static_cast<int32_t>(corners[i](coordinate::Y)*PPM + 0.5) : max_y;
	}
	std::cout << "min_x:\t" << min_x << std::endl;
	std::cout << "max_x:\t" << max_x << std::endl;
	std::cout << "min_y:\t" << min_y << std::endl;
	std::cout << "max_y:\t" << max_y << std::endl;

	// special case or not --> need to find X and Y coordinate copy on another corner
	bool specialCase = false;
	uint8_t inc = 0;
	for(uint8_t i = 1; i < 4; i++){
		// check corner[0] quantized with the rest
		inc += ((corners[0](coordinate::X) == corners[i](coordinate::X)) || (corners[0](coordinate::Y) == corners[i](coordinate::Y)))? 1 : 0;
		specialCase = (inc == 2)? true : false; 
		if(specialCase){
			break; // stop checking
		}
	}
	// special case --> paint every loop!
	Vector2f painterCorners[4];
	if(!specialCase){ // need to put our rectangle corners in standard order for painting
		int32_t borderVals[4] = {min_x, max_x, min_y, max_y};
		for(uint8_t i = 0; i < 4; i++){ // i = iterator for output paintCorner
			// iterate over the painterCornerss --> rectangle corners in standard order
			for(uint8_t j = 0; j < 4; j++){ // iterate over the borderValues in standard order
				// iterator over the 4 border values --> check if the corresponding corner's coordinate matches
				// j < border::MIN_Y == checking x's
				// j > border::MAX_X == checking y's
				for(uint8_t k=0; k < 4; k++){ // iterate over all rectangle members: corner[0-3]
					painterCorners[i] =	((j < border::MIN_Y) && ( static_cast<int32_t>((corners[k](coordinate::X)*PPM)+0.5) == borderVals[j]))? corners[k] :
										((j > border::MAX_X) && ( static_cast<int32_t>((corners[k](coordinate::Y)*PPM)+0.5) == borderVals[j]))? corners[k] : painterCorners[i];
				}
			}
		}
	}

	// @TODO: Range clipping on the upper bounds
	for(uint16_t i = (min_x >=0)? min_x: 0; i<max_x; i++){
		for(uint16_t j = (min_y >=0)? min_y: 0; j<max_y; j++){
			if(specialCase){
				painter.newPixel(i,j);
			} else { // need to do piecewise line checking..
				Vector2f topCheck[2];
				Vector2f botCheck[2];
				topCheck[0] = (i <= static_cast<int32_t>( painterCorners[border::MAX_Y](coordinate::X)*PPM + 0.5 ))? painterCorners[border::MIN_X] : painterCorners[border::MAX_Y];
				topCheck[1] = (i <= static_cast<int32_t>( painterCorners[border::MAX_Y](coordinate::X)*PPM + 0.5 ))? painterCorners[border::MAX_Y] : painterCorners[border::MAX_X];

				botCheck[0] = (i <= static_cast<int32_t>( painterCorners[border::MIN_Y](coordinate::X)*PPM + 0.5 ))? painterCorners[border::MIN_X] : painterCorners[border::MIN_Y];
				botCheck[1] = (i <= static_cast<int32_t>( painterCorners[border::MIN_Y](coordinate::X)*PPM + 0.5 ))? painterCorners[border::MIN_Y] : painterCorners[border::MAX_X];
				// top line checking
				Vector2f pointCheck(i,j);
				
				// bottom line checking
				
				if( ( lineCheck(topCheck[0],topCheck[1],pointCheck) ) && 
					( lineCheck(botCheck[0],botCheck[1],pointCheck) )){
					painter.newPixel(i,j);
				}
			}
		}
	}
	return;
}

bool Rectangle::lineCheck(Vector2f p1, Vector2f p2, Vector2f pIn){
	// reference point = p1
	// decider = p2 - p1
	// check = p3 - p1
	// normalize them both
	// project them in y direction --> setting p1 as reference makes the Y component good enough
		// ---------------------???
	Vector2f decider = p2 - p1;
	Vector2f check = pIn - p1;
	decider = (decider*50)/decider.norm(); // normalize and set magnitude to 50
	check 	= (check*50)/check.norm();

	bool retVal;
	retVal = ((check(coordinate::Y)) <= (decider(coordinate::Y)))? true : false;

	return retVal;
}

// updates velocities (accelerations too? not now)
// updates the 4 corners to generate pixels from

void Rectangle::move(bool angular){
	// need to shift
	if(angular){
		Matrix3f velAf;
		// Matrix3f accAf;
		// is the acceleration Affine even necessary?
		// I update velocity vector from acceleration, then update angular as scalar..
		velAf <<	cos(thetaDot),	sin(thetaDot),	vel(coordinate::X),
					-sin(thetaDot),	cos(thetaDot),	vel(coordinate::Y),
					0,0,1;

		// accAf <<	cos(theta2Dot),		sin(theta2Dot),acc(coordinate::X),
		// 			-sin(theta2Dot),	cos(theta2Dot),acc(coordinate::Y),
		// 			0,0,1;

		Vector3f tmpPos, tmpVel;
		for(uint8_t i = 0; i < 4; i++){
			// all 4 corners rotate RELATIVE to reference frame

			// scratch buffers for switching between 2D and 3D vectors
			Vector2f scratch2; // for doing 2D math
			Vector3f scratch3; // for doing 3D math
			scratch2 = corners[i] - pos; // referenced from the body reference frame
			Vector3f cornerTransform;
			cornerTransform << scratch2,1;
			cornerTransform = velAf * cornerTransform; // angular + linear velocity
			scratch3 << pos,0;
			cornerTransform += scratch3; // shift back to ground frame!
			corners[i](coordinate::X) = cornerTransform(coordinate::X);
			corners[i](coordinate::Y) = cornerTransform(coordinate::Y);
		}

		// these vectors all for reference frame
		pos += vel;
		vel += acc;
		// not updating acceleration for now
		theta += thetaDot;
		thetaDot += theta2Dot;
	}
	return;
}

void Rectangle::bounce(uint16_t _width, uint16_t _height){
	return;
}
