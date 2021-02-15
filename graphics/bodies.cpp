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
pos(_xCenter,_yCenter), vel((5)*dt,(5)*dt), acc(0,0) // just for now
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
	// std::cout << "Center pixel: (" 	<< static_cast<uint16_t>(pos(coordinate::X)*PPM+0.5) << ","
									// << static_cast<uint16_t>(pos(coordinate::Y)*PPM + 0.5) << ")\n";
	
	// std::cout <<
	// "\tmin_x:\t" << min_x << std::endl << 
	// "\tmax_x:\t" << max_x << std::endl << 
	// "\tmin_y:\t" << min_y << std::endl << 
	// "\tmax_y:\t" << max_y << std::endl <<
	// "\tvelX:\t" << vel(coordinate::X) << std::endl <<
	// "\tvelY:\t" << vel(coordinate::Y) << std::endl;

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

void Ball::move(){
	Matrix3f velAf;
	// cos,-sin
	// sin,cos --> counterclockwise orientation
	velAf <<	cos(thetaDot),		-sin(thetaDot),	vel(coordinate::X),
				sin(thetaDot),		cos(thetaDot),	vel(coordinate::Y),
				0,					0,				1;
	Matrix3f accAf;
	accAf <<	cos(theta2Dot),		-sin(theta2Dot),acc(coordinate::X),
				sin(theta2Dot),		cos(theta2Dot),	acc(coordinate::Y),
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
	// vel((MPS*10)*dt,(MPS*5)*dt), acc((MPS/10)*dt,(MPS/10)*dt) // arbitrary stuff
	vel((5.0)*dt,(5.0)*dt), acc(0,0)
	// rPos is going to be recalculated
	// rPos(0,0), rVel(0,0), rAcc(0,0)
{
	// want to track the referenceFrame
		// 'standard graphics uses the corner'...
	// position will be the body's reference frame from origin

	theta = 3*pi/2;
	length = _length;
	width = _width;
	// first generate all corners by referencing the generator corner and position args
	Vector2f tmpLen, tmpWid;
	tmpLen << 0,length;
	tmpWid << width,0;
	corners[0] << _posX, _posY; // generator
	corners[1] = tmpLen + corners[0];
	corners[2] = tmpWid + corners[0];
	corners[3] = tmpLen + tmpWid + corners[0];
	// calculate reference frame for the body

	// second create the actual reference frame --> here on out all corners will use this
	// reference frame lies on the base of the rectangle: between corners 0 and 2, halfway

	pos = corners[2] - corners[0]; // referencing the generator
	pos = ((width/2)*pos)/pos.norm();
	pos += corners[0]; // now this reference frame references the ground frame

	// rotate
	Matrix2f ccRot;
	ccRot <<	cos(theta),		-sin(theta), // the angular orientation
				sin(theta),		cos(theta);

	for(uint8_t i = 0; i<4; i++){
		corners[i] -= pos; // make the corner in reference to reference point
		corners[i] = ccRot * corners[i]; // rotate CC by pi
		corners[i] += pos; // place the corner in reference to ground frame when done
	}

	thetaDot = ((2*pi)/2)*dt;
	theta2Dot = (pi/100)*dt;
	// theta2Dot = ((2*pi)/200)*dt;

	// thetaDot = (2*pi)*dt; // full rotation in 2 seconds
	// theta2Dot = ((2*pi)/100)*dt;

	update();
	return;
}

// creates painter pixels
// borders (X and Y ranges)
void Rectangle::update(){
	painter.clear();

	// figure out the borders
	// pos represents one corner

	// std::cout << "\ncorners[0]:\n" << corners[0].transpose() << std::endl;
	// std::cout << "\ncorners[1]:\n" << corners[1].transpose() << std::endl;
	// std::cout << "\ncorners[2]:\n" << corners[2].transpose() << std::endl;
	// std::cout << "\ncorners[3]:\n" << corners[3].transpose() << std::endl;

	max_x = max_y = std::numeric_limits<int32_t>::min();
	min_x = min_y = std::numeric_limits<int32_t>::max();

	for(uint8_t i = 0; i < 4; i++){
		min_x =	( roundHelper(corners[i](coordinate::X)*PPM) < min_x )?  roundHelper(corners[i](coordinate::X)*PPM) : min_x ;
		max_x = ( roundHelper(corners[i](coordinate::X)*PPM) > max_x )?  roundHelper(corners[i](coordinate::X)*PPM) : max_x ;
		min_y = ( roundHelper(corners[i](coordinate::Y)*PPM) < min_y )?  roundHelper(corners[i](coordinate::Y)*PPM) : min_y ;
		max_y = ( roundHelper(corners[i](coordinate::Y)*PPM) > max_y )?  roundHelper(corners[i](coordinate::Y)*PPM) : max_y ;
	}
	// std::cout << "min_x:\t" << min_x << std::endl;
	// std::cout << "max_x:\t" << max_x << std::endl;
	// std::cout << "min_y:\t" << min_y << std::endl;
	// std::cout << "max_y:\t" << max_y << std::endl;

	// special case or not --> need to find X and Y coordinate copy on another corner
	bool specialCase = false;
	uint8_t inc = 0;

	// THIS IS THE SOURCE OF SOOOO MANY BUGS
	for(uint8_t i = 1; i < 4; i++){
		// check corner[0] quantized with the rest
		// DUE TO FLOATING POINT ERRORS, WILL ALWAYS NEED TO COMPARE BY USING:
			// +- some epsilon (infintesimally small fraction)
			// converting data points to a quantized, agreed upon scale
		// scale by PPM and + 0.5 for flooring. If they end up on the SAME PIXEL, then we increment
		inc += (( roundHelper(corners[0](coordinate::X)*PPM) == roundHelper(corners[i](coordinate::X)*PPM) ) || 
				( roundHelper(corners[0](coordinate::Y)*PPM) == roundHelper(corners[i](coordinate::Y)*PPM) ))? 1 : 0;
		specialCase = (inc == 2)? true : false; 
		if(specialCase){
			break; // stop checking
		}
	}
	// std::set<> Set;
	// special case --> paint every loop!
	Vector2f * painterCorners[4]; // references an address --> know if there are copies
	int32_t borderVals[4] = {min_x, max_x, min_y, max_y};
	if(!specialCase){ // need to put our rectangle corners in standard order for painting
		for(uint8_t i = 0; i < 4; i++){ // corresponds to borderVals AND painterCorners (trying to map borderVals to corner standard form)
			// iterator over the 4 border values --> check if the corresponding corner's coordinate matches
			// j < border::MIN_Y == checking x's
			// j > border::MAX_X == checking y's
			for(uint8_t j=0; j < 4; j++){ // iterate over all rectangle members: corner[0-3]
				painterCorners[i] =	( (i < border::MIN_Y) && ( roundHelper(corners[j](coordinate::X)*PPM) == borderVals[i] ) )? &corners[j] :
									( (i > border::MAX_X) && ( roundHelper(corners[j](coordinate::Y)*PPM) == borderVals[i] ) )? &corners[j] : painterCorners[i];
			}
		}
	}
	// returns an optional --> tuple: address copy, both indexes found at
	auto check = checkCopies(painterCorners);
	if(check.has_value()){ // there are 2 copies of something
		// uint8_t indices[2] = {std::get<1>(check.value()),std::get<2>(check.value())}; // these are the only 2 to check for border vals
		// can do set.count(KEY_VALUE) to determine if it exists in my set
		uint8_t missingIdx = 0;
		for(uint8_t i = 0; i < 4; i++){
			missingIdx =	( check.value().count(&corners[i]))? missingIdx : i; // returns 0 if it doesn't exist in the set
		}
		for(uint8_t i = 0; i < 4; i++){ // if there's another bug, it might be because corner[0] overwrote multiple..
			painterCorners[i] =	( (i < border::MIN_Y) && ( roundHelper(corners[missingIdx](coordinate::X)*PPM) == borderVals[i] ) )? &corners[missingIdx] :
								( (i > border::MAX_X) && ( roundHelper(corners[missingIdx](coordinate::Y)*PPM) == borderVals[i] ) )? &corners[missingIdx] : painterCorners[i];
		}
	} // checkCopies again?

	// @TODO: Range clipping on the upper bounds
	for(uint16_t i = (min_x >=0)? min_x: 0; i<max_x; i++){
		for(uint16_t j = (min_y >=0)? min_y: 0; j<max_y; j++){
			if(specialCase){
				painter.newPixel(i,j);
			} else { // need to do piecewise line checking..
				Vector2f topCheck[2];
				Vector2f botCheck[2];
				// reads: 	line 1 = ternary
				//			line 2 = return values

				topCheck[0] = (i <= roundHelper(static_cast<Vector2f>(*painterCorners[border::MAX_Y])(coordinate::X)*PPM) )? 
						static_cast<Vector2f>(*painterCorners[border::MIN_X]) : static_cast<Vector2f>(*painterCorners[border::MAX_Y]);
				topCheck[1] = (i <= roundHelper(static_cast<Vector2f>(*painterCorners[border::MAX_Y])(coordinate::X)*PPM) )? 
						static_cast<Vector2f>(*painterCorners[border::MAX_Y]) : static_cast<Vector2f>(*painterCorners[border::MAX_X]);

				botCheck[0] = (i <= roundHelper(static_cast<Vector2f>(*painterCorners[border::MIN_Y])(coordinate::X)*PPM) )? 
						static_cast<Vector2f>(*painterCorners[border::MIN_X]) : static_cast<Vector2f>(*painterCorners[border::MIN_Y]);
				botCheck[1] = (i <= roundHelper(static_cast<Vector2f>(*painterCorners[border::MIN_Y])(coordinate::X)*PPM) )? 
						static_cast<Vector2f>(*painterCorners[border::MIN_Y]) : static_cast<Vector2f>(*painterCorners[border::MAX_X]);
				// top line checking
				Vector2f pointCheck(static_cast<float>(i)/PPM,static_cast<float>(j)/PPM);
				
				// bottom line checking
				
				if( ( lineCheck(topCheck[0],topCheck[1],pointCheck,true) ) && 
					( lineCheck(botCheck[0],botCheck[1],pointCheck,false) )){
					painter.newPixel(i,j);
				}
			}
		}
	}
	return;
}

// checks piont for being inside rectangle bounds
bool Rectangle::lineCheck(Vector2f p1, Vector2f p2, Vector2f pIn, bool under){
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
	// should I quantize these coordinates?
	if(under){ // check below decider
		retVal = ((check(coordinate::Y)) <= (decider(coordinate::Y)))? true : false;
	} else { // check above decider
		retVal = ((check(coordinate::Y)) >= (decider(coordinate::Y)))? true : false;
	}
	return retVal;
}

// updates velocities (accelerations too? not now)
// updates the 4 corners to generate pixels from

void Rectangle::move(){
	Matrix3f velAf;
	// Matrix3f accAf;
	// is the acceleration Affine even necessary?
	// I update velocity vector from acceleration, then update angular as scalar..
	velAf <<	cos(thetaDot),	-sin(thetaDot),	vel(coordinate::X),
				sin(thetaDot),	cos(thetaDot),	vel(coordinate::Y),
				0,0,1;

	// accAf <<	cos(theta2Dot),		sin(theta2Dot),acc(coordinate::X),
	// 			-sin(theta2Dot),	cos(theta2Dot),acc(coordinate::Y),
	// 			0,0,1;

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
	return;
}

void Rectangle::bounce(uint16_t _width, uint16_t _height){
	return;
}

int32_t roundHelper(float in){
	int32_t retVal;
	if(in < 0){
		retVal = static_cast<int32_t>(in - 0.5);
	} else {
		retVal = static_cast<int32_t>(in + 0.5);
	}
	return retVal;
}

// std::get<0 or 1> for the pointer
// c++17 structured binding:
	// auto [ address0, address1 ] = checkCopies(painterCorners);
// returns the set of addresses and the 2 targeted indices
// dont need tuple anymore, but it's really good to know
// std::optional<std::tuple<std::set<Vector2f*>>>
std::optional<std::set<Vector2f*>> checkCopies(Vector2f ** in){
	uint8_t offset = 0;
	while(offset < 3){
			for(uint8_t i = offset + 1; i < 4; i++){
				if(in[offset] == in[i]){ //
					std::set<Vector2f*> retSet;
					for(uint8_t j = 0; j < 4; j++){
						retSet.insert(in[j]);
					}
					// return { std::make_tuple(retSet) }; // tuple might be useless for me lol
					return retSet;
				}
			}
		offset++;
	}
	return std::nullopt;
}

// i learned a no no today :)
