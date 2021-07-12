#ifndef VECTOR2D_hpp
#define VECTOR2D_hpp

#include <math.h>

class Vector2D {
	private:
		double _x, _y;

	public:
		Vector2D () {}
		Vector2D (double x, double y) {
			_x = x;
			_y = y;
		}

		Vector2D operator + (Vector2D const &other) {
			return Vector2D (_x + other._x, _y + other._y);
		}

		Vector2D operator - (Vector2D const &other) {
			return Vector2D (_x - other._x, _y - other._y);
		}

		Vector2D operator * (double d) {
			return Vector2D (_x * d, _y * d);
		}

		Vector2D operator / (double i) {
			return Vector2D (_x / i, _y / i);
		}

		Vector2D perp () {
			return Vector2D (_y, -_x);
		}

		double perpdot (Vector2D const &other) {
			return _x * other._y - _y * other._x;
		}

		Vector2D operator += (Vector2D const &other) {
			_x += other._x;
			_y += other._y;
			return *this;
		}

		Vector2D operator -= (Vector2D const &other) {
			_x -= other._x;
			_y -= other._y;
			return *this;
		}

		bool operator == (Vector2D const &other) {
			return _x == other._x && _y == other._y;
		}
		
		double length () {
			return sqrt (_x*_x + _y*_y);
		}

		Vector2D eigen () {
			double _length = length ();
			return Vector2D (_x / _length, _y / _length);
		}

		double dot (Vector2D other) {
			return _x * other._x + _y * other._y;
		}

		double x() {
			return _x;
		}

		double y() {
			return _y;
		}

		double projectOnto (Vector2D other) {
			return (_x * other._x + _y * other._y) / other.length();
		}

		void scale(double p) {
			_x *= p; _y *= p;
		}
};
#endif
