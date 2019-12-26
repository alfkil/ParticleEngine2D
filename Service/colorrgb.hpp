#ifndef COLOR_hpp
#define COLOR_hpp

class ColorRGB {
	public:
		enum ColorID {
			COLOR_RED,
			COLOR_GREEN,
			COLOR_BLUE,
			COLOR_WHITE
		};

	private:
		double r, g, b;

	public:
		ColorRGB (double red, double green, double blue) :
			r(red),
			g(green),
			b(blue)
		{ }

		ColorRGB (ColorID color) {
			switch (color) {
				case COLOR_RED:
					r = 1.0; g = 0.0; b = 0.0;
					break;
				case COLOR_GREEN:
					r = 0.0; g = 1.0; b = 0.0;
					break;
				case COLOR_BLUE:
					r = 0.0; g = 0.0; b = 1.0;
					break;
				case COLOR_WHITE:
					r = 1.0; g = 1.0; b = 1.0;
					break;
			}
		}

		double red () {
			return r;
		}

		double green () {
			return g;
		}

		double blue () {
			return b;
		}

		unsigned int hex () {
			return 0xff000000
			+ 0x00ff0000 * (int)r
			+ 0x0000ff00 * (int)g
			+ 0x000000ff * (int)b;
		}
};
#endif
