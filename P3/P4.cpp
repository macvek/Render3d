/*
MIT License

Copyright (c) 2024 Maciej Aleksandrowicz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <iostream>
#include <cmath>
#include <SDL.h>
#include <vector>
#include <algorithm>
#include <cstdlib>

using namespace std;

#define SWIDTH 800
#define SHEIGHT 600

typedef float real;

const real SCREEN_WIDTH = SWIDTH;
const real SCREEN_HEIGHT = SHEIGHT;

real FOV = 90;

Uint32 globalCustomEventId = 0;
 
bool render = true;
bool useOrtho = false;
bool paused = false;
bool enableZTest = true;
bool showZBuffer = false;

struct TriangleLambdas {
	real l1;
	real l2;
	real l3;
};

real zBuffer[SWIDTH * SHEIGHT];

Uint32 emptyBuffer[SWIDTH * SHEIGHT];
Uint32* backbuffer;


Uint32 tickFrame(Uint32 interval, void* param) {
	SDL_Event e = {};
	e.type = globalCustomEventId;
	if (render) {
		SDL_PushEvent(&e);
	}

	return interval;
}

struct RealPoint {
	real x, y, z;
};

enum CoordSystem {
	WORLD = 0,
	CAMERA,
	PROJECTED,
	SCREEN
};

struct Naive_Vertex
{
	RealPoint position;
	SDL_Color  color;
	SDL_FPoint tex_coord;
	CoordSystem coordSystem;
};

struct M44 {
	real m[4][4];

	void InitAsIdentity() {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsScale(real sx, real sy, real sz) {
		m[0][0] = sx; m[0][1] = 0;  m[0][2] = 0;  m[0][3] = 0;
		m[1][0] = 0;  m[1][1] = sy; m[1][2] = 0;  m[1][3] = 0;
		m[2][0] = 0;  m[2][1] = 0;  m[2][2] = sz; m[2][3] = 0;
		m[3][0] = 0;  m[3][1] = 0;  m[3][2] = 0;  m[3][3] = 1;
	}

	void InitAsRotateX(real phi) {
		m[0][0] = 1; m[0][1] = 0;			m[0][2] = 0;		 m[0][3] = 0;
		m[1][0] = 0; m[1][1] = cos(phi);	m[1][2] = -sin(phi); m[1][3] = 0;
		m[2][0] = 0; m[2][1] = sin(phi);	m[2][2] = cos(phi);	 m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0;			m[3][2] = 0;		 m[3][3] = 1;
	}

	void InitAsRotateY(real phi) {
		m[0][0] = cos(phi);   m[0][1] = 0;			m[0][2] = sin(phi);	m[0][3] = 0;
		m[1][0] = 0;		  m[1][1] = 1;		   	m[1][2] = 0;		m[1][3] = 0;
		m[2][0] = -sin(phi);  m[2][1] = 0;			m[2][2] = cos(phi); m[2][3] = 0;
		m[3][0] = 0;		  m[3][1] = 0;			m[3][2] = 0;		m[3][3] = 1;
	}


	void InitAsRotateZ(real phi) {
		m[0][0] = cos(phi);   m[0][1] = -sin(phi);	m[0][2] = 0; m[0][3] = 0;
		m[1][0] = sin(phi);	  m[1][1] = cos(phi);	m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0;		  m[2][1] = 0;			m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0;		  m[3][1] = 0;			m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsTranslate(real x, real y, real z) {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = x;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = y;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = z;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsOrthographic(real right, real top, real far, real near) {
		m[0][0] = 1 / right;	m[0][1] = 0;		m[0][2] = 0;				m[0][3] = 0;
		m[1][0] = 0;			m[1][1] = 1 / top;	m[1][2] = 0;				m[1][3] = 0;
		m[2][0] = 0;			m[2][1] = 0;		m[2][2] = -2 / (far - near);m[2][3] = -(far + near) / (far - near);
		m[3][0] = 0;			m[3][1] = 0;		m[3][2] = 0;				m[3][3] = 1;
	}

	void InitAsPerspective(real right, real top, real far, real near) {
		m[0][0] = near / right; m[0][1] = 0;		  m[0][2] = 0;							 m[0][3] = 0;
		m[1][0] = 0;			m[1][1] = near / top; m[1][2] = 0;							 m[1][3] = 0;
		m[2][0] = 0;			m[2][1] = 0;		  m[2][2] = -(far + near) / (far - near);m[2][3] = -2 * far * near / (far - near);
		m[3][0] = 0;			m[3][1] = 0;		  m[3][2] = -1;							 m[3][3] = 0;
	}

	void FillFrom(M44& s) {
		for (int y = 0; y < 4; ++y)
			for (int x = 0; x < 4; ++x)
				m[y][x] = s.m[y][x];
	}

	void Mult(M44& o) {
		M44 r;
		for (int l = 0; l < 4; ++l) {
			r.m[l][0] = m[l][0] * o.m[0][0] + m[l][1] * o.m[1][0] + m[l][2] * o.m[2][0] + m[l][3] * o.m[3][0];
			r.m[l][1] = m[l][0] * o.m[0][1] + m[l][1] * o.m[1][1] + m[l][2] * o.m[2][1] + m[l][3] * o.m[3][1];
			r.m[l][2] = m[l][0] * o.m[0][2] + m[l][1] * o.m[1][2] + m[l][2] * o.m[2][2] + m[l][3] * o.m[3][2];
			r.m[l][3] = m[l][0] * o.m[0][3] + m[l][1] * o.m[1][3] + m[l][2] * o.m[2][3] + m[l][3] * o.m[3][3];
		}

		FillFrom(r);
	}


	void Print() {
		cout << " [ " << m[0][0] << "\t" << m[0][1] << "\t" << m[0][2] << "\t" << m[0][3] << " ] " << endl;
		cout << " [ " << m[1][0] << "\t" << m[1][1] << "\t" << m[1][2] << "\t" << m[1][3] << " ] " << endl;
		cout << " [ " << m[2][0] << "\t" << m[2][1] << "\t" << m[2][2] << "\t" << m[2][3] << " ] " << endl;
		cout << " [ " << m[3][0] << "\t" << m[3][1] << "\t" << m[3][2] << "\t" << m[3][3] << " ] " << endl;
	}

	RealPoint ApplyOnPoint(RealPoint& p) {
		real nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		real nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		real nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];

		return RealPoint{ nX , nY, nZ };
	}

	RealPoint Projection(RealPoint& p) {
		real nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		real nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		real nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
		real nW = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];

		return RealPoint{ nX / nW , nY / nW, nZ / nW };
	}

};

struct Shape {
	std::vector<RealPoint> points;
};

struct BarycentricForTriangle {
	real detT;
	real T[2][2];

	const Naive_Vertex* a;
	const Naive_Vertex* b;
	const Naive_Vertex* c;
};

struct BaryPrecalcLambdas {
	real y2MINUSy3;
	real x3MINUSx2;
	real y3MINUSy1;
	real x1MINUSx3;

	real x3;
	real y3;

	real oneOverDetT;

	real prefixL1;
	real prefixL2;
};

struct BaryLinePrecalc {
	real suffixL1;
	real suffixL2;

};

bool isBetweenPoints(int y, const RealPoint& a, const RealPoint& b) {
	int aY = (int)a.y;
	int bY = (int)b.y;

	if (aY <= bY) {
		return y >= aY && y <= bY;
	}
	else {
		return y > bY && y <= aY;
	}
}


// here we already checked that point lays on the line of (a,b)
real pointOnLineAt(int y, const RealPoint& a, const RealPoint& b) {
	int aY = (int)a.y;
	int bY = (int)b.y;

	if (aY == bY) { // pick first option, as vertices during checks never take same place twice
		return a.x;
	}

	// cases of hitting corner
	if (aY == y) {
		return a.x;
	}

	if (bY == y) {
		return b.x;
	}

	return (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x;

}

struct LineCache {
	int top;
	std::vector<std::pair<int, int>> ranges;
};


struct PipelineTriangle { // indices pointing to pipeline->vertices in pipeline
	int indices[3];
	bool onScreen;
	BaryPrecalcLambdas precalc;
	LineCache lines;
	real zIndex;

	// this stage is calculated in normalized (0;1), where lower is closer to camera
	void calcZIndex(const std::vector<Naive_Vertex>& vertices) {
		const Naive_Vertex& a = vertices[indices[0]];
		const Naive_Vertex& b = vertices[indices[1]];
		const Naive_Vertex& c = vertices[indices[2]];

		zIndex = std::max<real>(a.position.z, b.position.z);
		zIndex = std::max<real>(zIndex, c.position.z);
	}

	// this stage is calculated in normalized (-1;1) space
	void calcOnScreen(const std::vector<Naive_Vertex>& vertices) {
		const Naive_Vertex& a = vertices[indices[0]];
		const Naive_Vertex& b = vertices[indices[1]];
		const Naive_Vertex& c = vertices[indices[2]];

		onScreen = !(
			(a.position.x < -1 && b.position.x < -1 && c.position.x < -1) || // all to the left
			(a.position.x > 1 && b.position.x > 1 && c.position.x > 1) || // all to the right

			(a.position.y < -1 && b.position.y < -1 && c.position.y < -1) || // all over the top
			(a.position.y > 1 && b.position.y > 1 && c.position.y > 1) || // all below the bottom

			(a.position.z < 0 && b.position.z < 0 && c.position.z < 0) || // all behind near limit
			(a.position.z > 1 && b.position.z > 1 && c.position.z > 1)  // all behind far limit
		);
	}

	// taken from https://en.wikipedia.org/wiki/Barycentric_coordinate_system
	void calcBarycentric(const std::vector<Naive_Vertex> &vertices) {
		const Naive_Vertex& a = vertices[indices[0]];
		const Naive_Vertex& b = vertices[indices[1]];
		const Naive_Vertex& c = vertices[indices[2]];

		auto x1 = a.position.x;
		auto x2 = b.position.x;
		auto x3 = c.position.x;

		auto y1 = a.position.y;
		auto y2 = b.position.y;
		auto y3 = c.position.y;

		auto t00 = x1 - x3;
		auto t10 = x2 - x3;
		auto t01 = y1 - y3;
		auto t11 = y2 - y3;

		real detT = t00 * t11 - t10 * t01;
		precalc.oneOverDetT = 1 / detT;
		
		precalc.y2MINUSy3 = y2 - y3;
		precalc.y3MINUSy1 = y3 - y1;
		
		precalc.x3MINUSx2 = x3 - x2;
		precalc.x1MINUSx3 = x1 - x3;

		precalc.prefixL1 = precalc.y2MINUSy3 * precalc.oneOverDetT;
		precalc.prefixL2 = precalc.y3MINUSy1 * precalc.oneOverDetT;

		precalc.x3 = x3;
		precalc.y3 = y3;
	}

	void calcLines(const std::vector<Naive_Vertex>& vertices) {
		real top = SCREEN_HEIGHT - 1;
		real bottom = 0;

		const Naive_Vertex* picked[3];

		picked[0] = &vertices[indices[0]];
		picked[1] = &vertices[indices[1]];
		picked[2] = &vertices[indices[2]];

		for (int i = 0; i < 3; ++i) {
			top = std::min(picked[i]->position.y, top);
			bottom = std::max(picked[i]->position.y, bottom);
		}

		int yStart = std::max((int)top, 0);
		int yEnd = std::min((int)bottom, SHEIGHT - 1);

		lines.top = yStart;

		real lX = 0;
		real rX = SCREEN_WIDTH - 1;

		for (int y = yStart; y <= yEnd; ++y) {
			lX = SCREEN_WIDTH;
			rX = 0;

			for (int leftIdx = 0; leftIdx < 3; ++leftIdx) {
				int rightIdx = (leftIdx + 1) % 3;

				if (isBetweenPoints(y, picked[leftIdx]->position, picked[rightIdx]->position)) {
					real x = pointOnLineAt(y, picked[leftIdx]->position, picked[rightIdx]->position);
					lX = std::min(lX, x);
					rX = std::max(rX, x);
				}
			}

			lX = std::min(SCREEN_WIDTH - 1, std::max((real)0.0, lX));
			rX = std::min(SCREEN_WIDTH - 1, std::max((real)0.0, rX));

			if (lX <= rX) {
				lines.ranges.push_back( std::pair<int, int>((int)lX, (int)rX) );
			}
		}
	}
};

void precalcLine(BaryLinePrecalc& ret, BaryPrecalcLambdas& precalc, real coordsY3, int y) {
	real yMINUSy3 = y - coordsY3;

	ret.suffixL1 = precalc.x3MINUSx2 * yMINUSy3 * precalc.oneOverDetT;
	ret.suffixL2 = precalc.x1MINUSx3 * yMINUSy3 * precalc.oneOverDetT;
}

void calcLambdaForPoint(TriangleLambdas& ret, BaryPrecalcLambdas& precalc, BaryLinePrecalc linePrecalc, int x) {

	real xMINUSx3 = x - precalc.x3;

	ret.l1 = (precalc.prefixL1 * xMINUSx3) + linePrecalc.suffixL1;
	ret.l2 = (precalc.prefixL2 * xMINUSx3) + linePrecalc.suffixL2;

	ret.l3 = 1 - ret.l1 - ret.l2;
}

void rangeLambdas(TriangleLambdas& lambdas) {
	TriangleLambdas old = lambdas;

	lambdas.l1 = std::max((real)0.0, std::min((real)1.0, old.l1));
	lambdas.l2 = std::max((real)0.0, std::min((real)1.0, old.l2));
	lambdas.l3 = std::max((real)0.0, std::min((real)1.0, old.l3));
}

Uint32 COLOR(Uint8 r, Uint8 g, Uint8 b) {
	return 0xFF << 24 | b << 16 | g << 8 | r;
}

void Naive_SetBackbufferPoint(int x, int y, Uint8 r, Uint8 g, Uint8 b) {
	backbuffer[y * SWIDTH + x] = COLOR(r, g, b);
}

struct Pipeline {
	M44 worldTransform;
	M44 projection;
	std::vector<Naive_Vertex> vertices;
	std::vector<PipelineTriangle> triangles;
	
	void putShape(const Shape& s) {
		// temporary coloring //
		SDL_Color colors[3] = {
			{255,0,0, 255}, {0,255,0, 255}, {0,0,255, 255}
		};

		int colorPtr = 0;
	
		int lastIndex = (int)vertices.size();
		for (auto ptr = s.points.begin(); ptr < s.points.end(); ++ptr) {

			Naive_Vertex copiedVertex;
			copiedVertex.position = *ptr;
			copiedVertex.color = colors[colorPtr % 3];
			++colorPtr;

			vertices.push_back(copiedVertex);
		}
		
		// TODO, check ordering for future normal calculation
		PipelineTriangle a{ { lastIndex, lastIndex+1, lastIndex+3} };
		PipelineTriangle b{ { lastIndex + 1, lastIndex + 2, lastIndex + 3} };

		triangles.push_back(b);
		triangles.push_back(a);
	}
	
	void applyWorldTransformation() {
		for (auto ptr = vertices.begin(); ptr < vertices.end(); ++ptr) {
			ptr->position = worldTransform.ApplyOnPoint(ptr->position);
			ptr->coordSystem = CAMERA;
		}
	}

	void applyProjection() {
		for (auto ptr = vertices.begin(); ptr < vertices.end(); ++ptr) {
			ptr->position = projection.Projection(ptr->position);
			ptr->coordSystem = PROJECTED;
		}
	}

	void applyClipping() {
		for (auto ptr = triangles.begin(); ptr < triangles.end(); ++ptr) {
			ptr->calcOnScreen(vertices);
			if (ptr->onScreen) {
				ptr->calcZIndex(vertices);
			}
		}
	}

	void applyToScreen() {
		for (auto ptr = triangles.begin(); ptr < triangles.end() && ptr->onScreen; ++ptr) {
			for (int i = 0; i < 3; ++i) {
				auto &v = vertices.at(ptr->indices[i]);
				if (v.coordSystem == PROJECTED) {
					v.position.x = (1 + v.position.x) * (SCREEN_WIDTH / 2);
					v.position.y = (1 + v.position.y) * (SCREEN_HEIGHT / 2);
					v.position.z = 1 - v.position.z;
					v.coordSystem = SCREEN;
				}
			}
		}
	}

	void sortOverZAndOffScreen() {
		std::sort(triangles.begin(), triangles.end(), [](PipelineTriangle& first, PipelineTriangle& second) {
			return first.onScreen && (!second.onScreen || first.zIndex > second.zIndex);
		});
	}


	void precalculateTriangles() {
		for (auto ptr = triangles.begin(); ptr < triangles.end() && ptr->onScreen; ++ptr) {
			ptr->calcBarycentric(vertices);
			ptr->calcLines(vertices);
		}
	}


	void renderZBuffer() {
		if (!enableZTest) {
			return;
		}

		for (auto ptr = triangles.begin(); ptr < triangles.end() && ptr->onScreen; ++ptr) {
			Naive_Vertex& a = vertices[ptr->indices[0]];
			Naive_Vertex& b = vertices[ptr->indices[1]];
			Naive_Vertex& c = vertices[ptr->indices[2]];

			for (int i = 0; i < ptr->lines.ranges.size(); ++i) {
				int lineY = i + ptr->lines.top;
				auto& pair = ptr->lines.ranges[i];
				int leftX = pair.first;
				int rightX = pair.second;

				BaryLinePrecalc linePrecalc;
				precalcLine(linePrecalc, ptr->precalc, c.position.y, lineY);

				for (int x = leftX; x <= rightX; ++x) {
					int buffIdx = lineY * SWIDTH + x;

					TriangleLambdas lambdas;
					calcLambdaForPoint(lambdas, ptr->precalc, linePrecalc, x);

					if (x == leftX || x == rightX) {
						rangeLambdas(lambdas);
					}
					real zValue = a.position.z * lambdas.l1 + b.position.z * lambdas.l2 + c.position.z * lambdas.l3;
					if (zValue > 0 && zValue < 1) {
						zBuffer[buffIdx] = std::max(zValue, zBuffer[buffIdx]);
					}
				}
			}
		}
	}

	void renderBackbuffer() {
		for (auto ptr = triangles.begin(); ptr < triangles.end() && ptr->onScreen; ++ptr) {
			Naive_Vertex& a = vertices[ptr->indices[0]];
			Naive_Vertex& b = vertices[ptr->indices[1]];
			Naive_Vertex& c = vertices[ptr->indices[2]];

			for (int i = 0; i < ptr->lines.ranges.size(); ++i) {
				int lineY = i + ptr->lines.top;
				auto& pair = ptr->lines.ranges[i];
				int leftX = pair.first;
				int rightX = pair.second;

				BaryLinePrecalc linePrecalc;
				precalcLine(linePrecalc, ptr->precalc, c.position.y, lineY);

				for (int x = leftX; x <= rightX; ++x) {
					int buffIdx = lineY * SWIDTH + x;

					TriangleLambdas lambdas;
					calcLambdaForPoint(lambdas, ptr->precalc, linePrecalc, x);

					if (x == leftX || x == rightX) {
						rangeLambdas(lambdas);
					}
					real zValue = a.position.z * lambdas.l1 + b.position.z * lambdas.l2 + c.position.z * lambdas.l3;

					if (!enableZTest || ( zValue >= zBuffer[buffIdx] && zValue < 1)) {
						Uint8 cR = (Uint8)(a.color.r * lambdas.l1 + b.color.r * lambdas.l2 + c.color.r * lambdas.l3);
						Uint8 cG = (Uint8)(a.color.g * lambdas.l1 + b.color.g * lambdas.l2 + c.color.g * lambdas.l3);
						Uint8 cB = (Uint8)(a.color.b * lambdas.l1 + b.color.b * lambdas.l2 + c.color.b * lambdas.l3);

						Naive_SetBackbufferPoint(x, lineY, cR, cG, cB);
					}
				}
			}
		}
	}

	void renderBackbufferFromZ() {
		for (int x = 0; x < SWIDTH; ++x) {
			for (int y = 0; y < SHEIGHT; ++y) {
				int buffIdx = y * SWIDTH + x;
				real v = zBuffer[buffIdx];
				Naive_SetBackbufferPoint(x, y, v*256, v * 256, v * 256);
			}
		}
	}
};


int backbufferPitch;
SDL_Texture* backTexture = nullptr;

SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;

void initEmptyBuffer() {
	Uint32 fill = COLOR(255, 255, 255);
	for (int y = 0; y < SHEIGHT; ++y) {
		for (int x = 0; x < SWIDTH; ++x) {
			emptyBuffer[y * SWIDTH + x] = fill;
		}
	}
}

void resetBackBuffer() {
	if (backbuffer == nullptr) {
		cout << "Expected to have non-null backbuffer" << endl;
		return;
	}
	
	memcpy(backbuffer, emptyBuffer, SWIDTH * SHEIGHT * sizeof(int));
	if (enableZTest) {
		memset(zBuffer, 0, SWIDTH * SHEIGHT * sizeof(real));
	}
}

// taken from https://en.wikipedia.org/wiki/Barycentric_coordinate_system
void setupBarycentricForTriangle(BarycentricForTriangle& coords, const Naive_Vertex* a, const Naive_Vertex* b, const Naive_Vertex* c) {
	coords.a = a;
	coords.b = b;
	coords.c = c;

	auto x1 = coords.a->position.x;
	auto x2 = coords.b->position.x;
	auto x3 = coords.c->position.x;

	auto y1 = coords.a->position.y;
	auto y2 = coords.b->position.y;
	auto y3 = coords.c->position.y;

	coords.T[0][0] = x1 - x3;
	coords.T[1][0] = x2 - x3;
	coords.T[0][1] = y1 - y3;
	coords.T[1][1] = y2 - y3;

	coords.detT = coords.T[0][0] * coords.T[1][1] - coords.T[1][0] * coords.T[0][1];
}

void precalcLambda(BaryPrecalcLambdas& ret, BarycentricForTriangle& coords) {
	ret.oneOverDetT = 1 / coords.detT;
	
	auto x1 = coords.a->position.x;
	auto x2 = coords.b->position.x;
	auto x3 = coords.c->position.x;

	auto y1 = coords.a->position.y;
	auto y2 = coords.b->position.y;
	auto y3 = coords.c->position.y;

	ret.y2MINUSy3 = y2 - y3;
	ret.x3MINUSx2 = x3 - x2;
	ret.y3MINUSy1 = y3 - y1;
	ret.x1MINUSx3 = x1 - x3;

	ret.prefixL1 = ret.y2MINUSy3 * ret.oneOverDetT;
	ret.prefixL2 = ret.y3MINUSy1 * ret.oneOverDetT;

	ret.x3 = x3;
	ret.y3 = y3;
}

Shape generateFaceAt(real x, real y, real c) {
	real w = 50;
	real h = 50;

	return { { {-w+x,-h+y, c}, {w+x,-h+y, c}, {w+x,h+y, c}, {-w+x,h+y, c} } };
}

void Naive_FlushBuffer() {
	SDL_Vertex vertices[4];
	int indices[6] = { 0, 1, 2, 1, 2, 3 };

	vertices[0].color = { 255,255,255,255 };
	vertices[0].position = { 0,0 };
	vertices[0].tex_coord = { 0.0, 0.0 };

	vertices[1].color = { 255,255,255,255 };
	vertices[1].position = { SWIDTH ,0 };
	vertices[1].tex_coord = { 1.0, 0.0 };

	vertices[2].color = { 255,255,255,255 };
	vertices[2].position = { 0, SHEIGHT };
	vertices[2].tex_coord = { 0.0, 1.0 };

	vertices[3].color = { 255,255,255,255 };
	vertices[3].position = { SWIDTH, SHEIGHT };
	vertices[3].tex_coord = { 1.0, 1.0 };

	SDL_RenderGeometry(renderer,backTexture, vertices, 4, indices, 6);
}

int frame = 0;

std::vector<Shape> allShapes;

void renderFrame() {
	backbuffer = nullptr;
	if (SDL_LockTexture(backTexture, nullptr, (void**)&backbuffer, &backbufferPitch)) {
		cout << "SDL_LockTexture: " << SDL_GetError() << endl;
	}
	resetBackBuffer();

	Pipeline p{};
	// TODO: worldTransform can be understood as camera; to rethink how to efficiently place items dynamically before reaching this point
	p.worldTransform.InitAsTranslate(0, 0, -100);

	real angleZ = (frame / 400.0) * M_PI;
	M44 rotZ{}; rotZ.InitAsRotateZ(angleZ);

	real angleY = angleZ / 2;
	M44 rotY{}; rotY.InitAsRotateY(angleY);

	real angleX = angleZ / 3;
	M44 rotX{}; rotX.InitAsRotateY(angleX);

	p.worldTransform.Mult(rotX);
	p.worldTransform.Mult(rotY);
	p.worldTransform.Mult(rotZ);

	double far = 250;
	double near = 20;

	if (useOrtho) {
		p.projection.InitAsOrthographic(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, far, near);
	}
	else {
		real aspectRatio = SCREEN_WIDTH / SCREEN_HEIGHT;
		real tangent = tan(FOV / 2 * M_PI / 180);
		real right = near * tangent;
		real top = right / aspectRatio;

		p.projection.InitAsPerspective(right, top, far, near);
	}

	for (auto ptr = allShapes.begin(); ptr < allShapes.end(); ++ptr) {
		p.putShape(*ptr);
	}

	p.applyWorldTransformation();
	// TODO: before projection, clipping must be performed based on clipping planes, more here: https://gabrielgambetta.com/computer-graphics-from-scratch/11-clipping.html
	p.applyProjection();
	p.applyClipping();
	p.sortOverZAndOffScreen();
	p.applyToScreen();
	p.precalculateTriangles();
	p.renderZBuffer();
	if (showZBuffer) {
		p.renderBackbufferFromZ();
	}
	else {
		p.renderBackbuffer();
	}

	SDL_UnlockTexture(backTexture);
	Naive_FlushBuffer();
	SDL_RenderPresent(renderer);
}

void updateFrame(int offset) {
	frame += offset;
	cout << "FRAME: " << frame << endl;

}

int main(int argc, char* argv[])
{
	for (int i = 0; i < 30; i += 5) {
		allShapes.push_back(generateFaceAt(0, 0, i));
	}
	

	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		cout << "Error initializing SDL: " << SDL_GetError() << endl;
		return 1;
	}

	globalCustomEventId = SDL_RegisterEvents(1);
	if (((Uint32)-1) == globalCustomEventId) {
		cout << "Failed to get registered event.." << endl;
		return -1;
	}

	int registeredTimer = SDL_AddTimer(35, tickFrame, nullptr);
	window = SDL_CreateWindow("3d preview", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, (int)SCREEN_WIDTH, (int)SCREEN_HEIGHT, 0);

	if (nullptr == window) {
		cout << "Error creating window: " << SDL_GetError() << endl;
		return 1;
	}

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
	if (nullptr == renderer) {
		cout << "Error getting renderer: " << SDL_GetError() << endl;
		return 1;
	}

	backTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, SWIDTH, SHEIGHT);
	initEmptyBuffer();
	renderFrame();
	SDL_RenderPresent(renderer);

	SDL_Event e;
	for (;;) {
		if (SDL_WaitEvent(&e)) {
			if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
				SDL_KeyboardEvent* keyboardEvent = (SDL_KeyboardEvent*)(&e);
				auto key = keyboardEvent->keysym.scancode;
				if (key == SDL_SCANCODE_ESCAPE) {
					break;
				}

				if (e.type == SDL_KEYUP) {

					if (key == SDL_SCANCODE_R) {
						updateFrame(1);
						renderFrame();
					}

					if (key == SDL_SCANCODE_Q) {
						updateFrame(-1);
						renderFrame();
					}

					if (key == SDL_SCANCODE_O) {
						useOrtho = !useOrtho;
					}

					if (key == SDL_SCANCODE_P) {
						paused = !paused;
						cout << "PAUSED " << paused << endl;
					}

					if (key == SDL_SCANCODE_W) {
						render = !render;
					}

					if (key == SDL_SCANCODE_Z) {
						showZBuffer = !showZBuffer;
					}

					if (key == SDL_SCANCODE_KP_PLUS) {
						FOV += 5;
						cout << "FOV: " << FOV << endl;
					}

					if (key == SDL_SCANCODE_KP_MINUS) {
						FOV -= 5;
						cout << "FOV: " << FOV << endl;
					}

				}

			}
			else if (e.type == globalCustomEventId) {
				if (!paused) {
					updateFrame(1);
				}
				renderFrame();
			}
			else if (e.type == SDL_QUIT) {
				break;
			}
		}
		else {
			cout << "SDL_WaitEvent: " << SDL_GetError() << endl;
			break;
		}
	}

	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}

