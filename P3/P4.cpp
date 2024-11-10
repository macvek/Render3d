#include <iostream>
#include <cmath>
#include <SDL.h>
#include <vector>
#include <cstdlib>

using namespace std;

#define SWIDTH 800
#define SHEIGHT 600

typedef double real;

const real SCREEN_WIDTH = SWIDTH;
const real SCREEN_HEIGHT = SHEIGHT;

real FOV = 90;

Uint32 globalCustomEventId = 0;
 
bool render = true;
bool useOrtho = false;
bool paused = true;
Uint32 tickFrame(Uint32 interval, void* param) {
	SDL_Event e = {};
	e.type = globalCustomEventId;
	if (render) {
		SDL_PushEvent(&e);
	}

	return interval;
}

struct DoublePoint {
	real x, y, z;
};

struct Naive_Vertex
{
	DoublePoint position;
	SDL_Color  color;
	SDL_FPoint tex_coord;
};

Uint32 emptyBuffer[SWIDTH * SHEIGHT];
real zBuffer[SWIDTH * SHEIGHT];

Uint32* backbuffer;


int backbufferPitch;
SDL_Texture* backTexture = nullptr;

SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
void Naive_RenderGeometry(SDL_Renderer* renderer, SDL_Texture*, const Naive_Vertex* vertices, int num_vertices, const int* indices, int num_indices, bool useZ);

Uint32 COLOR(Uint8 r, Uint8 g, Uint8 b) {
	return 0xFF << 24 | b << 16 | g << 8 | r;
}

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
	memset(zBuffer, 0, SWIDTH * SHEIGHT * sizeof(double));
}


void Naive_SetBackbufferPoint(int x, int y, Uint8 r, Uint8 g, Uint8 b) {
	backbuffer[y * SWIDTH + x] = COLOR(r,g,b);
}


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
};

struct BarycentricLambdas {
	real l1;
	real l2;
	real l3;
};



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

	ret.x3 = x3;
	ret.y3 = y3;
}

void calcLambdaForPoint(BarycentricLambdas& ret, BaryPrecalcLambdas& precalc, real x, real yMINUSy3) {

	real xMINUSx3 = x - precalc.x3;

	ret.l1 = (precalc.y2MINUSy3 * xMINUSx3 + precalc.x3MINUSx2 * yMINUSy3) * precalc.oneOverDetT;
	ret.l2 = (precalc.y3MINUSy1 * xMINUSx3 + precalc.x1MINUSx3 * yMINUSy3) * precalc.oneOverDetT;

	ret.l3 = 1 - ret.l1 - ret.l2;
}

void calcXYForLambda(SDL_FPoint& ret, BarycentricLambdas& lambdas, BarycentricForTriangle& coords) {
	ret.x = lambdas.l1 * coords.a->position.x + lambdas.l2 * coords.b->position.x + lambdas.l3 * coords.c->position.x;
	ret.y = lambdas.l1 * coords.a->position.y + lambdas.l2 * coords.b->position.y + lambdas.l3 * coords.c->position.y;
}


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
		m[0][0] = 1/right;	m[0][1] = 0;	 m[0][2] = 0;			  m[0][3] = 0;
		m[1][0] = 0;		m[1][1] = 1/top; m[1][2] = 0;			  m[1][3] = 0;
		m[2][0] = 0;		m[2][1] = 0;	 m[2][2] = -2/(far-near); m[2][3] = - (far + near) / (far - near);
		m[3][0] = 0;		m[3][1] = 0;	 m[3][2] = 0;			  m[3][3] = 1;
	}

	void InitAsPerspective(real right, real top, real far, real near) {
		m[0][0] = near/right; m[0][1] = 0;		  m[0][2] = 0;			  m[0][3] = 0;
		m[1][0] = 0;		  m[1][1] = near/top; m[1][2] = 0;			  m[1][3] = 0;
		m[2][0] = 0;		  m[2][1] = 0;		  m[2][2] = -(far + near) / (far - near) ; m[2][3] = -2 * far * near / (far - near);
		m[3][0] = 0;		  m[3][1] = 0;		  m[3][2] = -1;			  m[3][3] = 0;
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
	
	DoublePoint ApplyOnPoint(DoublePoint& p) {
		real nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		real nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		real nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];

		return DoublePoint{ nX , nY, nZ };
	}

	DoublePoint Projection(DoublePoint& p) {
		real nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		real nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		real nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
		real nW = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];

		return DoublePoint{ nX/nW , nY/nW, nZ/nW };
	}

};

struct Shape {
	std::vector<DoublePoint> points;
};

real degToRad(real a) {
	return a * M_PI / 180;
}

void projectShapeToVertices(M44& toApply, M44& toProject, Shape& shape, std::vector<Naive_Vertex>& outVertices) {
	SDL_Color colors[3] = {
		{255,0,0, 255}, {0,255,0, 255}, {0,0,255, 255}
	};
	
	int colorPtr = 0;
	for (auto ptr = shape.points.begin(); ptr < shape.points.end(); ++ptr) {
		DoublePoint transformed = toApply.ApplyOnPoint(*ptr);
		
		DoublePoint projected = toProject.Projection(transformed);
		Naive_Vertex each;
		each.position.x = (1+projected.x) * (SCREEN_WIDTH / 2);
		each.position.y = (1+projected.y) * (SCREEN_HEIGHT / 2);
		each.position.z = 1 - projected.z;
		each.color = colors[colorPtr % 3];
		++colorPtr;

		outVertices.push_back(each);
	}
}

void renderShape(Shape& shape, M44& toApply, M44& projection, bool useZ) {
	std::vector<Naive_Vertex> vertices;
	projectShapeToVertices(toApply, projection, shape, vertices);

	int indices[6];
	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 3;

	indices[3] = 1;
	indices[4] = 2;
	indices[5] = 3;

	Naive_RenderGeometry(renderer, nullptr, vertices.data(), vertices.size(), indices, 6, useZ);
}

Shape faceFront = { { {-50,-50, 10}, {50,-50, 10}, {50,50, 10}, {-50,50, 10} } };
Shape faceMiddle = { { {-50,-50,0}, {50,-50, 0}, {50,50, 0}, {-50,50, 0} } };
Shape faceBack = { { {-50,-50, -10}, {50,-50, -10}, {50,50, -10}, {-50,50, -10} } };

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
void renderFrame() {
	backbuffer = nullptr;
	if (SDL_LockTexture(backTexture, nullptr, (void**)&backbuffer, &backbufferPitch)) {
		cout << "SDL_LockTexture: " << SDL_GetError() << endl;
	}
	resetBackBuffer();

	M44 toApply;
	toApply.InitAsTranslate(0, 0, -100);

	real angleZ = (frame / 400.0) * M_PI;
	M44 rotZ; rotZ.InitAsRotateZ(angleZ);

	real angleY = angleZ / 2;
	M44 rotY; rotY.InitAsRotateY(angleY);

	real angleX = angleZ / 3;
	M44 rotX; rotX.InitAsRotateY(angleX);

	toApply.Mult(rotX);
	toApply.Mult(rotY);
	toApply.Mult(rotZ);

	M44 ident;
	ident.InitAsIdentity();

	M44 projection;


	if (useOrtho) {
		projection.InitAsOrthographic(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 200, 1);
	}
	else {

		real aspectRatio = SCREEN_WIDTH / SCREEN_HEIGHT;
		real front = 1;
		real tangent = tan(FOV / 2 * M_PI / 180);
		real right = front * tangent;
		real top = right / aspectRatio;

		projection.InitAsPerspective(right, top, 200, 1);
	}

	
	renderShape(faceBack, toApply, projection, true);
	renderShape(faceMiddle, toApply, projection, true);
	renderShape(faceFront, toApply, projection, true);

	renderShape(faceBack, toApply, projection, false);
	renderShape(faceMiddle, toApply, projection, false);
	renderShape(faceFront, toApply, projection, false);
	
	SDL_UnlockTexture(backTexture);
	Naive_FlushBuffer();
	SDL_RenderPresent(renderer);
	if (!paused) {
		++frame;
		cout << "FRAME: " << frame << endl;
	}
}



void Naive_DrawTriangleLine(SDL_Renderer* renderer, BarycentricForTriangle& coords, BaryPrecalcLambdas &precalc, int leftX, int rightX, int lineY, bool useZ) {
	BarycentricLambdas lambdas;

	DoublePoint point;
	point.y = lineY;
	real yMINUSy3 = point.y - precalc.y3;

	for (int x = leftX; x <= rightX; ++x) {
		calcLambdaForPoint(lambdas, precalc, x, yMINUSy3);
		int buffIdx = lineY * SWIDTH + x;
		
		real zValue = coords.a->position.z * lambdas.l1 + coords.b->position.z * lambdas.l2 + coords.c->position.z * lambdas.l3;
		if (useZ) {
			zBuffer[buffIdx] = std::max(zValue, zBuffer[buffIdx]);
		}
		else if (zValue >= zBuffer[buffIdx]) {
			Uint8 r = coords.a->color.r * lambdas.l1 + coords.b->color.r * lambdas.l2 + coords.c->color.r * lambdas.l3;
			Uint8 g = coords.a->color.g * lambdas.l1 + coords.b->color.g * lambdas.l2 + coords.c->color.g * lambdas.l3;
			Uint8 b = coords.a->color.b * lambdas.l1 + coords.b->color.b * lambdas.l2 + coords.c->color.b * lambdas.l3;

			Naive_SetBackbufferPoint(x, lineY, r, g, b);
		}
	}
}

bool isBetweenPoints(int y, const DoublePoint& a, const DoublePoint& b) {
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
double pointOnLineAt(int y, const DoublePoint& a, const DoublePoint& b) {
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

void Naive_FillVertices(SDL_Renderer* renderer, const Naive_Vertex* vertices, const int* indices, bool useZ) {
	BarycentricForTriangle coords;
	BaryPrecalcLambdas precalc;

	const Naive_Vertex *picked[3];

	picked[0] = vertices + *(indices + 0);
	picked[1] = vertices + *(indices + 1);
	picked[2] = vertices + *(indices + 2);

	setupBarycentricForTriangle(coords, picked[0], picked[1], picked[2]);
	precalcLambda(precalc, coords);

	real top = SCREEN_HEIGHT - 1;
	real bottom = 0;

	for (int i = 0; i < 3; ++i) {
		top = std::min(picked[i]->position.y, top);
		bottom = std::max(picked[i]->position.y, bottom);
	}

	int yStart = std::max((int)top,0);
	int yEnd = std::min((int)bottom, SHEIGHT-1);

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

		lX = std::min(SCREEN_WIDTH - 1, std::max(0.0, lX));
		rX = std::min(SCREEN_WIDTH - 1, std::max(0.0, rX));

		if (lX <= rX) {
			Naive_DrawTriangleLine(renderer, coords, precalc, lX, rX, y, useZ);
		}
	}
}



void Naive_RenderGeometry(SDL_Renderer* renderer, SDL_Texture* , const Naive_Vertex* vertices, int num_vertices, const int* indices, int num_indices, bool useZ) {
	Naive_FillVertices(renderer, vertices, indices + 3, useZ);
	Naive_FillVertices(renderer, vertices, indices, useZ);
}

int main(int argc, char* argv[])
{
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
						++frame;
					}

					if (key == SDL_SCANCODE_Q) {
						--frame;
					}

					if (key == SDL_SCANCODE_O) {
						useOrtho = !useOrtho;
					}

					if (key == SDL_SCANCODE_P) {
						paused = !paused;
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

