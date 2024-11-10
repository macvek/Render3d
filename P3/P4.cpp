#include <iostream>
#include <cmath>
#include <SDL.h>
#include <vector>
#include <cstdlib>

using namespace std;

double SCREEN_WIDTH = 800;
double SCREEN_HEIGHT = 600;

double FOV = 90;

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

SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
void Naive_RenderGeometry(SDL_Renderer* renderer, SDL_Texture*, const SDL_Vertex* vertices, int num_vertices, const int* indices, int num_indices);

struct DoublePoint {
	double x, y, z;
};

struct BarycentricForTriangle {
	float detT;
	float T[2][2];

	const SDL_Vertex* a;
	const SDL_Vertex* b;
	const SDL_Vertex* c;
};

struct BarycentricLambdas {
	float l1;
	float l2;
	float l3;
};



// taken from https://en.wikipedia.org/wiki/Barycentric_coordinate_system
void setupBarycentricForTriangle(BarycentricForTriangle& coords, const SDL_Vertex* a, const SDL_Vertex* b, const SDL_Vertex* c) {
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

void calcLambdaForPoint(BarycentricLambdas& ret, BarycentricForTriangle& coords, const SDL_FPoint& point) {
	auto x1 = coords.a->position.x;
	auto x2 = coords.b->position.x;
	auto x3 = coords.c->position.x;

	auto y1 = coords.a->position.y;
	auto y2 = coords.b->position.y;
	auto y3 = coords.c->position.y;

	ret.l1 = ((y2 - y3) * (point.x - x3) + (x3 - x2) * (point.y - y3)) / coords.detT;
	ret.l2 = ((y3 - y1) * (point.x - x3) + (x1 - x3) * (point.y - y3)) / coords.detT;
	ret.l3 = 1 - ret.l1 - ret.l2;
}

void calcXYForLambda(SDL_FPoint& ret, BarycentricLambdas& lambdas, BarycentricForTriangle& coords) {
	ret.x = lambdas.l1 * coords.a->position.x + lambdas.l2 * coords.b->position.x + lambdas.l3 * coords.c->position.x;
	ret.y = lambdas.l1 * coords.a->position.y + lambdas.l2 * coords.b->position.y + lambdas.l3 * coords.c->position.y;
}


struct M44 {
	double m[4][4];

	void InitAsIdentity() {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsScale(double sx, double sy, double sz) {
		m[0][0] = sx; m[0][1] = 0;  m[0][2] = 0;  m[0][3] = 0;
		m[1][0] = 0;  m[1][1] = sy; m[1][2] = 0;  m[1][3] = 0;
		m[2][0] = 0;  m[2][1] = 0;  m[2][2] = sz; m[2][3] = 0;
		m[3][0] = 0;  m[3][1] = 0;  m[3][2] = 0;  m[3][3] = 1;
	}

	void InitAsRotateX(double phi) {
		m[0][0] = 1; m[0][1] = 0;			m[0][2] = 0;		 m[0][3] = 0;
		m[1][0] = 0; m[1][1] = cos(phi);	m[1][2] = -sin(phi); m[1][3] = 0;
		m[2][0] = 0; m[2][1] = sin(phi);	m[2][2] = cos(phi);	 m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0;			m[3][2] = 0;		 m[3][3] = 1;
	}

	void InitAsRotateY(double phi) {
		m[0][0] = cos(phi);   m[0][1] = 0;			m[0][2] = sin(phi);	m[0][3] = 0;
		m[1][0] = 0;		  m[1][1] = 1;		   	m[1][2] = 0;		m[1][3] = 0;
		m[2][0] = -sin(phi);  m[2][1] = 0;			m[2][2] = cos(phi); m[2][3] = 0;
		m[3][0] = 0;		  m[3][1] = 0;			m[3][2] = 0;		m[3][3] = 1;
	}

	
	void InitAsRotateZ(double phi) {
		m[0][0] = cos(phi);   m[0][1] = -sin(phi);	m[0][2] = 0; m[0][3] = 0;
		m[1][0] = sin(phi);	  m[1][1] = cos(phi);	m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0;		  m[2][1] = 0;			m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0;		  m[3][1] = 0;			m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsTranslate(double x, double y, double z) {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = x;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = y;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = z;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
	}

	void InitAsOrthographic(double right, double top, double far, double near) {
		m[0][0] = 1/right;	m[0][1] = 0;	 m[0][2] = 0;			  m[0][3] = 0;
		m[1][0] = 0;		m[1][1] = 1/top; m[1][2] = 0;			  m[1][3] = 0;
		m[2][0] = 0;		m[2][1] = 0;	 m[2][2] = -2/(far-near); m[2][3] = - (far + near) / (far - near);
		m[3][0] = 0;		m[3][1] = 0;	 m[3][2] = 0;			  m[3][3] = 1;
	}

	void InitAsPerspective(double right, double top, double far, double near) {
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
		double nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		double nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		double nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];

		return DoublePoint{ nX , nY, nZ };
	}

	DoublePoint Projection(DoublePoint& p) {
		double nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		double nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		double nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];
		double nW = m[3][0] * p.x + m[3][1] * p.y + m[3][2] * p.z + m[3][3];

		return DoublePoint{ nX/nW , nY/nW, nZ/nW };
	}
	
	DoublePoint ProjectOrtho(DoublePoint& p, double viewWidth, double viewHeight) {
		double nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		double nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];

		double ratioX = SCREEN_WIDTH / viewWidth;
		double ratioY = SCREEN_HEIGHT / viewHeight;

		return DoublePoint{ nX * ratioX, nY * ratioY };
	}

	// Screen is assumed to be a rectangle
	double calcFocalFromFov(double screenSize, double fov) {
		double beta = (M_PI - fov) / 2;
		return screenSize * tan(beta) / 2;
	}

	DoublePoint ProjectPerspective(DoublePoint& p, double viewWidth, double viewHeight, double fov) {
		double nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		double nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];
		double nZ = m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3];

		double F = calcFocalFromFov(viewWidth, fov);
		double focalMult = F / (F+nZ);
		
		double xPerspective = nX * focalMult;
		double yPerspective = nY * focalMult;

		double ratioX = SCREEN_WIDTH / viewWidth;
		double ratioY = SCREEN_HEIGHT / viewHeight;

		return DoublePoint{ xPerspective * ratioX, yPerspective * ratioY };
	}

};

struct Shape {
	std::vector<DoublePoint> points;
};

double degToRad(double a) {
	return a * M_PI / 180;
}

void shapeToProjectedVertices(M44& toApply, Shape& shape, std::vector<SDL_Vertex>& outVertices) {
	float screenOffsetX = SCREEN_WIDTH / 2;
	float screenOffsetY = SCREEN_HEIGHT / 2;

	for (auto ptr = shape.points.begin(); ptr < shape.points.end(); ++ptr) {
		DoublePoint projected = toApply.ProjectPerspective(*ptr, 400, 300, degToRad(FOV));

		SDL_Vertex each;
		each.position.x = (float)projected.x + screenOffsetX;
		each.position.y = (float)projected.y + screenOffsetY;

		outVertices.push_back(each);
	}
}

void shapeToOrthoVertices(M44& toApply, Shape& shape, std::vector<SDL_Vertex>& outVertices) {
	float screenOffsetX = SCREEN_WIDTH / 2;
	float screenOffsetY = SCREEN_HEIGHT / 2;
	
	for (auto ptr = shape.points.begin(); ptr < shape.points.end(); ++ptr) {
		DoublePoint projected = toApply.ProjectOrtho(*ptr, 400, 300);

		SDL_Vertex each;
		each.position.x = (float)projected.x + screenOffsetX;
		each.position.y = (float)projected.y + screenOffsetY;

		outVertices.push_back(each);
	}
}

void projectShapeToVertices(M44& toApply, M44& toProject, Shape& shape, std::vector<SDL_Vertex>& outVertices) {
	SDL_Color colors[3] = {
		{255,0,0, 255}, {0,255,0, 255}, {0,0,255, 255}
	};
	
	int colorPtr = 0;
	for (auto ptr = shape.points.begin(); ptr < shape.points.end(); ++ptr) {
		DoublePoint transformed = toApply.ApplyOnPoint(*ptr);
		
		DoublePoint projected = toProject.Projection(transformed);
		SDL_Vertex each;
		each.position.x = (float)(1+projected.x) * (SCREEN_WIDTH / 2);
		each.position.y = (float)(1+projected.y) * (SCREEN_HEIGHT / 2);
		each.color = colors[colorPtr % 3];
		++colorPtr;

		outVertices.push_back(each);
	}
}

void renderShape(Shape& shape, M44& toApply, M44& projection) {
	std::vector<SDL_Vertex> vertices;
	projectShapeToVertices(toApply, projection, shape, vertices);

	int indices[6];
	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 3;

	indices[3] = 1;
	indices[4] = 2;
	indices[5] = 3;

	Naive_RenderGeometry(renderer, nullptr, vertices.data(), vertices.size(), indices, 6);
}

Shape faceFront = { { {-50,-50, 10}, {50,-50, 10}, {50,50, 10}, {-50,50, 10} } };
Shape faceMiddle = { { {-50,-50,0}, {50,-50, 0}, {50,50, 0}, {-50,50, 0} } };
Shape faceBack = { { {-50,-50, -10}, {50,-50, -10}, {50,50, -10}, {-50,50, -10} } };

int frame = 0;

void renderFrame() {
	if (SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE)) {
		cout << "SDL_SetRenderDrawColor: " << SDL_GetError() << endl;
	}

	if (SDL_RenderClear(renderer)) {
		cout << "SDL_RenderClear: " << SDL_GetError() << endl;
	}
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);

	M44 toApply;
	toApply.InitAsTranslate(0,0,-100);

	double angleZ = (frame / 400.0)* M_PI;
	M44 rotZ; rotZ.InitAsRotateZ(angleZ);

	double angleY = angleZ / 2;
	M44 rotY; rotY.InitAsRotateY(angleY);

	double angleX = angleZ / 3;
	M44 rotX; rotX.InitAsRotateY(angleX);

	toApply.Mult(rotX);
	toApply.Mult(rotY);
	toApply.Mult(rotZ);


	M44 ident;
	ident.InitAsIdentity();
	
	M44 projection;


	if (useOrtho) {
		projection.InitAsOrthographic(SCREEN_WIDTH/2, SCREEN_HEIGHT/2, 200, 1);
	}
	else {

		double aspectRatio = ((double)SCREEN_WIDTH)/SCREEN_HEIGHT;
		double front = 1;
		double tangent = tan(FOV / 2 * M_PI / 180);
		double right = front * tangent;
		double top = right / aspectRatio;

		projection.InitAsPerspective(right, top, 200, 1);
	}

	SDL_SetRenderDrawColor(renderer, 255, 64, 64, SDL_ALPHA_OPAQUE);
	renderShape(faceBack, toApply, projection);

	SDL_SetRenderDrawColor(renderer, 64, 255, 64, SDL_ALPHA_OPAQUE);
	renderShape(faceMiddle, toApply, projection);

	SDL_SetRenderDrawColor(renderer, 64, 64, 255, SDL_ALPHA_OPAQUE);
	renderShape(faceFront, toApply, projection);

	SDL_RenderPresent(renderer);
	if (!paused) {
		++frame;
		cout << "FRAME: " << frame << endl;
	}
}

void Naive_SortVertices(const SDL_Vertex* triangle, const int *indices, int* outIndices) {
	const SDL_Vertex* t = triangle;
	
	int a = *(indices + 0);
	int b = *(indices + 1);
	int c = *(indices + 2);

	int tmp;
	int list[3] = { a,b,c };
	if (t[a].position.y > t[b].position.y) { tmp = a; a = b; b = tmp; } // a<=>b
	if (t[a].position.y > t[c].position.y) { tmp = a; a = c; c = tmp; } // a<=>c
	if (t[b].position.y > t[c].position.y) { tmp = b; b = c; c = tmp; } // b<=>c

	if (t[a].position.y == t[b].position.y) { 
		// triangle with top horizontal line ; sort on X only top line and exit
		if (t[a].position.x > t[b].position.x) { tmp = a; a = b; b = tmp; } // a<=>b;
	}
	else {
		// single top vertex ; order other 2 on X;
		if (t[b].position.x > t[c].position.x) { tmp = b; b = c; c = tmp; } // b<=>c;
	}

	outIndices[0] = a;
	outIndices[1] = b;
	outIndices[2] = c;
}

float Naive_WalkTowards(float fromX, float fromY, float toX, float toY) {
	return (toX - fromX) / (toY - fromY);
}


void Naive_DrawTriangleLine(SDL_Renderer* renderer, BarycentricForTriangle &coords, float x1, float x2, float lineY) {
	// x1 and x2 are NOT expected that x1 < x2
	if (x1 > x2) {
		auto t = x1;
		x1 = x2;
		x2 = t;
	}

	if (lineY < 0 || lineY > SCREEN_HEIGHT) {
		return;
	}

	if (x1 < 0) {
		x1 = 0;
	}

	if (x2 > SCREEN_WIDTH) {
		x2 = SCREEN_WIDTH;
	}
	Uint8 r, g, b, a;

	BarycentricLambdas lambdas;
	SDL_FPoint point;
	point.y = lineY;
	for (float i = x1; i <= x2; ++i) {
		point.x = i;
		calcLambdaForPoint(lambdas, coords, point);
		Uint8 r = coords.a->color.r * lambdas.l1 + coords.b->color.r * lambdas.l2 + coords.c->color.r * lambdas.l3;
		Uint8 g = coords.a->color.g * lambdas.l1 + coords.b->color.g * lambdas.l2 + coords.c->color.g * lambdas.l3;
		Uint8 b = coords.a->color.b * lambdas.l1 + coords.b->color.b * lambdas.l2 + coords.c->color.b * lambdas.l3;
		SDL_SetRenderDrawColor(renderer, r, g, b, SDL_ALPHA_OPAQUE);
		
		SDL_RenderDrawPoint(renderer, i, lineY);
	}

}



void Naive_FillVertices(SDL_Renderer* renderer, const SDL_Vertex* vertices, const int* indices) {
	// TOTALLY NOT OPTIMIZED - has 2 loops while 2nd is doing partially what first does, but works; also it does way too much calculations; and uses too many variables; 
	// good subject to optimize, but should work fine
	int sortedIndices[3];

	float offLeft;
	float offRight;

	Naive_SortVertices(vertices, indices, sortedIndices);

	const SDL_Vertex* rawA = vertices + *(indices + 0);
	const SDL_Vertex* rawB = vertices + *(indices + 1);
	const SDL_Vertex* rawC = vertices + *(indices + 2);

	const SDL_Vertex* a = vertices + *(sortedIndices + 0);
	const SDL_Vertex* b = vertices + *(sortedIndices + 1);
	const SDL_Vertex* c = vertices + *(sortedIndices + 2);

	const SDL_FPoint* left;
	const SDL_FPoint* right;
	const SDL_FPoint* middle;
	
	BarycentricForTriangle coords;
	BarycentricLambdas lambdas;

	setupBarycentricForTriangle(coords, rawA, rawB, rawC);
	

	int lines;
	if (a->position.y == b->position.y) { // skip top triangle, setup walkers towards bottom point
		lines = c->position.y - a->position.y;

		left = &a->position;
		right = &b->position;
		middle = &c->position;
		
		offLeft = Naive_WalkTowards(left->x, left->y, middle->x, middle->y);
		offRight = Naive_WalkTowards(right->x, right->y, middle->x, middle->y);
		
		// check for reaching bottom line
		
		for (int i = 0; i < lines; ++i) {
			Naive_DrawTriangleLine(renderer, coords, left->x + i*offLeft, right->x + i * offRight, left->y + i);
		}
	}
	else {	// has top triangle, with A on top; and B/C (on left and right, but unknown which is higher)

		middle = &a->position;
		left = &b->position;
		right = &c->position;

		if (left->y < right->y) {
			lines = left->y - middle->y;
		}
		else {
			lines = right->y - middle->y;
		}
		
		offLeft = Naive_WalkTowards(middle->x, middle->y, left->x, left->y);
		offRight = Naive_WalkTowards(middle->x, middle->y, right->x, right->y);

		// draw towards horizontal line
		for (int i = 0; i < lines+1; ++i) {
			double leftX = round(middle->x + i * offLeft);
			double rightX = round(middle->x + i * offRight);
			int lineY = middle->y + i;

			Naive_DrawTriangleLine(renderer, coords, leftX, rightX, lineY);
		}

		if (left->y < right->y) {
			offLeft = Naive_WalkTowards(left->x, left->y, right->x, right->y);

			int nextLines = right->y - left->y;

			for (int i = 0; i < nextLines+1; ++i) {
				double leftX = left->x + i * offLeft;
				double rightX = middle->x + (lines+i) * offRight;
				double lineY = left->y + i;
				
				Naive_DrawTriangleLine(renderer, coords, leftX, rightX, lineY);
			}
		}
		else if (left->y > right->y) {
			offRight = Naive_WalkTowards(right->x, right->y, left->x, left->y);
			int nextLines = left->y - right->y;

			for (int i = 0; i < nextLines + 1; ++i) {
				double leftX = middle->x + (lines + i) * offLeft;
				double rightX = right->x + i * offRight;
				double lineY = right->y + i;

				Naive_DrawTriangleLine(renderer, coords, leftX, rightX, lineY);
			}
		}
	}
}

void Naive_RenderGeometry(SDL_Renderer* renderer, SDL_Texture* , const SDL_Vertex* vertices, int num_vertices, const int* indices, int num_indices) {
	// initially make a simple fill operation

	bool drawFill = true;
	bool drawWireframe = false;

	if (drawFill) {
		Naive_FillVertices(renderer, vertices, indices + 3);
		Naive_FillVertices(renderer, vertices, indices);
	}

	if (drawWireframe) {
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
		for (int t = 0; t < num_indices; t += 3) {
			for (int i = 0; i < 3; ++i) {
				int idxFrom = indices[i + t];
				int idxTo = indices[(i + 1) % 3 + t];

				const SDL_Vertex* fromV = vertices + idxFrom;
				const SDL_Vertex* toV = vertices + idxTo;

				SDL_RenderDrawLine(renderer, (int)fromV->position.x, (int)fromV->position.y, (int)toV->position.x, (int)toV->position.y);
			}
		}
	}
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


	int registeredTimer = SDL_AddTimer(25, tickFrame, nullptr);
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

	const int tx = 100;
	const int ty = 100;

	auto texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STATIC, tx, ty);
	if (nullptr == renderer) {
		cout << "Texture failed: " << SDL_GetError() << endl;
		return 1;
	}
	
	unsigned char pixels[tx * ty * 4] = {};
	int cnt = 0;
	for (int y=0;y<ty;y++) {
		for (int x = 0; x < tx; ++x) {
			int xy = 4 * (y * tx + x);
			
			pixels[xy + 0] = (cnt % 3) == 0 ? 255 : 0;
			pixels[xy + 1] = (cnt % 3) == 1 ? 255 : 0;
			pixels[xy + 2] = (cnt % 3) == 2 ? 255 : 0;
			pixels[xy + 3] = 255;
			++cnt;
		}
	}


	if (SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE)) {
		cout << "SDL_SetRenderDrawColor: " << SDL_GetError() << endl;
	}

	if (SDL_RenderClear(renderer)) {
		cout << "SDL_RenderClear: " << SDL_GetError() << endl;
	}
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
	SDL_UpdateTexture(texture, nullptr, pixels, tx * 4);
	
	SDL_Vertex vertices[6];
	float aX = 50;
	float aY = 100;

	float w = 200;
	float h = 200;

	vertices[0].color = { 255,255,255,255 };
	vertices[0].position = { aX,aY };
	vertices[0].tex_coord = { 0.0, 0.0 };
	
	vertices[1].color = { 255,255,255,255 };
	vertices[1].position = { aX+w,aY};
	vertices[1].tex_coord = { 1.0, 0.0 };
	
	vertices[2].color = { 255,255,255,255 };
	vertices[2].position = { aX,aY+h+100};
	vertices[2].tex_coord = { 0.0, 1.0 };

	vertices[3].color = { 255,255,255,255 };
	vertices[3].position = { aX+w,aY+h};
	vertices[3].tex_coord = { 1.0, 1.0 };

	int indices[6] = { 0,1,2, 1,2,3 };

	int renderStatus = SDL_RenderGeometry(renderer,
		texture,
		vertices, 4,
		indices, 6);

	for (int i = 0; i < 4; ++i) {
		vertices[i].position.x += 350;
	}
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
	Naive_RenderGeometry(renderer,
		texture,
		vertices, 4,
		indices, 6);
	

	cout << "Rendered: " << renderStatus << SDL_GetError() << endl;

	
	renderFrame();
	SDL_RenderPresent(renderer);

	BarycentricForTriangle coords;
	BarycentricLambdas lambdas;

	SDL_Vertex a;
	SDL_Vertex b;
	SDL_Vertex c;

	a.position = { 10,0 };
	b.position = { 0, 10 };
	c.position = { 20,10 };

	setupBarycentricForTriangle(coords, &a, &b, &c);
	
	SDL_FPoint point = { 10,5 };
	calcLambdaForPoint(lambdas, coords, point);
	
	SDL_FPoint outPoint;
	calcXYForLambda(outPoint, lambdas, coords);
	
	cout << "CALC:" << endl;
	cout << lambdas.l1 << endl;
	cout << lambdas.l3 << endl;
	cout << lambdas.l2 << endl;

	cout << "CALC:" << endl;
	cout << outPoint.x << endl;
	cout << outPoint.y << endl;
	
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

