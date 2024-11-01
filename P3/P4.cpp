#include <iostream>
#include <cmath>
#include <SDL.h>
#include <vector>
#include <cstdlib>

using namespace std;

double SCREEN_WIDTH = 800;
double SCREEN_HEIGHT = 600;

Uint32 globalCustomEventId = 0;
 
bool render = true;
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

struct DoublePoint {
	double x, y, z;
};

struct M44 {
	double m[4][4];

	void InitAsIdentity() {
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
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
		m[3][0] = 1; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
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
	
	
	DoublePoint ProjectOrtho(DoublePoint& p) {
		double nX = m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3];
		double nY = m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3];

		return DoublePoint{ nX, nY };
	}

};

struct Shape {
	std::vector<DoublePoint> points;
};


void renderShapeOrtho(M44& toApply, Shape& shape) {
	for (int i = 0; i < shape.points.size(); i++) {
		DoublePoint from = toApply.ProjectOrtho(shape.points[i]);
		DoublePoint to = toApply.ProjectOrtho(shape.points[(i + 1) % shape.points.size()]);

		SDL_RenderDrawLine(renderer, (int)floor(from.x), (int)floor(from.y), (int)floor(to.x), (int)floor(to.y));
	}
}

Shape faceFront = { { {-80,-80, 50}, {80,-80, 50}, {80,80, 50}, {-80,80, 50} } };
Shape faceMiddle = { { {-90,-90, 0}, {90,-90, 0}, {90,90, 0}, {-90,90, 0} } };
Shape faceBack = { { {-100,-100, -50}, {100,-100, -50}, {100,100, -50}, {-100,100, -50} } };


void renderShapeOrthoWithOffset(Shape& s, M44& base, double offX, double offY) {
	M44 flipOffset;
	flipOffset.InitAsTranslate(offX, offY, 0);
	flipOffset.Mult(base);
	renderShapeOrtho(flipOffset, s);
}

int frame = 0;

void renderFrame() {
	return;
	if (SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE)) {
		cout << "SDL_SetRenderDrawColor: " << SDL_GetError() << endl;
	}

	if (SDL_RenderClear(renderer)) {
		cout << "SDL_RenderClear: " << SDL_GetError() << endl;
	}
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);

	M44 base;

	double angleZ = (frame / 400.0)* M_PI;
	base.InitAsRotateZ(angleZ);

	double angleY = angleZ / 2;
	M44 rotY; rotY.InitAsRotateY(angleY);

	double angleX = angleZ / 3;
	M44 rotX; rotX.InitAsRotateY(angleX);

	base.Mult(rotX);

	renderShapeOrthoWithOffset(faceFront, base, 400, 300);
	renderShapeOrthoWithOffset(faceMiddle, base, 400, 300);
	renderShapeOrthoWithOffset(faceBack, base, 400, 300);

	SDL_RenderPresent(renderer);
	++frame;
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

void Naive_FillVertices(SDL_Renderer* renderer, const SDL_Vertex* vertices, const int* indices) {
	// TOTALLY NOT OPTIMIZED - has 2 loops while 2nd is doing partially what first does, but works
	int sortedIndices[3];
	float leftX;
	float rightX;

	float offLeft;
	float offRight;

	Naive_SortVertices(vertices, indices, sortedIndices);

	const SDL_Vertex* a = vertices + *(sortedIndices + 0);
	const SDL_Vertex* b = vertices + *(sortedIndices + 1);
	const SDL_Vertex* c = vertices + *(sortedIndices + 2);

	bool fromSingle = true;

	int cursor;
	int bottom;

	if (a->position.y == b->position.y) { // skip top triangle, setup walkers towards bottom point
		cursor = (int)a->position.y;
		bottom = (int)c->position.y;

		leftX = a->position.x;
		rightX = b->position.x;

		offLeft = Naive_WalkTowards(leftX, a->position.y, c->position.x, c->position.y);
		offRight = Naive_WalkTowards(rightX, b->position.y, c->position.x, c->position.y);
		
		// check for reaching bottom line
		for (cursor; cursor <= bottom; ++cursor) {
			SDL_RenderDrawLine(renderer, (int)leftX, cursor, (int)rightX, cursor);
			leftX += offLeft;
			rightX += offRight;
		}
	}
	else {	// has top triangle, with A on top; and B/C (on left and right, but unknown which is higher)
		cursor = (int)a->position.y;
		int sndBottom;
		bool leftIsHigher;

		if (b->position.y < c->position.y) { // draw until reaching first point
			bottom = (int)b->position.y;
			sndBottom = (int)c->position.y;
			leftIsHigher = true;
		}
		else {
			bottom = (int)c->position.y;
			sndBottom = (int)b->position.y;
			leftIsHigher = false;
		}

		leftX = a->position.x;
		rightX = leftX;

		offLeft = Naive_WalkTowards(a->position.x, a->position.y, b->position.x, b->position.y);
		offRight = Naive_WalkTowards(a->position.x, a->position.y, c->position.x, c->position.y);

		// draw towards horizontal line
		for (cursor; cursor <= bottom; ++cursor) {
			SDL_RenderDrawLine(renderer, (int)leftX, cursor, (int)rightX, cursor);
			leftX += offLeft;
			rightX += offRight;
		}

		if (bottom == sndBottom) { // nothing more to draw
			return;
		}
		
		// point left or right cursor towards last point
		if (leftIsHigher) {
			offLeft = Naive_WalkTowards(b->position.x, b->position.y, c->position.x, c->position.y);
		}
		else {
			offRight = Naive_WalkTowards(c->position.x, c->position.y, b->position.x, b->position.y);
		}

		for (cursor; cursor <= sndBottom; ++cursor) {
			SDL_RenderDrawLine(renderer, (int)leftX, cursor, (int)rightX, cursor);
			leftX += offLeft;
			rightX += offRight;
		}

	}

}

void Naive_RenderGeometry(SDL_Renderer* renderer, SDL_Texture* texture, const SDL_Vertex* vertices, int num_vertices, const int* indices, int num_indices) {
	// initially make a simple fill operation

	bool drawFill = true;
	bool drawWireframe = false;

	if (drawFill) {
		Naive_FillVertices(renderer, vertices, indices + 3);
		Naive_FillVertices(renderer, vertices, indices);
		
	}

	if (drawWireframe) {
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
	
	SDL_Event e;
	for (;;) {
		if (SDL_WaitEvent(&e)) {
			if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
				SDL_KeyboardEvent* keyboardEvent = (SDL_KeyboardEvent*)(&e);
				auto key = keyboardEvent->keysym.scancode;
				if (key == SDL_SCANCODE_ESCAPE) {
					break;
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

