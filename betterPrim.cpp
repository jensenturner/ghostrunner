#include <linux/fb.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include "cat.h"

#define argb(alpha, red, green, blue) ((((alpha << 24) | red << 16) | green << 8) | blue)
#define afromargb(argb) ((argb & 0xff000000)>>24)
#define rfromargb(argb) ((argb & 0x00ff0000)>>16)
#define gfromargb(argb) ((argb & 0x0000ff00)>>8)
#define bfromargb(argb) (argb & 0x000000ff)

//various clipping stuff + relative unit stuff (glasses dimensions, pixels / cm, etc.)
//measured in cm
//negative Z because right-handed coordinate system, so -z axis is 'forwards'
const float minDepth = -40.0f;
const float maxDepth = -8000.0f;
//measured in pixels
//screen dimension constants
int SCREEN_WIDTH = 1280;
int SCREEN_HEIGHT = 720;
const float headDistance = 100.0f;
//measurement constants
const float pixpercm = 1280.0f/150.0f;
const float cmperpix = 150.0f/1280.0f;

//all shading uses a lazy system where the radiance
//is cut off at this value and scaled
const float maxRadiance = 100.0f;

//IMPORTANT: don't rely on these buffers being public. might become local when a second eye is introduced
uint32_t* gPixels = new uint32_t[SCREEN_WIDTH*SCREEN_HEIGHT];
float* zBuffer = new float[SCREEN_WIDTH*SCREEN_HEIGHT];

//math structures
struct mat4 {
	float stuff[4][4];
};

struct vector3 {
	float x;
	float y;
	float z;
};

struct vector4 {
	float x;
	float y;
	float z;
	float w;
};

struct vertice {
	struct vector3 pos;
	struct vector3 normal;
	struct vector3 eyeV;
	struct vector4 color;
};

//utility functions (aka. functions that have functions)
float fMax(float a, float b)
{
	if (a > b)
		return a;
	else
		return b;
}

float fMin(float a, float b)
{
	if (a < b)
		return a;
	else
		return b;
}

float vMax(float a, float b, float c)
{
	if (a > b)
	{
		if (a > c)
			return a;
		else
			return c;
	}
	else
	{
		if (b > c)
			return b;
		else
			return c;
	}
}

float vMin(float a, float b, float c)
{
	if (a < b)
	{
		if (a < c)
			return a;
		else
			return c;
	}
	else
	{
		if (b < c)
			return b;
		else
			return c;
	}
}

struct vector3 addV(struct vector3 a, struct vector3 b)
{
	struct vector3 temp = {a.x + b.x, a.y + b.y, a.z + b.z};
	return(temp);
}

struct vector4 addV(struct vector4 a, struct vector4 b)
{
	struct vector4 temp = {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
	return(temp);
}

struct vector3 subV(struct vector3 a, struct vector3 b)
{
	struct vector3 temp = {a.x - b.x, a.y - b.y, a.z - b.z};
	return(temp);
}

struct vector3 multV(struct vector3 a, float b)
{
	struct vector3 temp = { a.x * b, a.y * b, a.z * b };
	return(temp);
}

struct vector4 multV(struct vector4 a, float b)
{
	struct vector4 temp = {a.x * b, a.y * b, a.z * b, a.w * b};
	return(temp);
}

float mag(struct vector3 a)
{
	return(sqrt((a.x*a.x) + (a.y*a.y) + (a.z*a.z)));
}

float dot(struct vector3 a, struct vector3 b)
{
	return (a.x*b.x + a.y*b.y + a.z*b.z);
}

float distance(struct vector3 p1, struct vector3 p2)
{
	return(sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) + (p2.z - p1.z)*(p2.z - p1.z)));
}

//weight of 0 = a, weight of 1 = b
float lerp(float weight, float a, float b)
{
	return(a*(1 - weight) + b*weight);
}

//weight of 0 = a, weight of 1 = b
struct vector3 lerp(float weight, struct vector3 a, struct vector3 b)
{
	struct vector3 temp = {a.x * (1-weight) + b.x * weight, a.y * (1-weight) + b.y * weight, a.z * (1-weight) + b.z * weight};
	return(temp);
}

//these should be a lot faster than using a bunch of vector functions or something
struct vector3 baryV3(float* bary, struct vector3 v1, struct vector3 v2, struct vector3 v3)
{
	struct vector3 temp = {bary[0] * v1.x + bary[1] * v2.x + bary[2] * v3.x, bary[0] * v1.y + bary[1] * v2.y + bary[2] * v3.y, bary[0] * v1.z + bary[1] * v2.z + bary[2] * v3.z};
	return(temp);
}

struct vector4 baryV4(float* bary, struct vector4 v1, struct vector4 v2, struct vector4 v3)
{
	struct vector4 temp = {bary[0] * v1.x + bary[1] * v2.x + bary[2] * v3.x, bary[0] * v1.y + bary[1] * v2.y + bary[2] * v3.y, bary[0] * v1.z + bary[1] * v2.z + bary[2] * v3.z, bary[0]*v1.w + bary[1]*v2.w + bary[2]*v3.w};
	return(temp);
}

//makes a vector into a unit vector (not into a normal vector, that's cross product)
struct vector3 normalize(struct vector3 in)
{
	float temp = mag(in);
	struct vector3 tempV = {in.x / temp, in.y / temp, in.z / temp};
	return(tempV);
}

//first index is x, second index is y
struct vector4 matMult(struct mat4 inMat, struct vector4 inVec)
{
	struct vector4 tempVec;
	tempVec.x = inMat.stuff[0][0] * inVec.x + inMat.stuff[1][0] * inVec.y + inMat.stuff[2][0] * inVec.z + inMat.stuff[3][0] * inVec.w;
	tempVec.y = inMat.stuff[0][1] * inVec.x + inMat.stuff[1][1] * inVec.y + inMat.stuff[2][1] * inVec.z + inMat.stuff[3][1] * inVec.w;
	tempVec.z = inMat.stuff[0][2] * inVec.x + inMat.stuff[1][2] * inVec.y + inMat.stuff[2][2] * inVec.z + inMat.stuff[3][2] * inVec.w;
	tempVec.w = inMat.stuff[0][3] * inVec.x + inMat.stuff[1][3] * inVec.y + inMat.stuff[2][3] * inVec.z + inMat.stuff[3][3] * inVec.w;
	return(tempVec);
}

struct mat4 matMult(struct mat4 left, struct mat4 right)
{
	struct mat4 tempMat;
	for (int x = 0; x < 4; x++)
		for (int y = 0; y < 4; y++)
		{
			tempMat.stuff[x][y] = 0.0f;
			for (int i = 0; i < 4; i++)
				tempMat.stuff[x][y] += left.stuff[i][y] * right.stuff[x][i];
		}
	return(tempMat);
}

struct vector3 cross(struct vector3 a, struct vector3 b)
{
	struct vector3 temp = {a.y*b.z - a.z*b.y, -a.x*b.z + a.z*b.x, a.x*b.y - a.y*b.x};
	return(temp);
}

struct vector4 pWise(struct vector4 a, struct vector4 b)
{
	struct vector4 temp = {a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w};
	return(temp);
}

//IMPORTANT: now using column vectors
// mat A, then B, then C applied to vec v means mathematically
//C*(B*(A*v)) = (C*B*A)*v

//returns a 3d translation matrix
struct mat4 transmat(float xOff, float yOff, float zOff)
{
	struct mat4 temp = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {xOff, yOff, zOff, 1}}};
	return temp;
}

//returns a 3d scaling matrix
struct mat4 scalmat(float xScale, float yScale, float zScale)
{
	struct mat4 temp = {{{xScale, 0, 0, 0}, {0, yScale, 0, 0}, {0, 0, zScale, 0}, {0, 0, 0, 1}}};
	return temp;
}

//returns a 3d rotation matrix in x axis around 0,0,0
//angles are in radians
struct mat4 rotxmat(float angle)
{
	float tempCos = cosf(angle);
	float tempSin = sinf(angle);
	struct mat4 temp = {{{1, 0, 0, 0}, {0, tempCos, -tempSin, 0}, {0, tempSin, tempCos, 0}, {0, 0, 0, 1}}};
	return temp;
}

//returns a 3d rotation matrix in y axis around 0,0,0
//angles are in radians
struct mat4 rotymat(float angle)
{
	float tempCos = cosf(angle);
	float tempSin = sinf(angle);
	struct mat4 temp = {{{tempCos, 0, tempSin, 0}, {0, 1, 0, 0}, {-tempSin, 0, tempCos, 0}, {0, 0, 0, 1}}};
	return temp;
}

//returns a 3d rotation matrix in z axis around 0,0,0
//angles are in radians
struct mat4 rotzmat(float angle)
{
	float tempCos = cosf(angle);
	float tempSin = sinf(angle);
	struct mat4 temp = {{{tempCos, -tempSin, 0, 0}, {tempSin, tempCos, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
	return temp;
}


//pipline things (our pipeline only lets one thing through at a time)

//general shader (does not add shadows; todo: make a different pipeline element that throws shade)
struct vector4 shade(struct vector3 eyeV, struct vector3 normal, struct vector4 diffC, struct vector4 specC, float smooth, struct vector3 lightN, struct vector4 lightC)
{
	struct vector3 h = normalize(addV(eyeV, lightN));
	float cosAngleA = fMax(0, dot(normal, h));
	float cosAngleB = fMax(0, dot(normal, lightN));
	struct vector4 color = pWise(addV(diffC, multV(specC, powf(cosAngleA, smooth))), (multV(lightC, cosAngleB)));
	//rescale the brightness
	color.x = color.x / maxRadiance;
	if (color.x > 255)
		color.x = 255;
	color.y = color.y / maxRadiance;
	if (color.y > 255)
		color.y = 255;
	color.z = color.z / maxRadiance;
	if (color.z > 255)
		color.z = 255;
	color.w = color.w / maxRadiance;
	if (color.w > 255)
		color.w = 255;
	return(color);
}

//checks if the triangle is obviously outside the viewing frustum (false == should be culled, true == shouldn't
bool cullTri(struct vector3 a, struct vector3 b, struct vector3 c, float bounds[4])
{
	if (a.z > minDepth && b.z > minDepth && c.z > minDepth)
		return false;
	if (a.z < maxDepth && b.z < maxDepth && c.z < maxDepth)
		return false;
	if (a.x > lerp(a.z / maxDepth, 0, bounds[0] * maxDepth / minDepth) &&
		b.x > lerp(b.z / maxDepth, 0, bounds[0] * maxDepth / minDepth) &&
		c.x > lerp(c.z / maxDepth, 0, bounds[0] * maxDepth / minDepth))
		return false;
	if (a.x < lerp(a.z / maxDepth, 0, bounds[1] * maxDepth / minDepth) &&
		b.x < lerp(b.z / maxDepth, 0, bounds[1] * maxDepth / minDepth) &&
		c.x < lerp(c.z / maxDepth, 0, bounds[1] * maxDepth / minDepth))
		return false;
	if (a.y > lerp(a.z / maxDepth, 0, bounds[2] * maxDepth / minDepth) &&
		b.y > lerp(b.z / maxDepth, 0, bounds[2] * maxDepth / minDepth) &&
		c.y > lerp(c.z / maxDepth, 0, bounds[2] * maxDepth / minDepth))
		return false;
	if (a.y < lerp(a.z / maxDepth, 0, bounds[3] * maxDepth / minDepth) &&
		b.y < lerp(b.z / maxDepth, 0, bounds[3] * maxDepth / minDepth) &&
		c.y < lerp(c.z / maxDepth, 0, bounds[3] * maxDepth / minDepth))
		return false;
	return true;
}

//perspective transformation stuff

//very nice perspective transformation. uses -z axis forwards convention and returrns everything
//mapped to a left-handed 'unit' cube (zfar = 1, znear = -1, top = 1, left = -1, etc. etc.)
//the old z value is transferred to the w component, but just putting this in the project function
//with any old vector input (provided it has w = 1) will do that for you (don't use without dividing by w)
struct mat4 asymFrust(struct vector3 Sll, struct vector3 Slr, struct vector3 Sul, struct vector3 Epos, float nearP, float farP, float* boundInfo)
{
	//make the direction vectors for the screen orientation
	struct vector3 right = normalize(subV(Slr, Sll));
	struct vector3 up = normalize(subV(Sul, Sll));
	struct vector3 out = normalize(cross(right, up));

	struct vector3 EtoSll = subV(Sll, Epos);
	struct vector3 EtoSlr = subV(Slr, Epos);
	struct vector3 EtoSul = subV(Sul, Epos);
	
	//no minus sign on this because -z axis is forwards, so this being negative will cancel
	//with the negative near plane, making the right side on the right and left on the left
	float d = dot(EtoSll, out);

	float l = dot(right, EtoSll) * nearP / d;
	float r = dot(right, EtoSlr) * nearP / d;
	float t = dot(up, EtoSul) * nearP / d;
	float b = dot(up, EtoSll) * nearP / d;

	//copy the near plane boundry info to the pointer
	boundInfo[0] = r;
	boundInfo[1] = l;
	boundInfo[2] = t;
	boundInfo[3] = b;

	//column vectors, -z axis forwards
	struct mat4 pMat = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
	pMat.stuff[0][0] = 2 * nearP / (r - l);
	pMat.stuff[1][1] = 2 * nearP / (t - b);
	pMat.stuff[2][0] = -1 * (r + l) / (r - l);
	pMat.stuff[2][1] = -1 * (t + b) / (t - b);
	pMat.stuff[2][2] = 1 * (farP + nearP) / (farP - nearP);
	pMat.stuff[2][3] = 1;
	pMat.stuff[3][2] = -1 * (2 * farP*nearP) / (farP - nearP);
	//remember, old z value is transferred to w. make sure not to add 1 for no reason
	pMat.stuff[3][3] = 0;
	
	//now make the fancy rotation matrix
	struct mat4 M = {{{right.x, right.y, right.z, 0}, {up.x, up.y, up.z, 0}, {out.x, out.y, out.z, 0}, {0, 0, 0, 1}}};

	//now do the fancy translation matrix
	struct mat4 T = transmat(-Epos.x, -Epos.y, -Epos.z);

	//get the order right
	return matMult(T, matMult(M, pMat));
}

//just applies a projection matrix and does division by z (only makes sense with points, w = 1)
struct vector3 project(struct vector3 in, struct mat4 mat)
{
	struct vector4 tempA = { in.x, in.y, in.z, 1 };
	tempA = matMult(mat, tempA);
	struct vector3 tempB;
	//catch for divide-by-zero
	if (tempA.w != 0.0f)
		tempB = {tempA.x / tempA.w, tempA.y / tempA.w, tempA.z / tempA.w};
	else
		tempB = {0, 0, -1};
	return(tempB);
}

//converts the viewing unit box to screen coordinates
//inz = 1 maps to max depth, inz = -1 maps to min depth
struct vector3 screenConvert(struct vector3 vIn)
{
	struct vector3 temp;
	//could use lerp, but this is slightly faster and not that hard to read
	temp.x = (vIn.x + 1) / 2 * (SCREEN_WIDTH - 1);
	temp.y = (-vIn.y + 1) / 2 * (SCREEN_HEIGHT - 1);
	temp.z = (vIn.z + 1) / 2 * (maxDepth - minDepth) + minDepth;
	return(temp);
}

//rasterizer things
//float positions for convenience and robustness (read: laziness)
//using constant screen resolutions should be fine, since both eye buffers will have same max x and y
void drawPixel(uint32_t* buffer, int x, int y, struct vector4 color)
{
	if (x >= 0 && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT)
		buffer[x + y*SCREEN_WIDTH] = argb((unsigned int)color.x, (unsigned int)color.y, (unsigned int)color.z, (unsigned int)color.w);
}

//barycentric weights for a triangle
void baryWeights(float x1, float x2, float x3, float y1, float y2, float y3, float pX, float pY, float* storage)
{
	storage[0] = ((y2 - y3)*(pX - x3) + (x3 - x2)*(pY - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3));
	storage[1] = ((y3 - y1)*(pX - x3) + (x1 - x3)*(pY - y3)) / ((y2 - y3)*(x1 - x3) + (x3 - x2)*(y1 - y3));
	storage[2] = 1 - storage[0] - storage[1];
}

void baryBoxTri(Uint32* buffer, float* zBuffer, struct vertice points[3])
{
	float tempz;
	struct vector4 tempColor;
	float boxBary[12];
	int boundMaxX = (int)fMin(ceilf(vMax(points[0].pos.x, points[1].pos.x, points[2].pos.x)), (float)(SCREEN_WIDTH - 1));
	int boundMaxY = (int)fMin(ceilf(vMax(points[0].pos.y, points[1].pos.y, points[2].pos.y)), (float)(SCREEN_HEIGHT - 1));
	//loop through a bunch of different boxes
	int boxSize = 4;
	for (int boxMinX = (int)fMax(floorf(vMin(points[0].pos.x, points[1].pos.x, points[2].pos.x)), 0.0f);
		boxMinX <= boundMaxX - (boxSize - 1); boxMinX += boxSize)
	{
		for (int boxMinY = (int)fMax(floorf(vMin(points[0].pos.y, points[1].pos.y, points[2].pos.y)), 0.0f);
			boxMinY <= boundMaxY - (boxSize - 1); boxMinY += boxSize)
		{
			int boxMaxX = min(boundMaxX, boxMinX + boxSize - 1);
			int boxMaxY = min(boundMaxY, boxMinY + boxSize - 1);
			baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)boxMinX, (float)boxMinY, boxBary);
			baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)boxMinX, (float)boxMaxY, &boxBary[3]);
			baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)boxMaxX, (float)boxMinY, &boxBary[6]);
			baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)boxMaxX, (float)boxMaxY, &boxBary[9]);
			if (boxBary[0] >= 0 && boxBary[1] >= 0 && boxBary[2] >= 0 && boxBary[3] >= 0 && boxBary[4] >= 0 && boxBary[5] >= 0 && boxBary[6] >= 0 && boxBary[7] >= 0 && boxBary[8] >= 0 && boxBary[9] >= 0 && boxBary[10] >= 0 && boxBary[11] >= 0)
			{
				for (int yoff = 0; yoff < (min(boxSize + boxMinY, boundMaxY) - boxMinY); yoff++)
				{
					for (int xoff = 0; xoff < (min(boxSize + boxMinX, boundMaxX) - boxMinX); xoff++)
					{
						int tempX = xoff + boxMinX;
						int tempY = yoff + boxMinY;
						float temp[3];
						baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)tempX, (float)tempY, temp);
						if (temp[0] >= 0 && temp[1] >= 0 && temp[2] >= 0)
						{
							tempz = points[0].pos.z*temp[0] + points[1].pos.z*temp[1] + points[2].pos.z*temp[2];
							if (zBuffer[tempX + tempY*SCREEN_WIDTH] < tempz && tempz <= minDepth)
							{
								zBuffer[tempX + tempY*SCREEN_WIDTH] = tempz;
								tempColor = baryV4(temp, points[0].color, points[1].color, points[2].color);
								drawPixel(buffer, tempX, tempY, tempColor);
							}
						}
					}
				}
			}
		}
	}
}

void baryTri(uint32_t* buffer, float* zBuffer, struct vertice points[3])
{
	float tempZ;
	struct vector4 tempColor;
	//loop through every pixel in the bounding box around the triangle (but clip the box on the screen bounds)
	for (int x = (int)fMax(floorf(vMin(points[0].pos.x, points[1].pos.x, points[2].pos.x)), 0.0f); x <= (int)fMin(ceilf(vMax(points[0].pos.x, points[1].pos.x, points[2].pos.x)), (float)SCREEN_WIDTH); x++)
	{
		for (int y = (int)fMax(floorf(vMin(points[0].pos.y, points[1].pos.y, points[2].pos.y)), 0.0f); y <= (int)fMin(ceilf(vMax(points[0].pos.y, points[1].pos.y, points[2].pos.y)), (float)SCREEN_HEIGHT); y++)
		{
			float temp[3];
			baryWeights(points[0].pos.x, points[1].pos.x, points[2].pos.x, points[0].pos.y, points[1].pos.y, points[2].pos.y, (float)x, (float)y, temp);
			if (temp[0] >= 0 && temp[1] >= 0 && temp[2] >= 0)
			{
				tempZ = points[0].pos.z*temp[0] + points[1].pos.z*temp[1] + points[2].pos.z*temp[2];
				if (zBuffer[x + y*SCREEN_WIDTH] < tempZ && tempZ <= minDepth)
				{
					zBuffer[x + y*SCREEN_WIDTH] = tempZ;
					//tempColor = shade(normalize(points[0].eyeV*temp.x + points[1].eyeV*temp.y + points[2].eyeV*temp.z), normalize(points[0].normal*temp.x + points[1].normal*temp.y + points[2].normal*temp.z), diffC, specC, smooth, lightN, lightC);
					tempColor = baryV4(temp, points[0].color, points[1].color, points[2].color);
					drawPixel(buffer, x, y, tempColor);
				}
			}
		}
	}
}

//BEWARE: sdl2 already has a function main, so argc and args are necessary to avoid linker errors
int main(int argc, char* args[])
{
	//figure out the screen dimensions
	struct fb_var_screeninfo tvinfo;
	int fb_fdt = open("/dev/fb0", O_RDWR);
	//Get variable screen information
	ioctl(fb_fdt, FBIOGET_VSCREENINFO, &tvinfo);
	SCREEN_WIDTH = tvinfo.xres;
	SCREEN_HEIGHT = tvinfo.yres;
	delete[] gPixels;
	delete[] zBuffer;
	gPixels = new uint32_t[SCREEN_WIDTH*SCREEN_HEIGHT];
	zBuffer = new float[SCREEN_WIDTH*SCREEN_HEIGHT];

	//now things seem to be all set up, so let's initialize some random things we probably won't use more than once or twice
	//perspective projection stuff
	//units don't matter, since everything will cancel out; just  needs to be consistent here (using pixels)
	float sWidth = ((float)SCREEN_WIDTH)*cmperpix;
	float sHeight = ((float)SCREEN_HEIGHT)*cmperpix;
	struct vector3 screenLL = {-sWidth/2, -sHeight/2, -headDistance};
	struct vector3 screenLR = {sWidth/2, -sHeight/2, -headDistance};
	struct vector3 screenUL = {-sWidth/2, sHeight/2, -headDistance};
	struct vector3 eyePos = {0, 0, 0};
	//asymFrust writes to this for view-space culling info (right, left, top, down)
	float nearBounds[4];
	struct mat4 proMat = asymFrust(screenLL, screenLR, screenUL, eyePos, minDepth, maxDepth, nearBounds);

	//cat data
	struct vector4 catDiff = {0, 255, 165, 0};
	struct vector4 catSpec = {0, 255, 165, 0};
	float catSmooth = 4.0f;
	struct vector4 catDC = multV(catDiff, 1.0f/3.1415f);
	struct vector4 catSC = multV(catSpec, ((catSmooth + 8.0f) / (8.0f*3.1415f)));

	//points back towards light source
	struct vector3 sunDir = {0, 0, 1};
	struct vector4 sunPower = {0, 5.0f*maxRadiance, 5.0f*maxRadiance, 5.0f*maxRadiance};

	struct vector3 camPos = {0, 172, 0};
	struct vector3 camRot = {0, 0, 0};
	struct vector3 modelPos = {0, 0, 0};
	struct vector3 modelRot = {0, 0, 0};
	float camSpeed = 745.0f;
	float camRotSpeed = 0.7f;
	float catSpeed = 300.0f;
	struct mat4 modelMat;
	struct mat4 eyeMat;

	uint32_t backgroundColor = argb(0u, 0u,0u,0u);
	bool quit = false;
	
	//load them or something
	FILE* fp = fopen("C:\\Users\\xande\\Desktop\\path.txt", "r");
	for (int k = 0; k < numPoints; k++)
	{
		fscanf(fp, "%lf, %lf", &(rawGPSlat[k]), &(rawGPSlon[k]));
		//printf("%f %f", rawGPSlat[k], rawGPSlon[k]);
	}
	fclose(fp);

	///numPoints is the max number of unique coordinates
	//the gps data will probably be a bit messed up, so realPointNum should be used instead later
	struct vector3 pathCoords[numPoints];
	float pathDistances[numPoints - 1];
	float totalDistance = 0;
	int realPointNum = 1;
	pathCoords[0] = { 0, 0, 0};
	for (int i = 1; i < numPoints; i++)
	{
		double tempZoff = (rawGPSlat[i] - rawGPSlat[i - 1])/100.0*3.1415*earthR / 180.0;
		double tempXoff = (rawGPSlon[i] - rawGPSlon[i - 1])*3.1415*earthR*cos((rawGPSlat[i] + rawGPSlat[i - 1]) * 3.1415 / 180.0 / 100)/180.0/10.0;
		if (tempZoff != 0.0 || tempXoff != 0.0)
		{
			pathCoords[realPointNum] = { pathCoords[realPointNum - 1].x + (float)tempXoff, pathCoords[realPointNum - 1].y, pathCoords[realPointNum - 1].z - (float)tempZoff };
			pathDistances[realPointNum - 1] = distance(pathCoords[realPointNum - 1], pathCoords[realPointNum]);
			totalDistance += pathDistances[realPointNum - 1];
			realPointNum++;
		}
	}

	//now figure out the average direction of the cat at each point
	struct vector3 pathDirs[numPoints];
	pathDirs[0] = normalize(subV(pathCoords[1], pathCoords[0]));
	pathDirs[realPointNum - 1] = normalize(subV(pathCoords[realPointNum - 1], pathCoords[realPointNum - 2]));
	for (int i = 1; i < (realPointNum - 1); i++)
	{
		//if the previous and next point are the same, chose the direction to be perpendicular 
		if (pathCoords[i + 1].x == pathCoords[i - 1].x && pathCoords[i + 1].z == pathCoords[i - 1].z)
			pathDirs[i] = { -pathDirs[i - 1].z, pathDirs[i - 1].y, -pathDirs[i - 1].x };
		else
			pathDirs[i] = normalize(subV(pathCoords[i+1], pathCoords[i-1]));
	}

	float catDistance = 0.0f;
	bool smoothPath = true;

	//counters for frame rate and delta time (windows-specific)
	struct timeval StartingTime, EndingTime;
	float deltaT;
	gettimeofday(&StartingTime, NULL);
	//getting stuck in this is fine, since it's a stable time loop (steins gate is the best!)
	while (!quit)
	{
		//find delta time
		gettimeofday(&EndingTime, NULL);
		deltaT = EndingTime.tv_sec - StartingTime.tv_sec;
		//printf("Framerate is %f fps\n", 1 / deltaT);
		gettimeofday(&StartingTime, NULL);

		//application stuff (positions, rotations, etc.)
		catDistance += catSpeed*deltaT;
		if (catDistance > totalDistance)
			modelPos = pathCoords[realPointNum-1];
		else
		{
			float tempDistance = catDistance;
			int sec;
			for (sec = 0; sec < realPointNum - 1; sec++)
			{
				if (tempDistance < pathDistances[sec])
					break;
				else
				{
					tempDistance -= pathDistances[sec];
				}
			}
			struct vector3 tempCatDir;
			if (smoothPath)
			{
				tempCatDir = normalize(lerp(tempDistance / pathDistances[sec], pathDirs[sec], pathDirs[sec + 1]));
				modelPos = addV(modelPos, multV(tempCatDir, catSpeed*deltaT));
			}
			else
			{
				tempCatDir = normalize(subV(pathCoords[sec + 1], pathCoords[sec]));
				modelPos = lerp(tempDistance / pathDistances[sec], pathCoords[sec], pathCoords[sec + 1]);
			}
			const struct vector3 tempOffV = { 1, 0, 0 };
			modelRot.y = (tempCatDir.z < 0 ? -acos(dot(tempOffV, tempCatDir)) : acos(dot(tempOffV, tempCatDir)));
		}


		//creation necessitates destruction
		for (int x = 0; x < SCREEN_WIDTH; x++)
		{
			for (int y = 0; y < SCREEN_HEIGHT; y++)
			{
				gPixels[x + y*SCREEN_WIDTH] = backgroundColor;
				zBuffer[x + y*SCREEN_WIDTH] = maxDepth;
			}
		}

		//now we render the scene
		//model + camera matrixes -> vertex shader -> projection -> screen coordinates -> rasterizer
		modelMat = matMult(transmat(modelPos.x, modelPos.y, modelPos.z), matMult(rotymat(modelRot.y), matMult(rotxmat(modelRot.x), scalmat(0.5f, 0.5f, 0.5f))));
		eyeMat = matMult(rotymat(-camRot.y), matMult(rotxmat(-camRot.x), transmat(-camPos.x, -camPos.y, -camPos.z)));
		struct mat4 tempMat = matMult(eyeMat, modelMat);

		//don't forget to apply the eyespace matrix to the light direction
		struct vector4 tempLightB = { sunDir.x, sunDir.y, sunDir.z, 0 };
		tempLightB = matMult(eyeMat, tempLightB);
		struct vector3 tempLightA = { tempLightB.x, tempLightB.y, tempLightB.z };
		tempLightA = normalize(tempLightA);

		//draw the cat
		for (int i = 0; i < catIndicesLen; i += 3)
		{
			//with the cat model, each vertex just has the triangle's normal vector
			struct vector3 tempNormA = {catNorms[i][0], catNorms[i][1], catNorms[i][2]};
			tempNormA = multV(normalize(tempNormA), -1);
			struct vector4 tempNormB = {tempNormA.x, tempNormA.y, tempNormA.z, 0};
			tempNormB = matMult(tempMat, tempNormB);
			tempNormA = {tempNormB.x, tempNormB.y, tempNormB.z};
			tempNormA = normalize(tempNormA);
			
			//temp variable for various culling things
			bool visible = true;

			struct vertice tempTri[3];
			for (int k = 0; k < 3; k++)
			{
				struct vector4 tempPosB = {catVerts[i + k][0], catVerts[i + k][1], catVerts[i + k][2], 1};
				tempPosB = matMult(tempMat, tempPosB);
				struct vector3 tempPosA = {tempPosB.x, tempPosB.y, tempPosB.z};
				//eye matrix has already happened, so this is just that
				struct vector3 tempEyeV = normalize(multV(tempPosA, -1));
				//eye-space back-face culling
				if (k == 0)
				{
					if (dot(tempEyeV, tempNormA) < 0)
					{
						visible = false;
						break;
					}
				}
				//struct vector4 tempColor = shade(tempEyeV, tempNormA, catDC, catSC, catSmooth, tempLightA, sunPower);
				struct vector4 tempColor = catDiff;
				//tempPosA = screenConvert(project(tempPosB, proMat));
				tempTri[k] = {tempPosA, tempNormA, tempEyeV, tempColor};
			}

			//now try to cull the triangle (backface, then eye-space x, y and z frustum culling)
			if (visible)
			{
				if (cullTri(tempTri[0].pos, tempTri[1].pos, tempTri[2].pos, nearBounds))
				{
					//now convert from eye space to screen space
					tempTri[0].pos = screenConvert(project(tempTri[0].pos, proMat));
					tempTri[1].pos = screenConvert(project(tempTri[1].pos, proMat));
					tempTri[2].pos = screenConvert(project(tempTri[2].pos, proMat));
					baryBoxTri(gPixels, zBuffer, tempTri);
				}
			}
		}

		//now copy the buffers
		//MAGIC CODE! DO NOT TOUCH
		struct fb_fix_screeninfo finfo;
		struct fb_var_screeninfo vinfo;

		int fb_fd = open("/dev/fb0", O_RDWR);

		//Get variable screen information
		ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);
		vinfo.grayscale = 0;
		vinfo.bits_per_pixel = 32;
		ioctl(fb_fd, FBIOPUT_VSCREENINFO, &vinfo);
		ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);

		ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo);

		long screensize = vinfo.yres_virtual * finfo.line_length;

		char *fbp = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, (off_t)0);

		int x, y;

		for (x = 0; x<vinfo.xres; x++)
			for (y = 0; y<vinfo.yres; y++)
			{
				long location = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) + (y + vinfo.yoffset) * finfo.line_length;
				*((uint32_t*)(fbp + location)) = gPixels[x+y*SCREEN_WIDTH];
			}
		//force the framebuffer to refresh
		vinfo.activate |= FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
		if(0 > ioctl(fb_fd, FBIOPUT_VSCREENINFO, &vinfo))
		{
			printf("Failed to refresh\n");
			return -1;
		}
	}
	delete[] gPixels;
	delete[] zBuffer;
	delete[] rawGPSlat;
	delete[] rawGPSlon;
	return(0);
}
