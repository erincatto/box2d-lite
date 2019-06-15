/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "GL/glew.h"

#include "GLFW/glfw3.h"

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

namespace
{
	GLFWwindow* mainWindow = NULL;

	Body bodies[200];
	Joint joints[100];
	
	Body* bomb = NULL;

	float timeStep = 1.0f / 60.0f;
	int iterations = 10;
	Vec2 gravity(0.0f, -10.0f);

	int numBodies = 0;
	int numJoints = 0;

	int demoIndex = 0;

	int width = 1280;
	int height = 720;
	float zoom = 10.0f;
	float pan_y = 8.0f;

	GLuint program;
	GLfloat projection[16];
	GLint a_position_location, u_projection_location, u_color_location;

	World world(gravity, iterations);
}

static void glfwErrorCallback(int error, const char* description)
{
	printf("GLFW error %d: %s\n", error, description);
}

static GLuint LoadShader(GLenum type, const char* shaderSrc)
{
	GLuint shader;
	GLint compiled;
	shader = glCreateShader(type);
	if (shader == 0)
	{
		return 0;
	}
	glShaderSource(shader, 1, &shaderSrc, NULL);
	glCompileShader(shader);
	glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
	if (!compiled)
	{
		GLint infoLen = 0;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);
		if (infoLen > 1)
		{
			char* infoLog = (char*)malloc(sizeof(char) * infoLen);
			glGetShaderInfoLog(shader, infoLen, NULL, infoLog);
			fprintf(stderr, "Error compilling shader:\n%s\n", infoLog);
			free(infoLog);
		}
		glDeleteShader(shader);
		return 0;
	}
	return shader;
}

static void Ortho(GLfloat* projection, GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat near, GLfloat far)
{
	memset(projection, 0, sizeof(GLfloat)*16);
	projection[0]  = 2.0f / (right - left);
	projection[5]  = 2.0f / (top - bottom);
	projection[10] = -2.0f / (far - near);
	projection[12]  = -(right + left)/(right - left);
	projection[13]  = -(top + bottom)/(top - bottom);
	projection[14]  = -(far + near)/(far - near);
	projection[15] = 1.0f;
}

static void DrawText(int x, int y, const char* string)
{
	ImVec2 p;
	p.x = float(x);
	p.y = float(y);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(p);
	ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
	ImGui::End();
}

static void DrawBody(Body* body)
{
	Mat22 R(body->rotation);
	Vec2 x = body->position;
	Vec2 h = 0.5f * body->width;

	Vec2 positions[4], &v1 = positions[0], &v2 = positions[1], &v3 = positions[2], &v4 = positions[3];

	v1 = x + R * Vec2(-h.x, -h.y);
	v2 = x + R * Vec2( h.x, -h.y);
	v3 = x + R * Vec2( h.x,  h.y);
	v4 = x + R * Vec2(-h.x,  h.y);

	if (body == bomb)
		glUniform3f(u_color_location, 0.4f, 0.9f, 0.4f);
	else
		glUniform3f(u_color_location, 0.8f, 0.8f, 0.9f);
	glVertexAttribPointer(a_position_location, 2, GL_FLOAT, GL_FALSE, 0, positions);
	glDrawArrays(GL_LINE_LOOP, 0, 4);
}

static void DrawJoint(Joint* joint)
{
	Body* b1 = joint->body1;
	Body* b2 = joint->body2;

	Mat22 R1(b1->rotation);
	Mat22 R2(b2->rotation);

	Vec2 positions[4], &x1 = positions[0], &p1 = positions[1], &x2 = positions[2], &p2 = positions[3];

	x1 = b1->position;
	p1 = x1 + R1 * joint->localAnchor1;

	x2 = b2->position;
	p2 = x2 + R2 * joint->localAnchor2;

	glUniform3f(u_color_location, 0.5f, 0.5f, 0.8f);
	glVertexAttribPointer(a_position_location, 2, GL_FLOAT, GL_FALSE, 0, positions);
	glDrawArrays(GL_LINES, 0, 4);
}

static void LaunchBomb()
{
	if (!bomb)
	{
		bomb = bodies + numBodies;
		bomb->Set(Vec2(1.0f, 1.0f), 50.0f);
		bomb->friction = 0.2f;
		world.Add(bomb);
		++numBodies;
	}

	bomb->position.Set(Random(-15.0f, 15.0f), 15.0f);
	bomb->rotation = Random(-1.5f, 1.5f);
	bomb->velocity = -1.5f * bomb->position;
	bomb->angularVelocity = Random(-20.0f, 20.0f);
}

// Single box
static void Demo1(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 4.0f);
	world.Add(b);
	++b; ++numBodies;
}

// A simple pendulum
static void Demo2(Body* b, Joint* j)
{
	Body* b1 = b + 0;
	b1->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b1->friction = 0.2f;
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	b1->rotation = 0.0f;
	world.Add(b1);

	Body* b2 = b + 1;
	b2->Set(Vec2(1.0f, 1.0f), 100.0f);
	b2->friction = 0.2f;
	b2->position.Set(9.0f, 11.0f);
	b2->rotation = 0.0f;
	world.Add(b2);

	numBodies += 2;

	j->Set(b1, b2, Vec2(0.0f, 11.0f));
	world.Add(j);

	numJoints += 1;
}

// Varying friction coefficients
static void Demo3(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 11.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(5.25f, 9.5f);
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(2.0f, 7.0f);
	b->rotation = 0.25f;
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(-5.25f, 5.5f);
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 3.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; ++numBodies;

	float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
	for (int i = 0; i < 5; ++i)
	{
		b->Set(Vec2(0.5f, 0.5f), 25.0f);
		b->friction = friction[i];
		b->position.Set(-7.5f + 2.0f * i, 14.0f);
		world.Add(b);
		++b; ++numBodies;
	}
}

// A vertical stack
static void Demo4(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(Vec2(1.0f, 1.0f), 1.0f);
		b->friction = 0.2f;
		float x = Random(-0.1f, 0.1f);
		b->position.Set(x, 0.51f + 1.05f * i);
		world.Add(b);
		++b; ++numBodies;
	}
}

// A pyramid
static void Demo5(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	Vec2 x(-6.0f, 0.75f);
	Vec2 y;

	for (int i = 0; i < 12; ++i)
	{
		y = x;

		for (int j = i; j < 12; ++j)
		{
			b->Set(Vec2(1.0f, 1.0f), 10.0f);
			b->friction = 0.2f;
			b->position = y;
			world.Add(b);
			++b; ++numBodies;

			y += Vec2(1.125f, 0.0f);
		}

		//x += Vec2(0.5625f, 1.125f);
		x += Vec2(0.5625f, 2.0f);
	}
}

// A teeter
static void Demo6(Body* b, Joint* j)
{
	Body* b1 = b + 0;
	b1->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	world.Add(b1);

	Body* b2 = b + 1;
	b2->Set(Vec2(12.0f, 0.25f), 100.0f);
	b2->position.Set(0.0f, 1.0f);
	world.Add(b2);

	Body* b3 = b + 2;
	b3->Set(Vec2(0.5f, 0.5f), 25.0f);
	b3->position.Set(-5.0f, 2.0f);
	world.Add(b3);

	Body* b4 = b + 3;
	b4->Set(Vec2(0.5f, 0.5f), 25.0f);
	b4->position.Set(-5.5f, 2.0f);
	world.Add(b4);

	Body* b5 = b + 4;
	b5->Set(Vec2(1.0f, 1.0f), 100.0f);
	b5->position.Set(5.5f, 15.0f);
	world.Add(b5);

	numBodies += 5;

	j->Set(b1, b2, Vec2(0.0f, 1.0f));
	world.Add(j);

	numJoints += 1;
}

// A suspension bridge
static void Demo7(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; ++numBodies;

	const int numPlanks = 15;
	float mass = 50.0f;

	for (int i = 0; i < numPlanks; ++i)
	{
		b->Set(Vec2(1.0f, 0.25f), mass);
		b->friction = 0.2f;
		b->position.Set(-8.5f + 1.25f * i, 5.0f);
		world.Add(b);
		++b; ++numBodies;
	}

	// Tuning
	float frequencyHz = 2.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stifness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	for (int i = 0; i < numPlanks; ++i)
	{
		j->Set(bodies+i, bodies+i+1, Vec2(-9.125f + 1.25f * i, 5.0f));
		j->softness = softness;
		j->biasFactor = biasFactor;

		world.Add(j);
		++j; ++numJoints;
	}

	j->Set(bodies + numPlanks, bodies, Vec2(-9.125f + 1.25f * numPlanks, 5.0f));
	j->softness = softness;
	j->biasFactor = biasFactor;
	world.Add(j);
	++j; ++numJoints;
}

// Dominos
static void Demo8(Body* b, Joint* j)
{
	Body* b1 = b;
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; ++numBodies;

	b->Set(Vec2(12.0f, 0.5f), FLT_MAX);
	b->position.Set(-1.5f, 10.0f);
	world.Add(b);
	++b; ++numBodies;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(Vec2(0.2f, 2.0f), 10.0f);
		b->position.Set(-6.0f + 1.0f * i, 11.125f);
		b->friction = 0.1f;
		world.Add(b);
		++b; ++numBodies;
	}

	b->Set(Vec2(14.0f, 0.5f), FLT_MAX);
	b->position.Set(1.0f, 6.0f);
	b->rotation = 0.3f;
	world.Add(b);
	++b; ++numBodies;

	Body* b2 = b;
	b->Set(Vec2(0.5f, 3.0f), FLT_MAX);
	b->position.Set(-7.0f, 4.0f);
	world.Add(b);
	++b; ++numBodies;

	Body* b3 = b;
	b->Set(Vec2(12.0f, 0.25f), 20.0f);
	b->position.Set(-0.9f, 1.0f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b1, b3, Vec2(-2.0f, 1.0f));
	world.Add(j);
	++j; ++numJoints;

	Body* b4 = b;
	b->Set(Vec2(0.5f, 0.5f), 10.0f);
	b->position.Set(-10.0f, 15.0f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b2, b4, Vec2(-7.0f, 15.0f));
	world.Add(j);
	++j; ++numJoints;

	Body* b5 = b;
	b->Set(Vec2(2.0f, 2.0f), 20.0f);
	b->position.Set(6.0f, 2.5f);
	b->friction = 0.1f;
	world.Add(b);
	++b; ++numBodies;

	j->Set(b1, b5, Vec2(6.0f, 2.6f));
	world.Add(j);
	++j; ++numJoints;

	Body* b6 = b;
	b->Set(Vec2(2.0f, 0.2f), 10.0f);
	b->position.Set(6.0f, 3.6f);
	world.Add(b);
	++b; ++numBodies;

	j->Set(b5, b6, Vec2(7.0f, 3.5f));
	world.Add(j);
	++j; ++numJoints;
}

// A multi-pendulum
static void Demo9(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);

	Body * b1 = b;
	++b;
	++numBodies;

	float mass = 10.0f;

	// Tuning
	float frequencyHz = 4.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stiffness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	const float y = 12.0f;

	for (int i = 0; i < 15; ++i)
	{
		Vec2 x(0.5f + i, y);
		b->Set(Vec2(0.75f, 0.25f), mass);
		b->friction = 0.2f;
		b->position = x;
		b->rotation = 0.0f;
		world.Add(b);

		j->Set(b1, b, Vec2(float(i), y));
		j->softness = softness;
		j->biasFactor = biasFactor;
		world.Add(j);

		b1 = b;
		++b;
		++numBodies;
		++j;
		++numJoints;
	}
}

void (*demos[])(Body* b, Joint* j) = {Demo1, Demo2, Demo3, Demo4, Demo5, Demo6, Demo7, Demo8, Demo9};
const char* demoStrings[] = {
	"Demo 1: A Single Box",
	"Demo 2: Simple Pendulum",
	"Demo 3: Varying Friction Coefficients",
	"Demo 4: Randomized Stacking",
	"Demo 5: Pyramid Stacking",
	"Demo 6: A Teeter",
	"Demo 7: A Suspension Bridge",
	"Demo 8: Dominos",
	"Demo 9: Multi-pendulum"};

static void InitDemo(int index)
{
	world.Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	demoIndex = index;
	demos[index](bodies, joints);
}

static void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS)
	{
		return;
	}

	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// Quit
		glfwSetWindowShouldClose(mainWindow, GL_TRUE);
		break;

	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		InitDemo(key - GLFW_KEY_1);
		break;

	case GLFW_KEY_A:
		World::accumulateImpulses = !World::accumulateImpulses;
		break;

	case GLFW_KEY_P:
		World::positionCorrection = !World::positionCorrection;
		break;

	case GLFW_KEY_W:
		World::warmStarting = !World::warmStarting;
		break;

	case GLFW_KEY_SPACE:
		LaunchBomb();
		break;
	}
}

static void Reshape(GLFWwindow*, int w, int h)
{
	width = w;
	height = h > 0 ? h : 1;

	glViewport(0, 0, width, height);

	float aspect = float(width) / float(height);
	if (width >= height)
	{
		// aspect >= 1, set the height from -1 to 1, with larger width
		Ortho(projection, -zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);
	}
	else
	{
		// aspect < 1, set the width to -1 to 1, with larger height
		Ortho(projection, -zoom, zoom, -zoom / aspect + pan_y, zoom / aspect + pan_y, -1.0, 1.0);
	}
}

void frame()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	// Globally position text
	ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::End();

	DrawText(5, 5, demoStrings[demoIndex]);
	DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

	char buffer[64];
	sprintf(buffer, "(A)ccumulation %s", World::accumulateImpulses ? "ON" : "OFF");
	DrawText(5, 65, buffer);

	sprintf(buffer, "(P)osition Correction %s", World::positionCorrection ? "ON" : "OFF");
	DrawText(5, 95, buffer);

	sprintf(buffer, "(W)arm Starting %s", World::warmStarting ? "ON" : "OFF");
	DrawText(5, 125, buffer);

	world.Step(timeStep);

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glEnableVertexAttribArray(a_position_location);
	glUseProgram(program);
	glUniformMatrix4fv(u_projection_location, 1, GL_FALSE, projection);

	for (int i = 0; i < numBodies; ++i)
		DrawBody(bodies + i);

	for (int i = 0; i < numJoints; ++i)
		DrawJoint(joints + i);

	glUniform3f(u_color_location, 1.0f, 0.0f, 0.0f);
	std::map<ArbiterKey, Arbiter>::const_iterator iter;
	std::vector<Vec2> contacts;
	contacts.reserve(world.arbiters.size() * Arbiter::MAX_POINTS);
	for (iter = world.arbiters.begin(); iter != world.arbiters.end(); ++iter)
	{
		const Arbiter& arbiter = iter->second;
		for (int i = 0; i < arbiter.numContacts; ++i)
		{
			contacts.push_back(arbiter.contacts[i].position);
		}
	}
	glVertexAttribPointer(a_position_location, 2, GL_FLOAT, GL_FALSE, 0, contacts.data());
	glDrawArrays(GL_POINTS, 0, contacts.size());

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwPollEvents();
	glfwSwapBuffers(mainWindow);
}

int main(int, char**)
{
	glfwSetErrorCallback(glfwErrorCallback);

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	mainWindow = glfwCreateWindow(width, height, "box2d-lite", NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);

	fprintf(stdout, "Version OpenGL: %s\n", glGetString(GL_VERSION));
	fprintf(stdout, "Version GLSL: %s\n", glGetString(GL_SHADING_LANGUAGE_VERSION));

	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		fprintf(stderr, "GLEW error: %s\n", glewGetErrorString(err));
		return -1;
	}
	fprintf(stdout, "Version GLEW: %s\n", glewGetString(GLEW_VERSION));

	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	const char vertexShaderSrc[] =
	"attribute vec2 a_position;                                 \n"
	"uniform mat4 u_projection;                                 \n"
	"void main()                                                \n"
	"{                                                          \n"
	"   gl_PointSize = 4.0;                                     \n"
	"   gl_Position = u_projection * vec4(a_position, 0.0, 1.0);\n"
	"}";

	const char fragmentShaderSrc[] =
	"precision mediump float;             \n"
	"uniform vec3 u_color;                \n"
	"void main()                          \n"
	"{                                    \n"
	"   gl_FragColor = vec4(u_color, 1.0);\n"
	"}";

	GLuint shaders[2];
	shaders[0] = LoadShader(GL_VERTEX_SHADER, vertexShaderSrc);
	shaders[1] = LoadShader(GL_FRAGMENT_SHADER, fragmentShaderSrc);

	program = glCreateProgram();
	glAttachShader(program, shaders[0]);
	glAttachShader(program, shaders[1]);

	glLinkProgram(program);
	GLint linked;
	glGetProgramiv(program, GL_LINK_STATUS, &linked);
	if (!linked)
	{
		glDeleteProgram(program);
		return -1;
	}
	a_position_location = glGetAttribLocation(program, "a_position");
	u_projection_location = glGetUniformLocation(program, "u_projection");
	u_color_location = glGetUniformLocation(program, "u_color");

	glfwSwapInterval(1);
	glfwSetWindowSizeCallback(mainWindow, Reshape);
	glfwSetKeyCallback(mainWindow, Keyboard);

	IMGUI_CHECKVERSION();
	fprintf(stdout, "Version ImGui: %s\n", IMGUI_VERSION);
	ImGui::CreateContext();
	ImGui::StyleColorsClassic();
	ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
	ImGui_ImplOpenGL3_Init();
	ImGuiIO& io = ImGui::GetIO();

#ifndef __EMSCRIPTEN__
	float xscale, yscale;
	glfwGetWindowContentScale(mainWindow, &xscale, &yscale);
	float uiScale = xscale;
	io.FontGlobalScale = uiScale;
#endif

	Reshape(mainWindow, width, height);

	InitDemo(0);

#ifdef __EMSCRIPTEN__
	emscripten_set_main_loop(frame, 60, 1);
#else
	while (!glfwWindowShouldClose(mainWindow))
	{
		frame();
	}
	glfwTerminate();
#endif

	return 0;
}
