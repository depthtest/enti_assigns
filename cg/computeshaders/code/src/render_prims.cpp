#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_sdl_gl3.h>

// Boolean variables allow to show/hide the primitives
bool renderSphere = false;
bool renderCapsule = false;
bool renderParticles = false;
bool renderCloth = false;
bool renderCube = false;

extern GLuint compileShader(const char* shaderStr, GLenum shaderType, const char* name = "");
extern void linkProgram(GLuint program);
namespace Sphere {
	extern GLuint sphereProgram;
	extern void setupSphere(glm::vec3 pos, float radius);
	extern void updateSphere(glm::vec3 pos, float radius);
	extern void drawSphere();
	extern void cleanupSphere();
}
namespace RenderVars {
	extern glm::mat4 _projection;
	extern glm::mat4 _modelView;
	extern glm::mat4 _MVP;
}

bool play_Sim = false;
glm::vec4 SphPos(0.f, 0.f, 0.f, 1.f);

void rGUI() {
	bool show = true;
	ImGui::Begin("Parameters", &show, 0);

	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate

		ImGui::Checkbox("Play simulation", &play_Sim);

		bool isChanged = false;
		isChanged |= ImGui::DragFloat3("Sphere Position", &SphPos.x, 0.01f, -5.f, 10.f);
		isChanged |= ImGui::DragFloat("Sphere Radius", &SphPos.w, 0.01f, 0.1f, 5.f);
		if (isChanged) {
			Sphere::updateSphere(glm::vec3(SphPos.x, SphPos.y, SphPos.z), SphPos.w);
		}
	}
	
	ImGui::End();
}

#include <random>
#include <iostream>

#define NUM_PARTICLES 1024*1024
#define WGROUP_SIZE 128

GLuint SSbo[2];
GLuint Vao;

GLuint c_shader;
GLuint c_program;
const char* compute_shader =
"																		\n \
#version 430															\n \
layout( std430, binding=0 ) buffer Pos {vec4 pos[];};					\n \
layout( std430, binding=1 ) buffer Vel {vec4 vel[];};					\n \
layout( local_size_x = 128, local_size_y = 1, local_size_z = 1 ) in;	\n \
uniform vec4 SphPos;													\n \
uniform float DT;														\n \
const float eps = 0.4;													\n \
const float G = 9.81;													\n \
vec4 onSph(vec3 sph, float rad, vec3 pre_x, vec3 pos_x) {				\n \
	float cc = dot(sph, sph);											\n \
	float pp = dot(pre_x, pre_x);										\n \
	float cp = dot(sph, pre_x);											\n \
	float pq = dot(pre_x, pos_x);										\n \
	float cq = dot(sph, pos_x);											\n \
	float qq = dot(pos_x, pos_x);										\n \
	float a = qq + pp - 2 * pq;											\n \
	float b = 2 * cp + 2 * pq - 2 * cq - 2 * pp;						\n \
	float c = cc + pp - 2 * cp - rad * rad;								\n \
	float alpha_p = (-b + sqrt(b*b - 4 * a*c)) / (2 * a);				\n \
	float alpha_m = (-b - sqrt(b*b - 4 * a*c)) / (2 * a);				\n \
	float alpha = alpha_p;												\n \
	if (0.0 < alpha_m && alpha_m < 1.0) alpha = alpha_m;				\n \
	return vec4(pre_x + ((pos_x + (-pre_x)) * alpha), 1.0);				\n \
}																		\n \
void main() {															\n \
	uint gid = gl_GlobalInvocationID.x;									\n \
	vec4 p = pos[gid];													\n \
	vec4 v = vel[gid];													\n \
	vec4 grav = G * normalize(vec4(SphPos.xyz, 1.) - p);				\n \
	vec4 np = p + v * DT;												\n \
	vec4 nv = v + grav * DT;											\n \
	if(length(np.xyz - SphPos.xyz) < SphPos.w) {						\n \
		vec4 onS = onSph(SphPos.xyz, SphPos.w, p.xyx, np.xyz);			\n \
		vec4 n = normalize(onS - SphPos);								\n \
		float d = -dot(n, onS);											\n \
		np = np - (1+eps)*(dot(np, n)+d)*n;								\n \
		nv = nv - (1+eps)*(dot(nv, n))*n;								\n \
	}																	\n \
	pos[gid] = np;														\n \
	vel[gid] = nv;														\n \
}";

void setupPrims() {
	std::random_device rd;
	std::default_random_engine rnd_gen(rd());
	std::uniform_real_distribution<float> rnd_dist(0.f, 1.f);

	glGenBuffers(2, SSbo);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSbo[0]);
	glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_PARTICLES * sizeof(float) * 4, NULL, GL_STATIC_DRAW);
	float *pos = reinterpret_cast<float*>(glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, NUM_PARTICLES* sizeof(float) * 4, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT));
	for (int i = 0; i < NUM_PARTICLES; ++i) {
		*pos++ = rnd_dist(rnd_gen)*10.f - 5.f,
		*pos++ = rnd_dist(rnd_gen)*5.f + 5.f,
		*pos++ = rnd_dist(rnd_gen)*10.f - 5.f;
		*pos++ = 1.f;
	}
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, SSbo[1]);
	glBufferData(GL_SHADER_STORAGE_BUFFER, NUM_PARTICLES * sizeof(float) * 4, NULL, GL_STATIC_DRAW);
	float *vel = reinterpret_cast<float*>(glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, NUM_PARTICLES * sizeof(float) * 4, GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT));
	for (int i = 0; i < NUM_PARTICLES; ++i) {
		*vel++ = rnd_dist(rnd_gen)*4.f - 2.f,
		*vel++ = rnd_dist(rnd_gen)*0.f - 0.f,
		*vel++ = rnd_dist(rnd_gen)*4.f - 2.f;
		*vel++ = 0.f;
	}
	glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

	glGenVertexArrays(1, &Vao);
	glBindVertexArray(Vao);
	glBindBuffer(GL_ARRAY_BUFFER, SSbo[0]);
	glVertexAttribPointer((GLuint)0, 4, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	c_shader = compileShader(compute_shader, GL_COMPUTE_SHADER);
	c_program = glCreateProgram();
	glAttachShader(c_program, c_shader);
	linkProgram(c_program);

	Sphere::setupSphere(glm::vec3(0.f, 0.f, 0.f), 1.f);
}
void cleanupPrims() {
	glDeleteBuffers(2, SSbo);
	glDeleteVertexArrays(1, &Vao);

	glDeleteProgram(c_program);
	glDeleteShader(c_shader);

	Sphere::cleanupSphere();
}

void renderPrims(float dt) {
	if (play_Sim) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, SSbo[0]);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, SSbo[1]);

		glUseProgram(c_program);
		glUniform4fv(glGetUniformLocation(c_program, "SphPos"), 1, &SphPos.x);
		glUniform1f(glGetUniformLocation(c_program, "DT"), dt);
		glDispatchCompute(NUM_PARTICLES / WGROUP_SIZE, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glUseProgram(Sphere::sphereProgram);
	glUniformMatrix4fv(glGetUniformLocation(Sphere::sphereProgram, "mvpMat"), 1, GL_FALSE, glm::value_ptr(RenderVars::_MVP));
	glUniformMatrix4fv(glGetUniformLocation(Sphere::sphereProgram, "mv_Mat"), 1, GL_FALSE, glm::value_ptr(RenderVars::_modelView));
	glUniformMatrix4fv(glGetUniformLocation(Sphere::sphereProgram, "projMat"), 1, GL_FALSE, glm::value_ptr(RenderVars::_projection));
	glUniform4f(glGetUniformLocation(Sphere::sphereProgram, "color"), 0.1f, 0.1f, 0.6f, 1.f);
	glUniform1f(glGetUniformLocation(Sphere::sphereProgram, "radius"), 0.01f);

	glBindVertexArray(Vao);
	glDrawArrays(GL_POINTS, 0, NUM_PARTICLES);
	glBindVertexArray(0);
	glUseProgram(0);

	Sphere::drawSphere();
}

