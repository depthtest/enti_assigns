#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>

/////////Forward declarations
namespace Cube {
extern void updateCube(const glm::mat4& transform);
extern const float halfW;
}

namespace {

static struct PhysParams {
	float elastic_coeff = 0.5f;
	float friction_coeff = 0.1f;

	float tolerance = 0.1f;

	glm::vec3 gravity = glm::vec3(0.f, -9.81f, 0.f);

	bool play_Sim = false;

	bool useColisions = true;

	float accum_dt = 0.f;
	float timeToReset = 15.f;
} p_pars;

struct RigidCube {
	glm::vec3 pos;
	glm::quat ori;
	glm::vec3 P;
	glm::vec3 L;
	float inv_mass;
	glm::mat3 IbodyInv;

	glm::vec3 force;
	glm::vec3 torque;

	glm::vec3 v;
	glm::vec3 w;
	glm::mat4 xform;
	glm::mat3 I_Inv;

	glm::vec3 verts[8] = {
		glm::vec3(-Cube::halfW, -Cube::halfW, -Cube::halfW),
		glm::vec3(-Cube::halfW, -Cube::halfW,  Cube::halfW),
		glm::vec3(Cube::halfW, -Cube::halfW,  Cube::halfW),
		glm::vec3(Cube::halfW, -Cube::halfW, -Cube::halfW),
		glm::vec3(-Cube::halfW,  Cube::halfW, -Cube::halfW),
		glm::vec3(-Cube::halfW,  Cube::halfW,  Cube::halfW),
		glm::vec3(Cube::halfW,  Cube::halfW,  Cube::halfW),
		glm::vec3(Cube::halfW,  Cube::halfW, -Cube::halfW)
	};
};
static RigidCube s_RC;
static RigidCube s_RC_tmp;

struct prim_data {
	struct PLN {
		glm::vec3 n;
		float d;
		PLN(glm::vec3 _n, float _d) : n(_n), d(_d) {};
	};
	enum prim { PLANE };
	prim type;
	char data[sizeof(PLN)];
};

static prim_data collider_primitives[6];

void computeImpulseForContact(glm::vec3 ra, glm::mat3 invIa, float vrel, glm::vec3 normal) {
	float jup = -(1 + p_pars.elastic_coeff) * vrel;
	float jdown = (s_RC.inv_mass + glm::dot(normal, glm::cross(s_RC_tmp.I_Inv * glm::cross(ra, normal), ra)));
	float j = jup / jdown;
	glm::vec3 J = j * normal;
	glm::vec3 Jc = glm::cross(ra, J);
	s_RC_tmp.P += J;
	s_RC_tmp.L += Jc;

	s_RC_tmp.v = s_RC_tmp.v + J * s_RC.inv_mass;
	s_RC_tmp.w = s_RC_tmp.w + s_RC_tmp.I_Inv * Jc;
}

void computeResponses() {
	for(auto v : s_RC.verts) {
		glm::vec4 v2 = s_RC_tmp.xform * glm::vec4(v, 1.f);
		glm::vec3 vA = s_RC_tmp.v + glm::cross(s_RC_tmp.w, glm::vec3(v2.x, v2.y, v2.z) - s_RC_tmp.pos);
		for(auto p : collider_primitives) {
			prim_data::PLN *pln = reinterpret_cast<prim_data::PLN*>(p.data);
			if(glm::dot(pln->n, glm::vec3(v2.x, v2.y, v2.z)) + pln->d < p_pars.tolerance) {
				computeImpulseForContact(glm::vec3(v2.x, v2.y, v2.z) - s_RC_tmp.pos, s_RC_tmp.I_Inv, glm::dot(pln->n, vA), pln->n);
			}
		}
	}
}

void correctPositions() {
	for(auto v : s_RC.verts) {
		for(auto p : collider_primitives) {
			glm::vec4 v2 = s_RC.xform * glm::vec4(v, 1.f);
			prim_data::PLN *pln = reinterpret_cast<prim_data::PLN*>(p.data);
			float dist = glm::dot(pln->n, glm::vec3(v2.x, v2.y, v2.z)) + pln->d;

			if(dist < 0.f) {
				s_RC.pos -= pln->n * dist*1.001f;
				s_RC.xform =
					glm::translate(glm::mat4(1.f), s_RC.pos) *
					glm::mat4_cast(s_RC.ori);
			}
		}
	}
}

bool isCollision(bool &within_tolerance) {
	for(auto v : s_RC.verts) {
		glm::vec3 prev_v2 = s_RC.xform * glm::vec4(v, 1.f);
		glm::vec4 v2 = s_RC_tmp.xform * glm::vec4(v, 1.f);
		glm::vec3 vA = s_RC_tmp.v + glm::cross(s_RC_tmp.w, glm::vec3(v2.x, v2.y, v2.z) - s_RC_tmp.pos);
		for(auto p : collider_primitives) {
			prim_data::PLN *pln = reinterpret_cast<prim_data::PLN*>(p.data);
			float pdist_prev = glm::dot(pln->n, glm::vec3(prev_v2.x, prev_v2.y, prev_v2.z)) + pln->d;
			float pdist_post = glm::dot(pln->n, glm::vec3(v2.x, v2.y, v2.z)) + pln->d;
			if(pdist_prev*pdist_post < 0.f && glm::dot(pln->n, vA) < 0.f) {
				if(pdist_post > 0.f && pdist_post < p_pars.tolerance) within_tolerance = true;
				return true;
			}
		}
	}

	return false;
}

void commit() {
	s_RC.P = s_RC_tmp.P;
	s_RC.L = s_RC_tmp.L;
	s_RC.pos = s_RC_tmp.pos;
	s_RC.ori = s_RC_tmp.ori;
	s_RC.xform = s_RC_tmp.xform;
	s_RC.force = s_RC_tmp.force;
	s_RC.torque = s_RC_tmp.torque;
}

void calcforces() {
	s_RC_tmp.force = p_pars.gravity / s_RC.inv_mass;
	s_RC_tmp.torque = glm::vec3(0.f);
}

float euler_step(float dt) {
	s_RC_tmp.P = s_RC.P + dt * s_RC.force;
	s_RC_tmp.L = s_RC.L + dt * s_RC.torque;

	s_RC_tmp.v = s_RC_tmp.P * s_RC.inv_mass;
	s_RC_tmp.pos = s_RC.pos + dt * s_RC_tmp.v;

	glm::mat3 rot = glm::mat3_cast(s_RC.ori);
	glm::mat3 I = rot * s_RC.IbodyInv * glm::transpose(rot);
	s_RC_tmp.I_Inv = glm::inverse(I);
	s_RC_tmp.w = s_RC_tmp.I_Inv * s_RC_tmp.L;
	glm::quat quatDot = 0.5f * glm::quat(0.f, s_RC_tmp.w) * s_RC.ori;
	s_RC_tmp.ori = glm::normalize(s_RC.ori + dt * quatDot);

	calcforces();

	s_RC_tmp.xform =
		glm::translate(glm::mat4(1.f), s_RC_tmp.pos) *
		glm::mat4_cast(s_RC_tmp.ori);

	return dt;
}

float euler_rec(float dt, int level) {
	float sim_dt = euler_step(dt);

	if(level == 10) { //Max level recursion
		computeResponses();
		commit();
		return sim_dt;
	}

	bool in_tolerance = false;
	if(p_pars.useColisions) {
		if(isCollision(in_tolerance)) {
			if(in_tolerance) {
				computeResponses();
				commit();
				return sim_dt;
			} else return euler_rec(dt / 2.f, level + 1);
		}
	}
	commit();
	return sim_dt;
}

void initializeScene() {
	s_RC.inv_mass = 1.f / ((float)rand() / RAND_MAX * 20.f + 5.f);

	glm::mat3 Ibod = glm::mat3(1.f / 3.f * (2.f * 0.5f * 0.5f) / s_RC.inv_mass);
	s_RC.IbodyInv = glm::inverse(Ibod);

	s_RC.P = glm::vec3(0.f);
	s_RC.L = glm::vec3(0.f);
	s_RC.pos = glm::vec3(
		(float)rand() / RAND_MAX * 8.f - 4.f,
		5.f,
		(float)rand() / RAND_MAX * 8.f - 4.f
	);

	s_RC.force = glm::vec3(
		((float)rand() / RAND_MAX < 0.5 ? -1 : 1) * ((float)rand() / RAND_MAX * 400.f + 400.f),
		(float)rand() / RAND_MAX * 300.f + 800.f,
		((float)rand() / RAND_MAX < 0.5 ? -1 : 1) * ((float)rand() / RAND_MAX * 400.f + 400.f)
	);
	s_RC.torque = glm::cross(glm::vec3(
		((float)rand() / RAND_MAX) > 0.5 ? 0.2 : -0.2f,
		-0.5,//((float)rand() / RAND_MAX) > 0.5 ? 0.5 : -0.5f,
		((float)rand() / RAND_MAX) > 0.5 ? 0.2 : -0.2f),
		s_RC.force);

	s_RC.ori = glm::quat();

	glm::mat4 xform =
		glm::translate(glm::mat4(1.f), s_RC.pos) *
		glm::mat4_cast(s_RC.ori);

	s_RC_tmp = s_RC;

	Cube::updateCube(xform);
}

void resetScene() {
	initializeScene();
	p_pars.accum_dt = 0.f;
}

void do_euler(float dt) {
	float to_sim_dt = dt;
	while(to_sim_dt > 0.f) {
		to_sim_dt -= euler_rec(to_sim_dt, 0);
		correctPositions();
	}
}

}

void P3_GUI() {
	ImGui::Checkbox("Play simulation", &p_pars.play_Sim);
	if(ImGui::Button("Reset")) {
		initializeScene();
		p_pars.accum_dt = 0.f;
	}

	ImGui::DragFloat("Reset time", &p_pars.timeToReset, 0.001f, 2.f, 120.f);
	ImGui::DragFloat3("Gravity Accel", &p_pars.gravity.x, 0.001f, -10.f, 10.f);

	if(ImGui::TreeNode("Collisions")) {
		ImGui::Checkbox("Use Collisions", &p_pars.useColisions);
		if(p_pars.useColisions) {
			ImGui::DragFloat("Tolerance", &p_pars.tolerance, 0.001f, 0.01f, 1.f);
			ImGui::DragFloat("Elastic Coefficient", &p_pars.elastic_coeff, 0.001f, 0.f, 1.f);
			//ImGui::DragFloat("Friction Coefficient", &p_pars.friction_coeff, 0.001f, 0.f, 1.f);
		}
		ImGui::TreePop();
	}
}

void P3_PhysicsInit() {
	collider_primitives[0].type = prim_data::PLANE; prim_data::PLN* pln = ((prim_data::PLN*)&collider_primitives[0].data); pln->n = glm::vec3(1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[1].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[1].data); pln->n = glm::vec3(-1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[2].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[2].data); pln->n = glm::vec3(0.f, 1.f, 0.f); pln->d = 0.f;
	collider_primitives[3].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[3].data); pln->n = glm::vec3(0.f, -1.f, 0.f); pln->d = 10.f;
	collider_primitives[4].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[4].data); pln->n = glm::vec3(0.f, 0.f, 1.f); pln->d = 5.f;
	collider_primitives[5].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[5].data); pln->n = glm::vec3(0.f, 0.f, -1.f); pln->d = 5.f;

	extern bool renderCube; renderCube = true;

	initializeScene();
}

void P3_PhysicsUpdate(float dt) {
	if(p_pars.play_Sim) {
		p_pars.accum_dt += dt;
		if(p_pars.accum_dt > p_pars.timeToReset) {
			resetScene();
			return;
		}
		do_euler(dt);

		Cube::updateCube(s_RC.xform);
	}
}

void P3_PhysicsCleanup() {}

