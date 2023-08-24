#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>

/////////Forward declarations
namespace Sphere {
extern void updateSphere(glm::vec3 pos, float radius);
}

namespace ClothMesh {
extern const int numCols;
extern const int numRows;
extern const int numVerts;
extern void updateClothMesh(float *array_data);
}

namespace {

static struct PhysParams {
	float elastic_coeff = 0.5f;
	float friction_coeff = 0.1f;

	float k_stretch[2] = {1000.f, 50.f};
	float k_shear[2] = {1000.f, 50.f};
	float k_bend[2] = {1000.f, 50.f};
	float particle_link = 0.5f;
	float max_elongation_percent = 0.05f;

	glm::vec3 gravity = glm::vec3(0.f, -9.81f, 0.f);

	bool play_Sim = false;

	bool useColisions = true;
	bool useElongationCorrection = true;
	bool useSphere = true;
	glm::vec3 sphereC = glm::vec3(0.f, 1.f, 0.f);
	float sphereR = 1.f;

	float accum_dt = 0.f;
	float timeToReset = 20.f;
} p_pars;

static struct ClothSystem {
	glm::vec3 *position;
	glm::vec3 *prev_pos;
	glm::vec3 *next_pos;
	bool *is_fixed;
	//float m; //optional mass
	glm::vec3 *velocities;
	glm::vec3 *forces;
} s_CS;

struct prim_data {
	struct PLN {
		glm::vec3 n;
		float d;
		PLN(glm::vec3 _n, float _d) : n(_n), d(_d) {};
	};
	struct SPH {
		glm::vec3 c;
		float r;
		SPH(glm::vec3 _c, float _r) : c(_c), r(_r) {};
	};
	enum prim { PLANE, SPHERE };
	prim type;
	char data[sizeof(PLN)];
};

static prim_data collider_primitives[8];

bool collidesPlane(const prim_data::PLN& plane, const glm::vec3& prev_pos, glm::vec3& new_pos) {
	float d_next = glm::dot(plane.n, new_pos) + plane.d;
	if(d_next < 0.f) {
		return true;
	}
	return false;
}
void computeCollisionPlane(const prim_data::PLN& plane, const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3& corrected_prev_pos, glm::vec3 &corrected_vel) {
	new_pos = new_pos - plane.n * ((glm::dot(plane.n, new_pos) + plane.d) * (1.f + p_pars.elastic_coeff));
	corrected_prev_pos = prev_pos - plane.n * ((glm::dot(plane.n, prev_pos) + plane.d) * (1.f + p_pars.elastic_coeff));
	float t = 1.f / 30.f;
	corrected_vel = (new_pos - corrected_prev_pos) / t;
	corrected_prev_pos = corrected_prev_pos + (corrected_vel - plane.n * glm::dot(corrected_vel, plane.n)) * (p_pars.friction_coeff * t);
	corrected_vel = (new_pos - corrected_prev_pos) / t;
}

bool collidesSphere(const prim_data::SPH& sphere, const glm::vec3& prev_pos, glm::vec3& new_pos) {
	glm::vec3 vec2Prev = sphere.c - prev_pos;
	glm::vec3 vec2New = sphere.c - new_pos;
	float d_prevSq = glm::dot(vec2Prev, vec2Prev) - sphere.r*sphere.r;
	float d_newSq = glm::dot(vec2New, vec2New) - sphere.r*sphere.r;
	if(d_newSq < 0.f && d_prevSq > 0.f)
		return true;
	return false;
}
void computeCollisionSphere(const prim_data::SPH& sphere, const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3& corrected_prev_pos, glm::vec3& corrected_vel) {
	glm::vec3 vec_cap_prev = sphere.c - prev_pos;
	float d_prev = glm::length(vec_cap_prev) - sphere.r;

	glm::vec3 vec_cap_new = sphere.c - new_pos;
	float d_new = glm::length(vec_cap_new) - sphere.r;

	glm::vec3 onSph = new_pos + (prev_pos - new_pos) * -d_new / (d_prev - d_new);
	glm::vec3 pln_n = glm::normalize(onSph - sphere.c);
	float pln_d = -glm::dot(pln_n, onSph);
	computeCollisionPlane(prim_data::PLN(pln_n, pln_d), prev_pos, new_pos, corrected_prev_pos, corrected_vel);
}

void computeCollision(const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3 &corrected_prev_pos, glm::vec3 &corrected_vel) {
	bool hasCollided = true;
	for(const auto& prim : collider_primitives) {
		switch(prim.type) {
		case prim_data::PLANE:
			if(collidesPlane(*(prim_data::PLN*)&prim.data, prev_pos, new_pos))
				computeCollisionPlane(*(prim_data::PLN*)&prim.data, prev_pos, new_pos, corrected_prev_pos, corrected_vel);
			break;
		case prim_data::SPHERE:
			if(p_pars.useSphere) {
				if(collidesSphere(*(prim_data::SPH*)&prim.data, prev_pos, new_pos))
					computeCollisionSphere(*(prim_data::SPH*)&prim.data, prev_pos, new_pos, corrected_prev_pos, corrected_vel);
			} else continue;
			break;
		}
	}
}

inline glm::vec3 springforce(const glm::vec3& P1, const glm::vec3& V1, const glm::vec3& P2, const glm::vec3& V2, float L0, float ke, float kd) {
	glm::vec3 P1mP2 = P1 - P2;
	float P1mP2_length = glm::length(P1mP2);
	glm::vec3 P1mP2_norm = P1mP2 * (1.f / P1mP2_length);

	return -(ke*(P1mP2_length - L0) + kd*glm::dot(V1 - V2, P1mP2_norm))*P1mP2_norm;
}

inline void computeSpringForces(int idxA, int idxB, float link, float ke, float kd) {
	glm::vec3 tmp_force = springforce(s_CS.position[idxA], s_CS.velocities[idxA], s_CS.position[idxB], s_CS.velocities[idxB], link, ke, kd);
	s_CS.forces[idxA] += tmp_force;
	s_CS.forces[idxB] -= tmp_force;
};

void calcforces(float dt) {
	/////////////////////////Gravity
	for(int i = 0; i < ClothMesh::numVerts; ++i) {
		s_CS.forces[i] = p_pars.gravity;
	}
	/////////////////////////Springs
	for(int i = 0; i < ClothMesh::numRows; ++i) {
		for(int j = 0; j < ClothMesh::numCols; ++j) {
			int idxA = i*ClothMesh::numCols + j;
			///////Structural springs
			{
				if(j < ClothMesh::numCols - 1) { //Horizontal
					computeSpringForces(idxA, i*ClothMesh::numCols + j + 1, p_pars.particle_link, p_pars.k_stretch[0], p_pars.k_stretch[1]);
				}
				if(i < ClothMesh::numRows - 1) { //Vertical
					computeSpringForces(idxA, (i + 1)*ClothMesh::numCols + j, p_pars.particle_link, p_pars.k_stretch[0], p_pars.k_stretch[1]);
				}
			}
			///////Shear springs
			{
				if(i < ClothMesh::numRows - 1 && j < ClothMesh::numCols - 1) { //Diag RightDown
					computeSpringForces(idxA, (i + 1)*ClothMesh::numCols + j + 1, glm::root_two<float>()*p_pars.particle_link, p_pars.k_shear[0], p_pars.k_shear[1]);
				}
				if(i < ClothMesh::numRows - 1 && j > 0) { //Diag LeftDown
					computeSpringForces(idxA, (i + 1)*ClothMesh::numCols + j - 1, glm::root_two<float>()*p_pars.particle_link, p_pars.k_shear[0], p_pars.k_shear[1]);
				}
			}
			///////Bend Springs
			{
				if(j < ClothMesh::numCols - 2) { //Horizontal
					computeSpringForces(idxA, i*ClothMesh::numCols + j + 2, 2 * p_pars.particle_link, p_pars.k_bend[0], p_pars.k_bend[1]);
				}
				if(i < ClothMesh::numRows - 2) { //Vertical
					computeSpringForces(idxA, (i + 2)*ClothMesh::numCols + j, 2 * p_pars.particle_link, p_pars.k_bend[0], p_pars.k_bend[1]);
				}
			}
		}
	}
}

inline bool correctElongation(glm::vec3& A, bool isfixedA, glm::vec3& B, bool isfixedB) {
	glm::vec3 AB = B - A;
	float lengthAB = glm::length(AB);
	float diffLength = lengthAB - p_pars.particle_link;
	float deviation = p_pars.particle_link*p_pars.max_elongation_percent;
	if(glm::abs(diffLength) > deviation) {
		diffLength += diffLength > 0.f ? -deviation : deviation;
		float moveDistA = isfixedA ? 0.f : (isfixedB ? diffLength : diffLength * 0.5f);
		float moveDistB = isfixedB ? 0.f : (isfixedA ? diffLength : diffLength * 0.5f);

		A = A + AB * (moveDistA / lengthAB);
		B = B - AB * (moveDistB / lengthAB);

		return true;
	}
	return false;
}

void correctElongation() {
	int cntCorrected = 1, prevCorrected = 0;
	while(cntCorrected > prevCorrected) {
		prevCorrected = cntCorrected;
		cntCorrected = 0;
		for(int i = 0; i < ClothMesh::numRows; ++i) {
			for(int j = 0; j < ClothMesh::numCols; ++j) {
				int idxA = i*ClothMesh::numCols + j;
				if(j < ClothMesh::numCols - 1) {
					int idxB = i*ClothMesh::numCols + j + 1;
					cntCorrected += correctElongation(s_CS.position[idxA], s_CS.is_fixed[idxA], s_CS.position[idxB], s_CS.is_fixed[idxB]) ? 1 : 0;
				}
				if(i < ClothMesh::numRows - 1) {
					int idxB = (i + 1)*ClothMesh::numCols + j;
					cntCorrected += correctElongation(s_CS.position[idxA], s_CS.is_fixed[idxA], s_CS.position[idxB], s_CS.is_fixed[idxB]) ? 1 : 0;
				}
			}
		}
	}
}

void verlet(float dt) {
	calcforces(dt);
	for(int i = 0; i < ClothMesh::numRows; ++i) {
		for(int j = 0; j < ClothMesh::numCols; ++j) {
			int idx = i*ClothMesh::numCols + j;
			if(!s_CS.is_fixed[idx]) {
				s_CS.next_pos[idx] = s_CS.position[idx] * 2.f - s_CS.prev_pos[idx] + s_CS.forces[idx] * (dt*dt);
			} else {
				s_CS.next_pos[idx] = s_CS.position[idx];
			}
		}
	}

	glm::vec3 *tmp = s_CS.prev_pos;
	s_CS.prev_pos = s_CS.position;
	s_CS.position = s_CS.next_pos;
	s_CS.next_pos = tmp;

	if(p_pars.useElongationCorrection) correctElongation();

	//Collisions & velocities
	for(int i = 0; i < ClothMesh::numRows; ++i) {
		for(int j = 0; j < ClothMesh::numCols; ++j) {
			int idx = i*ClothMesh::numCols + j;
			s_CS.velocities[idx] = (s_CS.position[idx] - s_CS.prev_pos[idx]) / dt;

			if(p_pars.useColisions && !s_CS.is_fixed[idx]) {
				computeCollision(s_CS.prev_pos[idx], s_CS.position[idx], s_CS.prev_pos[idx], s_CS.velocities[idx]);
			}
		}
	}
}

void initializeScene() {
	p_pars.sphereC.x = ((float)rand() / RAND_MAX) * 8.f - 4.f;
	p_pars.sphereC.y = ((float)rand() / RAND_MAX) * 5.f;
	p_pars.sphereC.z = ((float)rand() / RAND_MAX) * 8.f - 4.f;
	p_pars.sphereR = ((float)rand() / RAND_MAX) * 2.f + 1.f;
	Sphere::updateSphere(p_pars.sphereC, p_pars.sphereR);
	prim_data::SPH* sph = ((prim_data::SPH*)&collider_primitives[6].data);
	sph->c = p_pars.sphereC;
	sph->r = p_pars.sphereR;

	for(int i = 0; i < ClothMesh::numRows; ++i) {
		for(int j = 0; j < ClothMesh::numCols; ++j) {
			int idx = i*ClothMesh::numCols + j;
			s_CS.is_fixed[idx] = (idx == 0 || idx == ClothMesh::numCols - 1);
			s_CS.position[idx] =
				s_CS.prev_pos[idx] =
				glm::vec3(-4.f + i*p_pars.particle_link,
					9.f,
					j*p_pars.particle_link - p_pars.particle_link*(ClothMesh::numCols - 1.f) / 2.f);

			s_CS.velocities[idx] = glm::vec3(0.f);
		}
	}
	ClothMesh::updateClothMesh(&s_CS.position[0].x);
}
}


void P2_GUI() {
	ImGui::Checkbox("Play simulation", &p_pars.play_Sim);
	if(ImGui::Button("Reset")) {
		initializeScene();
		p_pars.accum_dt = 0.f;
	}

	ImGui::DragFloat("Reset time", &p_pars.timeToReset, 0.001f, 20.f, 120.f);
	ImGui::DragFloat3("Gravity Accel", &p_pars.gravity.x, 0.001f, -10.f, 10.f);

	if(ImGui::TreeNode("Spring parameters")) {
		ImGui::DragFloat2("k_stretch", p_pars.k_stretch);
		ImGui::DragFloat2("k_shear", p_pars.k_shear);
		ImGui::DragFloat2("k_bend", p_pars.k_bend);
		ImGui::DragFloat("Particle Link Distance", &p_pars.particle_link);
		//ImGui::DragFloat("Max Elongation Percent", &p_pars.max_elongation_percent, 0.001f, 0.f, 1.f);
		//ImGui::Checkbox("Use Elongation Correction", &p_pars.useElongationCorrection);
		ImGui::TreePop();
	}

	if(ImGui::TreeNode("Collisions")) {
		ImGui::Checkbox("Use Collisions", &p_pars.useColisions);
		if(p_pars.useColisions) {
			ImGui::Checkbox("Use Sphere Collider", &p_pars.useSphere);

			ImGui::DragFloat("Elastic Coefficient", &p_pars.elastic_coeff, 0.001f, 0.f, 1.f);
			ImGui::DragFloat("Friction Coefficient", &p_pars.friction_coeff, 0.001f, 0.f, 1.f);
		}
		ImGui::TreePop();
	}
	
	extern bool renderCloth; renderCloth = true;
	extern bool renderSphere; renderSphere = p_pars.useSphere && p_pars.useColisions;
}

void P2_PhysicsInit() {
	s_CS.position = new glm::vec3[ClothMesh::numVerts];
	s_CS.prev_pos = new glm::vec3[ClothMesh::numVerts];
	s_CS.next_pos = new glm::vec3[ClothMesh::numVerts];
	s_CS.is_fixed = new bool[ClothMesh::numVerts];

	s_CS.velocities = new glm::vec3[ClothMesh::numVerts];
	s_CS.forces = new glm::vec3[ClothMesh::numVerts];

	collider_primitives[0].type = prim_data::PLANE; prim_data::PLN* pln = ((prim_data::PLN*)&collider_primitives[0].data); pln->n = glm::vec3(1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[1].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[1].data); pln->n = glm::vec3(-1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[2].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[2].data); pln->n = glm::vec3(0.f, 1.f, 0.f); pln->d = 0.f;
	collider_primitives[3].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[3].data); pln->n = glm::vec3(0.f, -1.f, 0.f); pln->d = 10.f;
	collider_primitives[4].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[4].data); pln->n = glm::vec3(0.f, 0.f, 1.f); pln->d = 5.f;
	collider_primitives[5].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[5].data); pln->n = glm::vec3(0.f, 0.f, -1.f); pln->d = 5.f;
	collider_primitives[6].type = prim_data::SPHERE;

	initializeScene();
}

void P2_PhysicsUpdate(float dt) {
	if(p_pars.play_Sim) {
		p_pars.accum_dt += dt;
		if(p_pars.accum_dt > p_pars.timeToReset) {
			initializeScene();
			p_pars.accum_dt = 0.f;
			return;
		}
		int numLoops = 10;
		for(int i = 0; i < numLoops; ++i) {
			verlet(dt / numLoops);
		}
		ClothMesh::updateClothMesh(&s_CS.position[0].x);
	}
}

void P2_PhysicsCleanup() {
	delete[] s_CS.position;
	delete[] s_CS.prev_pos;
	delete[] s_CS.next_pos;
	delete[] s_CS.is_fixed;

	delete[] s_CS.velocities;
	delete[] s_CS.forces;
}

