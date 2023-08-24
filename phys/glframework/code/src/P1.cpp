#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>

/////////Forward declarations
namespace Sphere {
extern void updateSphere(glm::vec3 pos, float radius);
}
namespace Capsule {
extern void updateCapsule(glm::vec3 posA, glm::vec3 posB, float radius);
}
namespace LilSpheres {
extern const int maxParticles;
extern int firstParticleIdx;
extern int particleCount;
extern void updateParticles(int startIdx, int count, float* array_data);
}

namespace {
//Integration methods
void euler(float);
void verlet(float);
//Emitter methods
void fountain(float);
void cascade(float);
///////////////////////////

static struct PhysParams {
	float elastic_coeff = 1.f;
	float friction_coeff = 0.f;

	glm::vec3 sphereC = glm::vec3(0.f, 1.f, 0.f);
	float sphereR = 1.f;

	glm::vec3 capsuleP1 = glm::vec3(-3.f, 2.f, -2.f);
	glm::vec3 capsuleP2 = glm::vec3(-4.f, 2.f, 2.f);
	float capsuleR = 1.f;

	bool useSphere = true;
	bool useCapsule = true;
	bool play_Sim = false;

	enum INT_MTHD { EULER, VERLET };
	int integrationMethod = EULER;
	void(*int_func)(float) = euler;

	enum EMIT_MTHD { FOUNTAIN, CASCADE };
	int emitMethod = FOUNTAIN;
	void(*emit_func)(float) = fountain;

	int emitter_rate = 100;
	float particle_life = 1;
	glm::vec3 fountain_pos = glm::vec3(0.f, 3.f, 0.f);
	glm::vec3 fountain_dir = glm::vec3(0.f, 1.f, 0.f);
	float fountain_angle = 15.f;
	glm::vec3 cascade_posA = glm::vec3(4.f, 4.f, 4.f);
	glm::vec3 cascade_posB = glm::vec3(4.f, 4.f, -4.f);
	glm::vec3 cascade_vel = glm::vec3(-1.f, 0.f, 0.f);

	bool useGravity = true;
	glm::vec3 gravity = glm::vec3(0.f, -9.81f, 0.f);
	bool useForceField = false;
	enum FORCEFIELD { ATTRACT, REPULSE, INTERM };
	int forcef = ATTRACT;
	glm::vec3 forcefield_pos = glm::vec3(0.f, 5.f, 0.f);
	float forcefield_mult = 1.f;
	float forcefield_mult2 = 1.f;
	float accum_dt = 0.f;
} p_pars;

static struct ParticleSystem {
	glm::vec3 *position;
	glm::vec3 *otherval; //velocity (for Euler) or previous pos (for Verlet)
	float *particlelife;
	//float m; //optional mass
	int numParticles;
	int firstParticleAlive = 0, countAlive = 0;
} s_PS;

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
	struct CAP {
		glm::vec3 pA;
		glm::vec3 pB;
		float r;
		CAP(glm::vec3 a, glm::vec3 b, float _r) : pA(a), pB(b), r(_r) {};
	};
	enum prim { PLANE, SPHERE, CAPSULE };
	prim type;
	char data[sizeof(CAP)];
};

static prim_data collider_primitives[8];

bool collidesPlane(const prim_data::PLN& plane, const glm::vec3& prev_pos, glm::vec3& new_pos) {
	float d_next = glm::dot(plane.n, new_pos) + plane.d;
	if(d_next < 0.f) {
		return true;
	}
	return false;
}
void computeCollisionPlane(const prim_data::PLN& plane, const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3& new_other_val) {
	new_pos = new_pos - plane.n * ((glm::dot(plane.n, new_pos) + plane.d) * (1.f + p_pars.elastic_coeff));
	switch(p_pars.integrationMethod) {
	case PhysParams::EULER:
		new_other_val = new_other_val - plane.n * (glm::dot(plane.n, new_other_val) * (1.f + p_pars.elastic_coeff));
		new_other_val = new_other_val - (new_other_val - (plane.n * glm::dot(new_other_val, plane.n))) * p_pars.friction_coeff;
		break;
	case PhysParams::VERLET:
	{
		new_other_val = prev_pos - plane.n * ((glm::dot(plane.n, prev_pos) + plane.d) * (1.f + p_pars.elastic_coeff));
		float t = 1.f / 30.f;
		glm::vec3 vel = (new_pos - new_other_val) / t;
		new_other_val = new_other_val + (vel - plane.n * glm::dot(vel, plane.n)) * (p_pars.friction_coeff * t);
	}
	break;
	}
}
bool collidesCapsule(const prim_data::CAP& capsule, const glm::vec3& prev_pos, glm::vec3& new_pos) {
	glm::vec3 xa = new_pos - capsule.pA;
	glm::vec3 ba = capsule.pB - capsule.pA;
	float length_ba = glm::length(ba);
	glm::vec3 vec_cap_new = capsule.pA + ba * glm::clamp(glm::dot(xa, ba) / (length_ba*length_ba), 0.f, 1.f) - new_pos;
	float d_new = sqrt(glm::dot(vec_cap_new, vec_cap_new)) - capsule.r;
	if(d_new < 0.f)
		return true;
	return false;
}
void computeCollisionCapsule(const prim_data::CAP& capsule, const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3& new_other_val) {
	glm::vec3 ba = capsule.pB - capsule.pA;
	float length_ba = glm::length(ba);

	glm::vec3 xa_p = prev_pos - capsule.pA;
	glm::vec3 vec_cap_prev = capsule.pA + ba * glm::clamp(glm::dot(xa_p, ba) / (length_ba*length_ba), 0.f, 1.f) - prev_pos;
	float d_prev = glm::length(vec_cap_prev) - capsule.r;

	glm::vec3 xa_n = new_pos - capsule.pA;
	glm::vec3 vec_cap_new = capsule.pA + ba * glm::clamp(glm::dot(xa_n, ba) / (length_ba*length_ba), 0.f, 1.f) - new_pos;
	float d_new = glm::length(vec_cap_new) - capsule.r;

	glm::vec3 onCap = new_pos + (prev_pos - new_pos) * -d_new / (d_prev - d_new);
	glm::vec3 pln_n = glm::normalize(onCap - (capsule.pA + ba * glm::clamp(glm::dot(onCap - capsule.pA, ba) / (length_ba*length_ba), 0.f, 1.f)));
	float pln_d = -glm::dot(pln_n, onCap);
	computeCollisionPlane(prim_data::PLN(pln_n, pln_d), prev_pos, new_pos, new_other_val);
}
bool collidesSphere(const prim_data::SPH& sphere, const glm::vec3& prev_pos, glm::vec3& new_pos) {
	glm::vec3 vec2New = sphere.c - new_pos;
	float d_newSq = glm::dot(vec2New, vec2New) - sphere.r*sphere.r;
	if(d_newSq < 0.f)
		return true;
	return false;
}
void computeCollisionSphere(const prim_data::SPH& sphere, const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3& new_other_val) {
	glm::vec3 vec_cap_prev = sphere.c - prev_pos;
	float d_prev = glm::length(vec_cap_prev) - sphere.r;

	glm::vec3 vec_cap_new = sphere.c - new_pos;
	float d_new = glm::length(vec_cap_new) - sphere.r;

	glm::vec3 onSph = new_pos + (prev_pos - new_pos) * -d_new / (d_prev - d_new);
	glm::vec3 pln_n = glm::normalize(onSph - sphere.c);
	float pln_d = -glm::dot(pln_n, onSph);
	computeCollisionPlane(prim_data::PLN(pln_n, pln_d), prev_pos, new_pos, new_other_val);
}

void computeCollision(const glm::vec3& prev_pos, glm::vec3& new_pos, glm::vec3 &other_val) {
	bool hasCollided = true;
	for(const auto& prim : collider_primitives) {
		switch(prim.type) {
		case prim_data::PLANE:
			if(collidesPlane(*(prim_data::PLN*)&prim.data, prev_pos, new_pos))
				computeCollisionPlane(*(prim_data::PLN*)&prim.data, prev_pos, new_pos, other_val);
			break;
		case prim_data::SPHERE:
			if(p_pars.useSphere) {
				if(collidesSphere(*(prim_data::SPH*)&prim.data, prev_pos, new_pos))
					computeCollisionSphere(*(prim_data::SPH*)&prim.data, prev_pos, new_pos, other_val);
			} else continue;
			break;
		case prim_data::CAPSULE:
			if(p_pars.useCapsule) {
				if(collidesCapsule(*(prim_data::CAP*)&prim.data, prev_pos, new_pos))
					computeCollisionCapsule(*(prim_data::CAP*)&prim.data, prev_pos, new_pos, other_val);
			} else continue;
			break;
		}
	}
}

glm::vec3 calcforces(float dt, const glm::vec3& pos) {
	glm::vec3 force(0.f, 0.f, 0.f);
	if(p_pars.useGravity)
		force += p_pars.gravity;

	if(p_pars.useForceField) {
		glm::vec3 v = pos - p_pars.forcefield_pos;
		float length_v = glm::length(v);
		switch(p_pars.forcef) {
		case PhysParams::ATTRACT:
			force -= glm::normalize(v) * (p_pars.forcefield_mult / (length_v));
			break;
		case PhysParams::REPULSE:
			force += glm::normalize(v) * (p_pars.forcefield_mult / (length_v));
			break;
		case PhysParams::INTERM:
			force += glm::normalize(v) * ((p_pars.forcefield_mult / (length_v)) * cos(p_pars.forcefield_mult2 * p_pars.accum_dt));
			break;
		}
	}
	return force;
}

inline int euler_range(float dt, int startidx, int count) {
	glm::vec3 force; glm::vec3 prev_pos_tmp;
	int aliveCount = count;
	for(int i = startidx; i < startidx + count; ++i) {
		s_PS.particlelife[i] -= dt;
		if(s_PS.particlelife[i] < 0.f) {
			--aliveCount;
			continue;
		}
		prev_pos_tmp = s_PS.position[i];
		force = calcforces(dt, s_PS.position[i]);
		s_PS.position[i] = s_PS.position[i] + s_PS.otherval[i] * dt;
		s_PS.otherval[i] = s_PS.otherval[i] + force * dt;
		computeCollision(prev_pos_tmp, s_PS.position[i], s_PS.otherval[i]);
	}
	return aliveCount;
}
void euler(float dt) {
	int remainingAlive = 0;
	if(s_PS.firstParticleAlive + s_PS.countAlive < s_PS.numParticles) {
		remainingAlive = euler_range(dt, s_PS.firstParticleAlive, s_PS.countAlive);
	} else {
		int rem = s_PS.numParticles - s_PS.firstParticleAlive;
		remainingAlive += euler_range(dt, s_PS.firstParticleAlive, rem);
		remainingAlive += euler_range(dt, 0, s_PS.countAlive - rem);
	}
	s_PS.firstParticleAlive = (s_PS.firstParticleAlive + (s_PS.countAlive - remainingAlive)) % s_PS.numParticles;
	s_PS.countAlive = remainingAlive;
}

#pragma region VERLET
inline int verlet_range(float dt, int startidx, int count) {
	glm::vec3 force; glm::vec3 pos_tmp;
	int aliveCount = count;
	for(int i = startidx; i < startidx + count; ++i) {
		s_PS.particlelife[i] -= dt;
		if(s_PS.particlelife[i] < 0) {
			--aliveCount;
			continue;
		}
		force = calcforces(dt, s_PS.position[i]);
		pos_tmp = s_PS.position[i];
		s_PS.position[i] = s_PS.position[i] * 2.f - s_PS.otherval[i] + force*(dt*dt);
		s_PS.otherval[i] = pos_tmp;
		computeCollision(s_PS.otherval[i], s_PS.position[i], s_PS.otherval[i]);
	}
	return aliveCount;
}
void verlet(float dt) {
	int remainingAlive = 0;
	if(s_PS.firstParticleAlive + s_PS.countAlive < s_PS.numParticles) {
		remainingAlive = verlet_range(dt, s_PS.firstParticleAlive, s_PS.countAlive);
	} else {
		int rem = s_PS.numParticles - s_PS.firstParticleAlive;
		remainingAlive += verlet_range(dt, s_PS.firstParticleAlive, rem);
		remainingAlive += verlet_range(dt, 0, s_PS.countAlive - rem);
	}
	s_PS.firstParticleAlive = (s_PS.firstParticleAlive + (s_PS.countAlive - remainingAlive)) % s_PS.numParticles;
	s_PS.countAlive = remainingAlive;
}
#pragma endregion

inline void initParticleFountain(int startidx, int count) {
	float init_speed = glm::length(p_pars.fountain_dir);
	glm::vec3 norm_dir = p_pars.fountain_dir / init_speed;
	glm::vec3 tangent(1.f, 0.f, 0.f);
	if(1.f - abs(glm::dot(norm_dir, tangent)) < 1e-3) {
		tangent = glm::vec3(0.f, 0.f, 1.f);
	}
	glm::vec3 bitangent = glm::normalize(glm::cross(norm_dir, tangent));
	tangent = glm::normalize(glm::cross(bitangent, norm_dir));

	glm::vec3 t2, bt2;
	for(int i = startidx; i < startidx + count; ++i) {
		s_PS.position[i] = p_pars.fountain_pos;
		float angleA = ((float)rand() / RAND_MAX) * glm::radians(p_pars.fountain_angle);
		float angleB = ((float)rand() / RAND_MAX) * glm::two_pi<float>();
		float sinA = sin(angleA), cosA = cos(angleA);
		float sinB = sin(angleB), cosB = cos(angleB);
		t2 = tangent * sinA * cosB;
		bt2 = bitangent * sinA * sinB;
		switch(p_pars.integrationMethod) {
		case PhysParams::EULER:
			s_PS.otherval[i] = glm::normalize(norm_dir * cosA + t2 + bt2) * init_speed;
			break;
		case PhysParams::VERLET:
			s_PS.otherval[i] = s_PS.position[i] - (glm::normalize(norm_dir * cosA + t2 + bt2) * (init_speed / 30.f)); //We have a forced dt = 1/30
			break;
		}
		s_PS.particlelife[i] = p_pars.particle_life;
	}
}
void fountain(float dt) {
	int newparts = (int)floor(p_pars.emitter_rate*dt);
	int lastPart = (s_PS.firstParticleAlive + s_PS.countAlive) % s_PS.numParticles;
	if(lastPart + newparts < s_PS.numParticles) {
		initParticleFountain(lastPart, newparts);
	} else {
		int rem = s_PS.numParticles - s_PS.firstParticleAlive - s_PS.countAlive;
		initParticleFountain(s_PS.firstParticleAlive + s_PS.countAlive, rem);
		initParticleFountain(0, newparts - rem);
	}
	s_PS.countAlive += newparts;
}
inline void initParticleCascade(int startidx, int count) {
	for(int i = startidx; i < startidx + count; ++i) {
		s_PS.position[i] = p_pars.cascade_posA + (p_pars.cascade_posB - p_pars.cascade_posA) * ((float)rand() / RAND_MAX);
		switch(p_pars.integrationMethod) {
		case PhysParams::EULER:
			s_PS.otherval[i] = p_pars.cascade_vel;//glm::vec3(0.f, 0.f, 0.f);
			break;
		case PhysParams::VERLET:
			s_PS.otherval[i] = s_PS.position[i] - p_pars.cascade_vel / 30.f;
			break;
		}
		s_PS.particlelife[i] = p_pars.particle_life;
	}
}
void cascade(float dt) {
	int newparts = (int)floor(p_pars.emitter_rate*dt);
	int lastPart = (s_PS.firstParticleAlive + s_PS.countAlive) % s_PS.numParticles;
	if(lastPart + newparts < s_PS.numParticles) {
		initParticleCascade(lastPart, newparts);
	} else {
		int rem = s_PS.numParticles - s_PS.firstParticleAlive - s_PS.countAlive;
		initParticleCascade(s_PS.firstParticleAlive + s_PS.countAlive, rem);
		initParticleCascade(0, newparts - rem);
	}
	s_PS.countAlive += newparts;
}
}

void P1_GUI() {
	ImGui::Checkbox("Play simulation", &p_pars.play_Sim);
	if(ImGui::Button("Reset simulation")) {
		s_PS.firstParticleAlive = 0;
		s_PS.countAlive = 0;
	}

	if(ImGui::TreeNode("Emitter")) {
		ImGui::DragInt("Emitter rate", &p_pars.emitter_rate, 10, 100, 1000);
		ImGui::DragFloat("Particle life", &p_pars.particle_life, 0.1f, 1.f, 5.f);

		ImGui::RadioButton("Fountain", &p_pars.emitMethod, PhysParams::FOUNTAIN); ImGui::SameLine();
		ImGui::RadioButton("Cascade", &p_pars.emitMethod, PhysParams::CASCADE);

		switch(p_pars.emitMethod) {
		case PhysParams::FOUNTAIN:
			p_pars.emit_func = fountain;
			ImGui::DragFloat3("Fountain pos", &p_pars.fountain_pos.x, 0.01f, -5.f, 10.f);
			ImGui::DragFloat3("Fountain dir", &p_pars.fountain_dir.x, 0.01f, -5.f, 10.f);
			ImGui::DragFloat("Fountain angle", &p_pars.fountain_angle, 0.01f, 0.f, 90.f);
			p_pars.emit_func = fountain;
			break;
		case PhysParams::CASCADE:
			ImGui::DragFloat3("Cascade posA", &p_pars.cascade_posA.x, 0.01f, -5.f, 10.f);
			ImGui::DragFloat3("Cascade posB", &p_pars.cascade_posB.x, 0.01f, -5.f, 10.f);
			ImGui::DragFloat3("Cascade velocity", &p_pars.cascade_vel.x, 0.01f, -5.f, 5.f);
			p_pars.emit_func = cascade;
			break;
		default:
			fprintf(stderr, "Emitter type not found\n");
			break;
		}

		ImGui::TreePop();
	}

#ifdef INTEGRATION
	if(ImGui::TreeNode("Integration")) {
		float lastdt = 1.f / 30.f; //We have a force dt = 1/30
		bool changed = false;
		changed |= ImGui::RadioButton("Euler", &p_pars.integrationMethod, PhysParams::EULER);
		changed |= ImGui::RadioButton("Verlet", &p_pars.integrationMethod, PhysParams::VERLET);

		if(changed) {
			switch(p_pars.integrationMethod) {
			case PhysParams::EULER:
				p_pars.int_func = euler;
				s_PS.firstParticleAlive = 0;
				s_PS.countAlive = 0;
				break;
			case PhysParams::VERLET:
				p_pars.int_func = verlet;
				s_PS.firstParticleAlive = 0;
				s_PS.countAlive = 0;
				break;
			default:
				fprintf(stderr, "Integration Method not found\n");
				break;
			}
		}
		ImGui::TreePop();
	}
#endif

	if(ImGui::TreeNode("Elasticity & Friction")) {
		ImGui::DragFloat("Elastic Coefficient", &p_pars.elastic_coeff, 0.001f, 0.f, 1.f);
		ImGui::DragFloat("Friction Coefficient", &p_pars.friction_coeff, 0.001f, 0.f, 1.f);
		ImGui::TreePop();
	}

	if(ImGui::TreeNode("Colliders")) {
		ImGui::Checkbox("Use Sphere Collider", &p_pars.useSphere);
		extern bool renderSphere; renderSphere = p_pars.useSphere;
		if(p_pars.useSphere) {
			bool isChanged = false;
			isChanged |= ImGui::DragFloat3("Sphere Position", &p_pars.sphereC.x, 0.01f, -5.f, 10.f);
			isChanged |= ImGui::DragFloat("Sphere Radius", &p_pars.sphereR, 0.01f, 0.1f, 5.f);
			if(isChanged) {
				prim_data::SPH* sph = (prim_data::SPH*) &collider_primitives[6].data;
				sph->c = p_pars.sphereC;
				sph->r = p_pars.sphereR;
				Sphere::updateSphere(p_pars.sphereC, p_pars.sphereR);
			}
		}

		ImGui::Checkbox("Use Capsule Collider", &p_pars.useCapsule);
		extern bool renderCapsule; renderCapsule = p_pars.useCapsule;
		if(p_pars.useCapsule) {
			bool isChanged = false;
			isChanged |= ImGui::DragFloat3("Capsule Pos A", &p_pars.capsuleP1.x, 0.01f, -5.f, 10.f);
			isChanged |= ImGui::DragFloat3("Capsule Pos B", &p_pars.capsuleP2.x, 0.01f, -5.f, 10.f);
			isChanged |= ImGui::DragFloat("Capsule Radius", &p_pars.capsuleR, 0.01f, 0.1f, 5.f);
			if(isChanged) {
				prim_data::CAP* cap = (prim_data::CAP*) &collider_primitives[7].data;
				cap->pA = p_pars.capsuleP1;
				cap->pB = p_pars.capsuleP2;
				cap->r = p_pars.capsuleR;
				Capsule::updateCapsule(p_pars.capsuleP1, p_pars.capsuleP2, p_pars.capsuleR);
			}
		}
		ImGui::TreePop();
	}

	if(ImGui::TreeNode("Forces")) {
		ImGui::Checkbox("Use gravity", &p_pars.useGravity);
		if(p_pars.useGravity) {
			ImGui::DragFloat3("Gravity Accel", &p_pars.gravity.x, 0.001f, -10.f, 10.f);
		}
#ifdef EXT_FORCES
		ImGui::Checkbox("Use Force Field", &p_pars.useForceField);
		if(p_pars.useForceField) {
			ImGui::RadioButton("Attraction", &p_pars.forcef, PhysParams::ATTRACT);
			ImGui::RadioButton("Repulsion", &p_pars.forcef, PhysParams::REPULSE);
			ImGui::RadioButton("Intermittence", &p_pars.forcef, PhysParams::INTERM);
			ImGui::DragFloat3("ForceField Pos", &p_pars.forcefield_pos.x, 0.001f, -5.f, 10.f);
			ImGui::DragFloat("ForceField Mult", &p_pars.forcefield_mult, 0.001f, 0.f, 5.f);
			if(p_pars.forcef == PhysParams::INTERM) {
				ImGui::DragFloat("ForceField Time Mult", &p_pars.forcefield_mult2, 0.001f, 0.f, 5.f);
			}
		}
#endif
		ImGui::TreePop();
	}
}

void P1_PhysicsInit() {
	s_PS.numParticles = LilSpheres::maxParticles;
	s_PS.position = new glm::vec3[s_PS.numParticles];
	s_PS.otherval = new glm::vec3[s_PS.numParticles];
	s_PS.particlelife = new float[s_PS.numParticles];

	collider_primitives[0].type = prim_data::PLANE; prim_data::PLN* pln = ((prim_data::PLN*)&collider_primitives[0].data); pln->n = glm::vec3(1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[1].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[1].data); pln->n = glm::vec3(-1.f, 0.f, 0.f); pln->d = 5.f;
	collider_primitives[2].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[2].data); pln->n = glm::vec3(0.f, 1.f, 0.f); pln->d = 0.f;
	collider_primitives[3].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[3].data); pln->n = glm::vec3(0.f, -1.f, 0.f); pln->d = 10.f;
	collider_primitives[4].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[4].data); pln->n = glm::vec3(0.f, 0.f, 1.f); pln->d = 5.f;
	collider_primitives[5].type = prim_data::PLANE; pln = ((prim_data::PLN*)&collider_primitives[5].data); pln->n = glm::vec3(0.f, 0.f, -1.f); pln->d = 5.f;
	collider_primitives[6].type = prim_data::SPHERE; prim_data::SPH* sph = ((prim_data::SPH*)&collider_primitives[6].data); sph->c = p_pars.sphereC; sph->r = p_pars.sphereR;
	collider_primitives[7].type = prim_data::CAPSULE; prim_data::CAP* cap = ((prim_data::CAP*)&collider_primitives[7].data); cap->pA = p_pars.capsuleP1; cap->pB = p_pars.capsuleP2; cap->r = p_pars.capsuleR;

	extern bool renderParticles; renderParticles = true;
	extern bool renderCapsule; renderCapsule = p_pars.useCapsule;
	extern bool renderSphere; renderSphere = p_pars.useSphere;
}

void P1_PhysicsUpdate(float dt) {
	if(p_pars.play_Sim) {
		p_pars.accum_dt += dt;
		if(p_pars.accum_dt > 10.f*glm::two_pi<float>())
			p_pars.accum_dt -= 10.f*glm::two_pi<float>();
		p_pars.int_func(dt);
		p_pars.emit_func(dt);

		if(s_PS.firstParticleAlive + s_PS.countAlive < s_PS.numParticles) {
			LilSpheres::updateParticles(s_PS.firstParticleAlive, s_PS.countAlive, &(s_PS.position[s_PS.firstParticleAlive].x));
		} else {
			int rem = s_PS.numParticles - s_PS.firstParticleAlive;
			LilSpheres::updateParticles(s_PS.firstParticleAlive, rem, &(s_PS.position[s_PS.firstParticleAlive].x));
			LilSpheres::updateParticles(0, s_PS.countAlive - rem, &(s_PS.position[0].x));
		}
	}
	LilSpheres::firstParticleIdx = s_PS.firstParticleAlive;
	LilSpheres::particleCount = s_PS.countAlive;
}

void P1_PhysicsCleanup() {
	delete[] s_PS.position;
	delete[] s_PS.otherval;
	delete[] s_PS.particlelife;
}

