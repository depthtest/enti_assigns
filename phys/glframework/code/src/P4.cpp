#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm\glm.hpp>
#include <glm\gtc\quaternion.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <tuple>

/////////Forward declarations
namespace Sphere {
extern void updateSphere(glm::vec3 pos, float radius = 1.f);
}
namespace ClothMesh {
extern const int numCols;
extern const int numRows;
extern const int numVerts;
extern void updateClothMesh(float* array_data);
}

namespace {

static struct PhysParams {
	glm::vec3 gravity = glm::vec3(0.f, -9.81f, 0.f);
	bool play_Sim = false;

	float accum_dt = 0.f;
	float timeToReset = 15.f;
} p_pars;

static struct FluidSystem {
	glm::vec3 *u_p;

	glm::vec3 sphere_prevpos = glm::vec3(0.f, 0.f, 0.f);
	glm::vec3 sphere_pos = glm::vec3(0.f, 0.f, 0.f);
	float sphere_rad = 1.f;
	float sphere_gui_mass = 1.f;
	float sphere_mass = 1.f;

	//Gerstner params
#define GER_NW 2
	float ampl[GER_NW] = {0.5f, 0.25f};
	float freq[GER_NW] = {2.f, 4.f};
	glm::vec3 wavev[GER_NW] = {glm::vec3(1.f, 0.f, 0.f), glm::vec3(0.f, 0.f, -3.f)};
	float wavem[GER_NW] = {1.f, -3.f};
} s_FS;

glm::vec3 getInitPos(int i, int j) {
	float wCell = 10.f / (ClothMesh::numCols - 1);
	float hCell = 10.f / (ClothMesh::numRows - 1);
	return glm::vec3(
		-5.f + i*hCell,
		3.f,
		-5.f + j*wCell);
}

glm::vec3 getGerstnerPos(int i, int j) {
	glm::vec3 prevX = getInitPos(i, j);
	float prevY = prevX.y; prevX.y = 0.f;
	glm::vec3 toret = prevX;
	for(int w = 0; w < GER_NW; ++w) {
		toret += s_FS.wavev[w] / s_FS.wavem[w] * s_FS.ampl[w] * sin(glm::dot(s_FS.wavev[w], prevX) - s_FS.freq[w] * p_pars.accum_dt);
		toret.y += s_FS.ampl[w] * cos(glm::dot(s_FS.wavev[w], prevX) - s_FS.freq[w] * p_pars.accum_dt);
	}
	toret.y += prevY;

	return toret;
}

std::tuple<int, int, int, int, float, float> get_ij_min_max_fromPos(const glm::vec3& pos) {
	float wCell = 10.f / (ClothMesh::numCols - 1);
	float hCell = 10.f / (ClothMesh::numRows - 1);

	int i_min = (int)floor(((pos.x + 5.f) / 10.f) * ClothMesh::numRows);
	int j_min = (int)floor(((pos.z + 5.f) / 10.f) * ClothMesh::numCols);

	int i_max = (int)ceil(((pos.x + 5.f) / 10.f) * ClothMesh::numRows);
	int j_max = (int)ceil(((pos.z + 5.f) / 10.f) * ClothMesh::numCols);

	float alpha_x = (pos.x + 5.f - i_min * hCell) / hCell;
	float alpha_z = (pos.z + 5.f - j_min * wCell) / wCell;

	return std::make_tuple(i_min, j_min, i_max, j_max, alpha_x, alpha_z);
}

void initializeScene() {
	float wCell = 10.f / (ClothMesh::numCols - 1);
	float hCell = 10.f / (ClothMesh::numRows - 1);

	for(int i = 0; i < ClothMesh::numRows; ++i) {
		for(int j = 0; j < ClothMesh::numCols; ++j) {
			int idx = i*ClothMesh::numCols + j;

			s_FS.u_p[idx] = getInitPos(i, j);
			s_FS.u_p[idx].y = getGerstnerPos(i, j).y;
		}
	}

	ClothMesh::updateClothMesh(&s_FS.u_p[0].x);

	s_FS.sphere_pos = s_FS.sphere_prevpos = glm::vec3(
		((float)rand() / RAND_MAX) * 8.f - 4.f,
		((float)rand() / RAND_MAX) * 2.f + 4.f,
		((float)rand() / RAND_MAX) * 8.f - 4.f
	);
	Sphere::updateSphere(s_FS.sphere_pos, s_FS.sphere_rad);
}

void resetScene() {
	initializeScene();
	p_pars.accum_dt = 0.f;
}

}

void P4_GUI() {
	ImGui::Checkbox("Play simulation", &p_pars.play_Sim);
	if(ImGui::Button("Reset")) {
		initializeScene();
		p_pars.accum_dt = 0.f;
	}

	ImGui::DragFloat("Reset time", &p_pars.timeToReset, 0.001f, 2.f, 120.f);
	ImGui::DragFloat3("Gravity Accel", &p_pars.gravity.x, 0.001f, -10.f, 10.f);

	ImGui::DragFloat("Sphere Mass", &s_FS.sphere_gui_mass, 0.001f, 5.f);
	if(s_FS.sphere_gui_mass < 1e-9) {
		s_FS.sphere_mass = 1.f;
		s_FS.sphere_gui_mass = 1.f;
	} else {
		s_FS.sphere_mass = s_FS.sphere_gui_mass;
	}
}

void P4_PhysicsInit() {
	s_FS.u_p = new glm::vec3[ClothMesh::numVerts];

	extern bool renderCloth; renderCloth = true;
	extern bool renderSphere; renderSphere = true;

	initializeScene();
}

void P4_PhysicsUpdate(float dt) {
	if(p_pars.play_Sim) {
		p_pars.accum_dt += dt;
		if(p_pars.accum_dt > p_pars.timeToReset) {
			resetScene();
			return;
		}

		//Sim Fluid
		for(int i = 0; i < ClothMesh::numRows; ++i) {
			for(int j = 0; j < ClothMesh::numCols; ++j) {
				int idx = i*ClothMesh::numCols + j;
				s_FS.u_p[idx] = getGerstnerPos(i, j);
			}
		}

		//Sim Sphere
		glm::vec3 force = p_pars.gravity * s_FS.sphere_mass;
		auto ij = get_ij_min_max_fromPos(s_FS.sphere_pos); //Will not be terribly correct, not accounting for horizontal displacement

		int idxA = std::get<0>(ij)*ClothMesh::numCols + std::get<1>(ij);
		int idxB = std::get<2>(ij)*ClothMesh::numCols + std::get<1>(ij);
		int idxC = std::get<0>(ij)*ClothMesh::numCols + std::get<3>(ij);
		int idxD = std::get<2>(ij)*ClothMesh::numCols + std::get<3>(ij);

		float real_y = glm::mix(
			glm::mix(s_FS.u_p[idxA].y, s_FS.u_p[idxB].y, std::get<4>(ij)),
			glm::mix(s_FS.u_p[idxC].y, s_FS.u_p[idxD].y, std::get<4>(ij)),
			std::get<5>(ij)
		);

		float vSub = 0.f; //compute submerged volume - just as box, don't want to compute an integral now...
		float hsub = -1.f;
		if(real_y < s_FS.sphere_pos.y) {
			hsub = s_FS.sphere_pos.y - real_y - s_FS.sphere_rad;
			if(hsub < 0.f) {
				vSub = -hsub * 4.f*s_FS.sphere_rad*s_FS.sphere_rad;
				force -= (s_FS.sphere_pos - s_FS.sphere_prevpos) / dt * 10.f * 0.5f * 2.f; //Accounting for DRAG
			}
		} else {
			hsub = glm::min(real_y - s_FS.sphere_pos.y, s_FS.sphere_rad) + s_FS.sphere_rad;
			vSub = hsub * 4.f*s_FS.sphere_rad*s_FS.sphere_rad;
			force -= (s_FS.sphere_pos - s_FS.sphere_prevpos) / dt * 10.f * 0.5f * 2.f; //Accounting for DRAG
		}
		force += glm::vec3(0.f, 1.f, 0.f) * (-p_pars.gravity.y) * vSub * 10.f; //fluid density

		glm::vec3 actpos = s_FS.sphere_pos;
		s_FS.sphere_pos = s_FS.sphere_pos *2.f - s_FS.sphere_prevpos + force / s_FS.sphere_mass * dt*dt;
		s_FS.sphere_prevpos = actpos;

		ClothMesh::updateClothMesh(&s_FS.u_p[0].x);
		Sphere::updateSphere(s_FS.sphere_pos);
	}
}

void P4_PhysicsCleanup() {
	delete[] s_FS.u_p;
}

