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

namespace LilSpheres {
	int firstParticleIdx, particleCount;
}


namespace Model {
	extern void setupModel();
	extern void cleanupModel();
	extern void updateModel(const glm::vec4& position, const glm::vec4& color);
	//extern void drawModel();
	extern void drawModelA();
	extern void drawModelB();

	extern void setupModelInstanced(int, int);
	extern void cleanupModelInstanced();
	extern void updateModelInstanced(glm::vec4 *position, glm::vec4 *color);
	extern void drawModelInstanced();

	extern void setupModelMultiDrawInstanced(int, int);
	extern void cleanupModelMultiDrawInstanced();
	extern void drawModelMultiDrawInstanced();
}

enum P5Case {LOOPED, INSTANCED, MULTIDRAW};
int looped = LOOPED;
void rGUI() {
	bool show = true;
	ImGui::Begin("Parameters", &show, 0);

	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate

		ImGui::RadioButton("Looped", &looped, LOOPED);
		ImGui::RadioButton("Instanced", &looped, INSTANCED);
		ImGui::RadioButton("MultiDrawIndirect", &looped, MULTIDRAW);
	}
	
	ImGui::End();
}

#define W_BUN 100
#define H_BUN 100
glm::vec4 init_trans = glm::vec4(-W_BUN*0.5f+0.5f, -H_BUN*0.5f+0.5f, 0.f, 1.f);

void setupPrims() {
	Model::setupModel();
	Model::setupModelInstanced(W_BUN, H_BUN);
	Model::setupModelMultiDrawInstanced(W_BUN, H_BUN);
}
void cleanupPrims() {
	Model::cleanupModel();
	Model::cleanupModelInstanced();
	Model::cleanupModelMultiDrawInstanced();
}

glm::vec4 pos_s[W_BUN*H_BUN];
glm::vec4 col_s[W_BUN*H_BUN];

void renderPrims() {
	static float mov = 0.f;
	mov += 0.05f;
	if(mov > glm::two_pi<float>()){
		mov = 0.f;
	}

	auto comp_pos = [](int i, int j) {
		return init_trans + glm::vec4(i, j, cosf(mov+0.25f*i)+cosf(mov+0.5f*j), 0.f);
	};
	auto comp_colA = [](int i, int j) {
		return glm::vec4(1.f, i/(float)W_BUN, j/(float)H_BUN, 1.f);
	};
	auto comp_colB = [](int i, int j) {
		return glm::vec4(i/(float)W_BUN, j/(float)H_BUN, 1.f, 1.f);
	};

	auto upd_Instanced = [&comp_pos, &comp_colA, &comp_colB]() {
		int xA = 0;
		int xB = W_BUN*H_BUN/2;
		for(int j=0; j<H_BUN; ++j) {
			for(int i=0; i<W_BUN; ++i){
				glm::vec4 nu_pos = comp_pos(i, j);
				if((i-1)&1 && j&1){
					pos_s[xA] = nu_pos;
					col_s[xA] = comp_colA(i, j);
					++xA;
				} else if(i&1 && (j-1)&1) {
					pos_s[xA] = nu_pos;
					col_s[xA] = comp_colA(i, j);
					++xA;
				} else {
					pos_s[xB] = nu_pos;
					col_s[xB] = comp_colB(i, j);
					++xB;
				}
			}
		}
	};

	switch(looped) {
		case LOOPED: {
			//P5 - Loop render
			upd_Instanced();
			int num_Insts = W_BUN*H_BUN;
			for(int cnt=0; cnt<num_Insts; ++cnt) {
				Model::updateModel(pos_s[cnt], col_s[cnt]);
				if(cnt < num_Insts/2) Model::drawModelA();
				else  Model::drawModelB();
			}
		} break;
		case INSTANCED: {
			//P5 - Instanced render
			upd_Instanced();
			Model::updateModelInstanced(pos_s, col_s);
			Model::drawModelInstanced();
		} break;
		case MULTIDRAW: {
			//P5 - MultiDrawIndirect
			upd_Instanced();
			Model::updateModelInstanced(pos_s, col_s);
			Model::drawModelMultiDrawInstanced();
		}
		default:
		;//Do nothing
	}
}
