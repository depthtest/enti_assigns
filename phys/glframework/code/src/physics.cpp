#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>

//P1
extern void P1_GUI();
extern void P1_PhysicsInit();
extern void P1_PhysicsUpdate(float dt);
extern void P1_PhysicsCleanup();

//P2
extern void P2_GUI();
extern void P2_PhysicsInit();
extern void P2_PhysicsUpdate(float dt);
extern void P2_PhysicsCleanup();

//P3
extern void P3_GUI();
extern void P3_PhysicsInit();
extern void P3_PhysicsUpdate(float dt);
extern void P3_PhysicsCleanup();

//P4
extern void P4_GUI();
extern void P4_PhysicsInit();
extern void P4_PhysicsUpdate(float dt);
extern void P4_PhysicsCleanup();

bool show_test_window = false;
void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		P1_GUI();
		//P2_GUI();
		//P3_GUI();
		//P4_GUI();
	}
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void PhysicsInit() {
	P1_PhysicsInit();
	//P2_PhysicsInit();
	//P3_PhysicsInit();
	//P4_PhysicsInit();
}

void PhysicsUpdate(float dt) {
	P1_PhysicsUpdate(dt);
	//P2_PhysicsUpdate(dt);
	//P3_PhysicsUpdate(dt);
	//P4_PhysicsUpdate(dt);
}

void PhysicsCleanup() {
	P1_PhysicsCleanup();
	//P2_PhysicsCleanup();
	//P3_PhysicsCleanup();
	//P4_PhysicsCleanup();
}