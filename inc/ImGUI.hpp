#ifndef IMGUI_HPP
#define IMGUI_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "Shader.hpp"
// #include "../src/Gravitas.hpp"

class ImGUI {

public:
	bool darkMode = false;

	void Init(GLFWwindow *window, const char *glsl_version) {
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();

		// Setup platform-specific bindings
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		// Setup imgui dark mode
		ImGui::StyleColorsDark();

	}

	void NewFrame() {
		// Feed inputs to imgui, start new frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
	}

	void SetDarkMode(Shader grid) {
		if (darkMode) {
			grid.use();

			grid.setVec4("gGridColorThin", glm::vec4(1.0, 1.0, 1.0, 1.0));
			grid.setVec4("gGridColorThick", glm::vec4(1.0, 1.0, 1.0, 1.0));
		}
	}


	// A custom toggle switch widget.
// Use as: ToggleButton("Toggle", &toggle_state);
	void ToggleButton(const char* str_id, bool* v)
	{
		// Get the current cursor position on screen.
		ImVec2 p = ImGui::GetCursorScreenPos();
		// Get the window's DrawList so we can issue custom draw commands.
		ImDrawList* draw_list = ImGui::GetWindowDrawList();

		// Determine the size of the toggle based on the frame height.
		float height = ImGui::GetFrameHeight();       // Use the default frame height.
		float width = height * 1.55f;                  // Width factor to make it look like a switch.
		float radius = height * 0.50f;                  // Radius for the circle knob.

		// Reserve a space for the invisible button.
		if (ImGui::InvisibleButton(str_id, ImVec2(width, height)))
			*v = !*v;  // Toggle the value on click.

		// Choose the background color based on the state and whether the widget is hovered.
		ImU32 col_bg;
		if (ImGui::IsItemHovered())
		{
			// Slightly brighter/darker versions when hovered:
			col_bg = *v ? IM_COL32(165, 231, 88, 255) : IM_COL32(198, 198, 198, 255);
		}
		else
		{
			// Normal colors for each state.
			col_bg = *v ? IM_COL32(145, 211, 68, 255) : IM_COL32(218, 218, 218, 255);
		}

		// Draw the rounded rectangle as the toggle's background.
		draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);

		// Calculate knob position: if toggle is ON, position it at the right, otherwise left.
		float knob_x = *v ? (p.x + width - radius) : (p.x + radius);
		// Draw the knob as a filled circle.
		draw_list->AddCircleFilled(ImVec2(knob_x, p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));

		// Optionally, you can add a label next to the toggle by calling ImGui::SameLine() and ImGui::Text()
		// if you wish to display text.
	}


	virtual void Update() {

		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

		ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)


		ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f

		static float clear_color[3];
		ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

		if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
			counter++;
		ImGui::SameLine();
		ImGui::Text("counter = %d", counter);

		ToggleButton("Dark Mode", &darkMode);

		// Optionally display the state.
		ImGui::SameLine();
		ImGui::Text(darkMode ? "ON" : "OFF");

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		/*ImGui::Checkbox("Show Debug Visuals", &);
		ImGui::SliderFloat("Drag Sensitivity", &dragSensitivity, 0.1f, 5.0f);

		if (isDragging && selected >= 0) {
			ImGui::Text("Selected Object: %d", selected);
			ImGui::Text("Original Pos: %.2f, %.2f, %.2f",
				initialObjectPos.x, initialObjectPos.y, initialObjectPos.z);
		}*/

		ImGui::End();
	}

	void Render() {
		// Render GUI
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	}

	void Shutdown() {
		// Cleanup
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();

	}
};

#endif
