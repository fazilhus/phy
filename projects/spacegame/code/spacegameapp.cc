//------------------------------------------------------------------------------
// spacegameapp.cc
// (C) 2022 Individual contributors, see AUTHORS file
//------------------------------------------------------------------------------
#include "config.h"
#include "spacegameapp.h"
#include <cstring>
#include "imgui.h"
#include "render/renderdevice.h"
#include "render/shaderresource.h"
#include <vector>
#include "render/textureresource.h"
#include "render/model.h"
#include "render/cameramanager.h"
#include "render/lightserver.h"
#include "render/debugrender.h"
#include "core/random.h"
#include "render/input/inputserver.h"
#include "core/cvar.h"
#include <chrono>
#include "fx/gltf.h"

#include "core/filesystem.h"

using namespace Display;
using namespace Render;

#ifndef ENV_ROOT
#define ENV_ROOT "."
#endif


namespace Game {

    //------------------------------------------------------------------------------
    /**
    */
    SpaceGameApp::SpaceGameApp() {
        // empty
    }

    //------------------------------------------------------------------------------
    /**
    */
    SpaceGameApp::~SpaceGameApp() {
        // empty
        if (camera != nullptr) { delete camera; }
    }

    //------------------------------------------------------------------------------
    /**
    */
    bool SpaceGameApp::Open() {
        App::Open();
        this->window = new Display::Window;
        this->window->SetSize(1920, 1080);

        if (this->window->Open()) {
            // set clear color to gray
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

            RenderDevice::Init();

            // set ui rendering function
            this->window->SetUiRender([this]() { this->RenderUI(); });

            return true;
        }
        return false;
    }

    //------------------------------------------------------------------------------
    /**
    */
    void SpaceGameApp::Run() {
        int w;
        int h;
        this->window->GetSize(w, h);
        glm::mat4 projection = glm::perspective(glm::radians(90.0f), float(w) / float(h), 0.01f, 1000.f);
        Camera* cam = CameraManager::GetCamera(CAMERA_MAIN);
        cam->projection = projection;

        auto cam_pos = glm::vec3(0, 0, 0);
        auto t = glm::mat4(1.0f);
        cam->view = lookAt(cam_pos, cam_pos + glm::vec3(t[2]), glm::vec3(t[1]));

        camera = new Render::DebugCamera(5.0f, 2.5f);

        // load all resources
        ModelId models[6] = {
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_1.glb")),
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_2.glb")),
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_3.glb")),
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_4.glb")),
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_5.glb")),
            LoadModel(fs::create_path_from_rel_s("assets/space/Asteroid_6.glb"))
        };

        std::vector<std::tuple<ModelId, glm::mat4>> asteroids;

        // Setup asteroids near
        for (int i = 0; i < 100; i++) {
            std::tuple<ModelId, glm::mat4> asteroid;
            size_t resourceIndex = (size_t)(Core::FastRandom() % 6);
            std::get<0>(asteroid) = models[resourceIndex];
            float span = 20.0f;
            glm::vec3 translation = glm::vec3(
                Core::RandomFloatNTP() * span,
                Core::RandomFloatNTP() * span,
                Core::RandomFloatNTP() * span
                );
            glm::vec3 rotationAxis = normalize(translation);
            float rotation = translation.x;
            glm::mat4 transform = glm::rotate(rotation, rotationAxis) * glm::translate(translation);
            std::get<1>(asteroid) = transform;
            asteroids.push_back(asteroid);
        }

        // Setup asteroids far
        for (int i = 0; i < 50; i++) {
            std::tuple<ModelId, glm::mat4> asteroid;
            size_t resourceIndex = (size_t)(Core::FastRandom() % 6);
            std::get<0>(asteroid) = models[resourceIndex];
            float span = 80.0f;
            glm::vec3 translation = glm::vec3(
                Core::RandomFloatNTP() * span,
                Core::RandomFloatNTP() * span,
                Core::RandomFloatNTP() * span
                );
            glm::vec3 rotationAxis = normalize(translation);
            float rotation = translation.x;
            glm::mat4 transform = glm::rotate(rotation, rotationAxis) * glm::translate(translation);
            std::get<1>(asteroid) = transform;
            asteroids.push_back(asteroid);
        }

        // Setup skybox
        std::vector<std::string> skybox
        {
            fs::create_path_from_rel_s("assets/space/bg.png"),
            fs::create_path_from_rel_s("assets/space/bg.png"),
            fs::create_path_from_rel_s("assets/space/bg.png"),
            fs::create_path_from_rel_s("assets/space/bg.png"),
            fs::create_path_from_rel_s("assets/space/bg.png"),
            fs::create_path_from_rel_s("assets/space/bg.png")
        };
        TextureResourceId skyboxId = TextureResource::LoadCubemap("skybox", skybox, true);
        RenderDevice::SetSkybox(skyboxId);

        Input::Keyboard* kbd = Input::GetDefaultKeyboard();

        const int numLights = 40;
        Render::PointLightId lights[numLights];
        // Setup lights
        for (int i = 0; i < numLights; i++) {
            glm::vec3 translation = glm::vec3(
                Core::RandomFloatNTP() * 20.0f,
                Core::RandomFloatNTP() * 20.0f,
                Core::RandomFloatNTP() * 20.0f
                );
            glm::vec3 color = glm::vec3(
                Core::RandomFloat(),
                Core::RandomFloat(),
                Core::RandomFloat()
                );
            lights[i] = Render::LightServer::CreatePointLight(
                translation, color, Core::RandomFloat() * 4.0f, 1.0f + (15 + Core::RandomFloat() * 10.0f)
                );
        }

        std::clock_t c_start = std::clock();
        double dt = 0.01667f;

        // game loop
        while (this->window->IsOpen()) {
            auto timeStart = std::chrono::steady_clock::now();
            glClear(GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);

            this->window->Update();
            this->camera->Update(dt);

            if (kbd->pressed[Input::Key::Code::End]) { ShaderResource::ReloadShaders(); }

            Debug::DrawGrid();

            Debug::DrawQuad(glm::vec3(0, 0, 0), glm::quat(), 1.0f, glm::vec4(1,0,0,1));

            // Store all drawcalls in the render device
            for (auto const& asteroid: asteroids) { RenderDevice::Draw(std::get<0>(asteroid), std::get<1>(asteroid)); }

            // Execute the entire rendering pipeline
            RenderDevice::Render(this->window, dt);

            // transfer new frame to window
            this->window->SwapBuffers();

            auto timeEnd = std::chrono::steady_clock::now();
            dt = std::min(0.04, std::chrono::duration<double>(timeEnd - timeStart).count());

            if (kbd->pressed[Input::Key::Code::Escape])
                this->Exit();
        }
    }

    //------------------------------------------------------------------------------
    /**
    */
    void SpaceGameApp::Exit() { this->window->Close(); }

    //------------------------------------------------------------------------------
    /**
    */
    void SpaceGameApp::RenderUI() {
        if (this->window->IsOpen()) {
            ImGui::Begin("Debug");

            // bool show = true;
            // ImGui::ShowDemoWindow(&show);

            ImGui::SeparatorText("Debug Camera");
            ImGui::InputFloat3("Pos", &camera->pos[0]);

            Core::CVar* r_draw_light_spheres = Core::CVarGet("r_draw_light_spheres");
            int drawLightSpheres = Core::CVarReadInt(r_draw_light_spheres);
            if (ImGui::Checkbox("Draw Light Spheres", (bool*)&drawLightSpheres))
                Core::CVarWriteInt(r_draw_light_spheres, drawLightSpheres);

            Core::CVar* r_draw_light_sphere_id = Core::CVarGet("r_draw_light_sphere_id");
            int lightSphereId = Core::CVarReadInt(r_draw_light_sphere_id);
            if (ImGui::InputInt("LightSphereId", (int*)&lightSphereId))
                Core::CVarWriteInt(r_draw_light_sphere_id, lightSphereId);

            ImGui::End();

            Debug::DispatchDebugTextDrawing();
        }
    }

} // namespace Game
