#include "config.h"
#include "debug_camera.h"

#include "cameramanager.h"
#include "core/math.h"
#include "input/inputserver.h"


namespace Render {

    void DebugCamera::Update(float dt) {
        using namespace Input;

        const auto camera = CameraManager::GetCamera(CAMERA_MAIN);

        const auto mouse = GetDefaultMouse();
        const auto kbd = GetDefaultKeyboard();

        auto vel = glm::vec3(0);

        if (kbd->held[Key::W]) { vel.z += 1.0; }
        if (kbd->held[Key::S]) { vel.z -= 1.0; }
        if (kbd->held[Key::A]) { vel.x += 1.0; }
        if (kbd->held[Key::D]) { vel.x -= 1.0; }

        vel = this->trans * glm::vec4(vel, 0.0f);

        if (kbd->held[Key::Q]) { vel.y += 1.0; }
        if (kbd->held[Key::E]) { vel.y -= 1.0; }
        vel = Math::safe_normal(vel);

        auto md = glm::vec2(0);
        if (mouse->held[Mouse::Button::RightButton]) {
            md = mouse->delta;
        }

        const auto yaw = glm::quat(glm::vec3(0, -md.x, 0) * this->rot_speed * dt);
        const auto pitch = glm::quat(glm::vec3(md.y, 0, 0) * this->rot_speed * dt);
        this->ort = yaw * this->ort * pitch;

        this->pos += vel * this->speed * dt;
        this->trans = glm::translate(this->pos) * static_cast<glm::mat4>(this->ort);

        camera->view = glm::lookAt(this->pos, this->pos + glm::vec3(this->trans[2]), glm::vec3(this->trans[1]));
    }

}
