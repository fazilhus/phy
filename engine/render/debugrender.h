#pragma once
#include "physics/plane.h"
//------------------------------------------------------------------------------
/**
	@file	debugrender.h

	Contains debug rendering functions.

	@copyright
	(C) 2021 Individual contributors, see AUTHORS file
*/
//------------------------------------------------------------------------------

namespace Debug {
    enum RenderMode {
        Normal = 1,
        AlwaysOnTop = 2,
        WireFrame = 4
    };

    ///Draw text in screenspace
    void DrawDebugText(const char* text, glm::vec3 point, const glm::vec4 color);
    ///Draw line that spans from startpoint to endpoint (worldspace coordinates)
    void DrawLine(
        const glm::vec3& startPoint, const glm::vec3& endPoint, const float lineWidth, const glm::vec4& startColor,
        const glm::vec4& endColor, const RenderMode& renderModes = RenderMode::Normal
        );
    void DrawLine(
        const glm::vec3& startPoint, const glm::vec3& endPoint, const float lineWidth, const glm::vec4& color,
        const RenderMode& renderModes = RenderMode::Normal
        );
    void DrawQuad(
        const glm::mat4& transform, const glm::vec4& color, const RenderMode render_mode = RenderMode::Normal,
        const float line_width = 1.0f
        );
    void DrawQuad(
        const glm::vec3& pos, const glm::quat& rot, const float scale, const glm::vec4& color,
        const RenderMode render_mode = RenderMode::Normal, const float line_width = 1.0f
        );
    void DrawQuad(
        const glm::vec3& pos, const glm::quat& rot, const float scale, const glm::vec4& color,
        const float width, const float height,
        const RenderMode render_mode = RenderMode::Normal, const float line_width = 1.0f
        );
    ///Draws a unit colored cube at position with rotation and scale
    void DrawBox(
        const glm::vec3& position, const glm::quat& rotation, const float scale, const glm::vec4& color,
        const RenderMode renderModes = RenderMode::Normal, const float lineWidth = 1.0f
        );
    ///Draws a colored box at position with rotation. Size of box will be width (x-axis), height (y-axis), length (z-axis).
    void DrawBox(
        const glm::vec3& position, const glm::quat& rotation, const float width, const float height, const float length,
        const glm::vec4& color, const RenderMode renderModes = RenderMode::Normal, const float lineWidth = 1.0f
        );
    ///Draws a colored box with transform
    void DrawBox(
        const glm::mat4& transform, const glm::vec4& color, const RenderMode renderModes = RenderMode::Normal,
        const float lineWidth = 1.0f
        );
    void DrawGrid(
        const RenderMode renderModes = RenderMode::Normal, const float lineWidth = 1.0f
        );
    void DrawPlane(
        const Physics::Plane& plane, const RenderMode render_modes = RenderMode::Normal, const float line_width = 1.0f
        );

    void InitDebugRendering();
    void DispatchDebugDrawing();
    void DispatchDebugTextDrawing();
} // namespace Debug
