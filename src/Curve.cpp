#include "Curve.h"

#include "AK/Math.h"
#include "Entity.h"
#include "Game/LevelController.h"

#include <glm/gtc/type_ptr.inl>

#include <iostream>

#if EDITOR
#include "imgui_extensions.h"
#include <imgui.h>
#include <implot.h>
#endif

std::shared_ptr<Curve> Curve::create()
{
    return std::make_shared<Curve>(AK::Badge<Curve> {});
}

Curve::Curve(AK::Badge<Curve>)
{
}

Curve::Curve() = default;

#if EDITOR
void Curve::draw_editor()
{
    Component::draw_editor();

    std::array const connection_types = {"Linear", "Catmull-Rom"};
    i32 current_item_index = static_cast<i32>(connection);
    if (ImGui::Combo("Point connection type", &current_item_index, connection_types.data(), connection_types.size()))
    {
        connection = static_cast<PointsConnection>(current_item_index);
    }

    if (ImGui::Checkbox("Clamp X axis", &clamp_x))
    {
        for (u32 i = 0; i < points.size(); i++)
        {
            if (clamp_x)
            {
                float left_clamp = 0.0f;
                float right_clamp = 1.0f;

                if (i > 0)
                {
                    left_clamp = points[i - 1].x;
                }

                if (i < points.size() - 1)
                {
                    right_clamp = points[i + 1].x;
                }

                points[i].x = glm::clamp(points[i].x, left_clamp, right_clamp);
            }
        }
    }

    if (ImPlot::BeginPlot("Path visualised"))
    {
        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2.0f);
        ImPlot::SetupLegend(ImPlotFlags_NoLegend);

        // Generate smooth curve
        int const num_segments = 20; // Number of segments between each point
        switch (connection)
        {
        case PointsConnection::Linear:
            curve = points;
            break;

        case PointsConnection::CatmullRom:
            curve = AK::Math::catmull_rom_curve(points, num_segments);
            break;
        }

        std::vector<float> xs, ys;
        std::vector<float> real_xs, real_ys;
        for (auto const& p : curve)
        {
            xs.push_back(p.x);
            ys.push_back(p.y);
        }

        for (auto const& p : points)
        {
            real_xs.push_back(p.x);
            real_ys.push_back(p.y);
        }

        ImPlot::PlotLine("##Line", xs.data(), ys.data(), curve.size());

        for (u32 i = 0; i < points.size(); i++)
        {
            double px = real_xs[i];
            double py = real_ys[i];
            if (ImPlot::DragPoint(i, &px, &py, ImVec4(0, 0.9f, 0, 1), 4))
            {
                points[i].x = px;

                if (clamp_x)
                {
                    float left_clamp = 0.0f;
                    float right_clamp = 1.0f;

                    if (i > 0)
                    {
                        left_clamp = points[i - 1].x;
                    }

                    if (i < points.size() - 1)
                    {
                        right_clamp = points[i + 1].x;
                    }

                    points[i].x = glm::clamp(points[i].x, left_clamp, right_clamp);
                }

                points[i].y = py;
            }
        }

        ImPlot::EndPlot();
    }

    if (ImGui::Button("Add point"))
    {
        if (points.size() == 0)
        {
            points.push_back({0.1f, 0.5f});
        }
        else
        {
            points.push_back({points.back().x + 0.1f, points.back().y});

            points.back().x = glm::clamp(points.back().x, 0.0f, 1.0f);
        }
    }

    for (u32 i = 0; i < points.size(); i++)
    {
        ImGuiEx::InputFloat2(("Position##" + std::to_string(i)).c_str(), glm::value_ptr(points[i]));
        ImGui::SameLine();
        if (ImGui::Button(("Remove point##" + std::to_string(i)).c_str()))
        {
            points.erase(points.begin() + i);
        }
    }
}
#endif

float Curve::length() const
{
    float distance = 0.0f;
    for (u32 i = 0; i < curve.size() - 1; i++)
    {
        distance += glm::distance(curve[i], curve[i + 1]);
    }

    return distance;
}

glm::vec2 Curve::get_point_at(float x) const
{
    if (points.empty())
    {
        return {0.0f, 0.0f};
    }

    if (points.size() == 1)
    {
        return points[0];
    }

    x = glm::clamp(x, 0.0f, 1.0f);

    float const path_length = length();
    float const desired_length = path_length * x;
    float distance = 0.0f;

    for (u32 i = 0; i < points.size() - 1; i++)
    {
        float const segment_length = glm::distance(points[i], points[i + 1]);

        if (distance + segment_length >= desired_length)
        {
            float const segment_ratio = (desired_length - distance) / segment_length;
            return glm::mix(points[i], points[i + 1], segment_ratio);
        }

        distance += segment_length;
    }

    return points.back();
}

float Curve::get_y_at(float x) const
{
    if (points.empty())
    {
        return 0.0f;
    }

    x = glm::clamp(x, 0.0f, 1.0f);

    for (u32 i = 0; i < points.size() - 1; i++)
    {
        if (x >= points[i].x && x <= points[i + 1].x)
        {
            float const x0 = points[i].x;
            float const y0 = points[i].y;
            float const x1 = points[i + 1].x;
            float const y1 = points[i + 1].y;

            float const t = (x - x0) / (x1 - x0);
            float const y = y0 + t * (y1 - y0);
            return y;
        }
    }

    return 0.0f;
}

void Curve::add_points(std::initializer_list<glm::vec2> new_points)
{
    points.insert(points.end(), new_points.begin(), new_points.end());
}
