#pragma once
#include <memory>
#include <string>

class Entity;

class Component : public std::enable_shared_from_this<Component>
{
public:
    Component();
    virtual ~Component() = default;

    virtual void initialize();
    virtual void uninitialize();

    virtual void reprepare();

    virtual void awake();
    virtual void start();
    virtual void update();
    virtual void on_enabled();
    virtual void on_disabled();

    void destroy_immediate();

    virtual void draw_editor();

    std::shared_ptr<Entity> entity;

    bool has_been_awaken = false;
    bool has_been_started = false;

    void set_can_tick(bool const value);
    bool get_can_tick() const;

    void set_enabled(bool const value);
    bool enabled() const;

    std::string guid;

private:
    bool m_enabled = true;
    bool m_can_tick = false;
};
