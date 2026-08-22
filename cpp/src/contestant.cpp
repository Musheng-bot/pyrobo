#include "pyrobo/interface.hpp"

#include <memory>
#include <stdexcept>

namespace {

class ContestantNavigationContext final : public pyrobo::NavigationContext {
public:
    explicit ContestantNavigationContext(const pyrobo::Map& planning_map)
        : planning_map_(planning_map) {}

    const pyrobo::Map& planning_map() const noexcept {
        return planning_map_;
    }

private:
    const pyrobo::Map& planning_map_;
};

}  // namespace

namespace pyrobo {

std::unique_ptr<NavigationContext> nav_init(Simulator& sim) {
    return std::make_unique<ContestantNavigationContext>(sim.map());
}

void nav_run(Simulator& sim, NavigationContext& context) {
    auto* navigation_context = dynamic_cast<ContestantNavigationContext*>(&context);
    if (navigation_context == nullptr) {
        throw std::runtime_error("invalid navigation context");
    }

    const auto goal = sim.get_goal();
    if (!goal.has_value()) {
        sim.set_display_path({});
        sim.set_control(0.0, 0.0);
        return;
    }

    const auto pose = sim.get_pose();
    const auto feedback = sim.get_feedback();
    const auto& planning_map = navigation_context->planning_map();
    (void)pose;
    (void)feedback;
    (void)planning_map;

    // Candidate code starts here. Return world-coordinate path points and
    // robot-frame control values matching the configured command model.
    const Path path = {{pose.x, pose.y}, {goal->x, goal->y}};
    sim.set_display_path(path);
    sim.set_control(0.0, 0.0);
}

}  // namespace pyrobo
