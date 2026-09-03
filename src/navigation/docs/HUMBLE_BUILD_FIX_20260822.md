# ROS 2 Humble build fix — 2026-08-22

This patch addresses two build failures observed with ROS 2 Humble / GCC 11:

1. `std::clamp()` received a ROS integer parameter (`int64`) together with `int` bounds. The parameter is now declared explicitly as `std::int64_t`, clamped using the same type, then safely narrowed to the existing `int` member after applying `[10, 5000]` bounds.
2. `rclcpp::Time::to_msg()` is not part of the Humble `rclcpp::Time` API. The fallback timestamp is now converted using the supported `operator builtin_interfaces::msg::Time()` via `static_cast`.

No estimator, GNSS fusion, Nav2, or safety behavior was changed by this patch.
