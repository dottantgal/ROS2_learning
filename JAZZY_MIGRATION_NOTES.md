# ROS 2 Jazzy Migration Notes

## Overview
This document summarizes the migration from ROS 2 Humble to ROS 2 Jazzy and the conversion of all C++ packages to Python equivalents.

## Migration Summary

### Branch Structure
- **humble**: Original branch with C++ packages only
- **jazzy**: New branch with both C++ and Python packages, updated for Jazzy
- **main**: Now points to jazzy branch (jazzy is the default)

### Key Changes

#### 1. Complete Python Conversion
All C++ packages have been converted to Python equivalents:
- ✅ `parameters` → `parameters_py`
- ✅ `service_server_and_client` → `service_server_and_client_py`
- ✅ `message_sync` → `message_sync_py`
- ✅ `actions/action_tutorial` → `actions/action_tutorial_py`
- ✅ `actions/custom_action` → `actions/custom_action_py` (interfaces only - Python packages depend on this)
- ✅ `dynamic_tf2_publisher` → `dynamic_tf2_publisher_py`
- ✅ `create_library_with_header` → `create_library_with_header_py`
- ✅ `plugins/vehicle_base` → `plugins/vehicle_base_py`
- ✅ `plugins/vehicle_plugins` → `plugins/vehicle_plugins_py`

**Important**: Custom interfaces (messages, services, actions) are defined in C++ packages. Python packages depend on these C++ packages rather than duplicating the interface definitions.

#### 2. Test Files Removal
All test files have been removed from Python packages as requested:
- Removed `test/` directories
- Removed `tests_require` from `setup.py`
- Removed test dependencies from `package.xml`

#### 3. Plugin System
- **C++**: Uses `pluginlib` (standard ROS 2 approach)
- **Python**: Uses setuptools entry points with `pkg_resources`/`importlib.metadata`

#### 4. Library Structure
- **C++**: Header files (`.h`, `.hpp`) with implementation files
- **Python**: Python modules (`.py`) with `__init__.py` for exports

## Jazzy-Specific Considerations

### API Compatibility
- All packages use Jazzy-compatible APIs
- Action server/client implementations follow Jazzy patterns
- Parameter handling uses modern Jazzy API:
  - Updated from `get_parameter(name, variable)` to `get_parameter(name).as_<type>()`
  - Using `as_string()`, `as_int()`, `as_double()` methods
- Updated parameter types from `float` to `double` for better precision

### Build System
- C++ packages: `ament_cmake` with C++17 standard (Jazzy recommendation)
- CMake minimum version: 3.8 (updated from 3.5)
- Python packages: `ament_python` with proper `setup.py` configuration
- All packages use package format 3

### Dependencies
- Updated to Jazzy-compatible versions
- Python packages use `rclpy` and related Jazzy packages
- C++ packages use `rclcpp` and related Jazzy packages

## Package Status

### C++ Packages (13 packages)
All C++ packages are maintained and compatible with Jazzy:
1. `start_with_simple_nodes`
2. `publisher_and_subscriber`
3. `custom_msg_and_srv`
4. `service_server_and_client`
5. `parameters`
6. `plugins/vehicle_base`
7. `plugins/vehicle_plugins`
8. `actions/action_tutorial`
9. `actions/custom_action`
10. `message_sync`
11. `create_library_with_header/publisher_library`
12. `create_library_with_header/use_library`
13. `dynamic_tf2_publisher`

### Python Packages (12 packages)
All Python packages are new implementations:
1. `start_with_simple_nodes_py`
2. `publisher_and_subscriber_py`
3. `custom_msg_and_srv_py` (depends on C++ `custom_msg_and_srv` for interfaces)
4. `service_server_and_client_py` (depends on C++ `custom_msg_and_srv` for interfaces)
5. `parameters_py`
6. `plugins/vehicle_base_py`
7. `plugins/vehicle_plugins_py`
8. `actions/action_tutorial_py` (depends on C++ `custom_action` for interfaces)
9. ~~`actions/custom_action_py`~~ (removed - interfaces defined in C++ `custom_action`)
10. `message_sync_py`
11. `create_library_with_header_py/publisher_library_py`
12. `create_library_with_header_py/use_library_py`
13. `dynamic_tf2_publisher_py`

**Note**: `custom_action_py` was removed following ROS 2 best practices. Custom interfaces are defined in C++ packages, and Python packages depend on them.

## Documentation
- All packages now have comprehensive README.md files
- Main README.md updated for Jazzy branch
- Each README includes:
  - Overview and purpose
  - Prerequisites
  - Building instructions
  - Usage examples
  - Key concepts
  - File descriptions

## Jazzy-Specific Improvements Applied

### 1. C++ Standard Update
- ✅ Updated all CMakeLists.txt from C++14 to C++17 (Jazzy recommendation)
- ✅ All 13 C++ packages updated

### 2. CMake Version Update
- ✅ Updated minimum CMake version from 3.5 to 3.8 (Jazzy requirement)
- ✅ All 13 C++ packages + 1 Python action package updated

### 3. Parameter API Modernization
- ✅ Updated `parameters` package: `get_parameter(name, var)` → `get_parameter(name).as_<type>()`
- ✅ Updated `dynamic_tf2_publisher` package: Modern parameter API with type conversion
- ✅ Changed parameter types from `float` to `double` for better precision

### 4. Code Quality
- ✅ All packages follow Jazzy best practices
- ✅ Action servers/clients use modern interface-based patterns
- ✅ Python packages use modern `.value` accessor for parameters

### 5. Python-Specific API Changes

#### Rate Creation API
- ❌ **Deprecated in Jazzy**: `rclpy.create_rate(frequency, clock)` 
- ✅ **New Jazzy API**: `node.create_rate(frequency)`
  - Example: `rate = node.create_rate(2)  # 2 Hz`
  - The rate object has a `sleep()` method: `rate.sleep()`
  - Reference: [ROS 2 Jazzy Migration Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1/Migrating-Python-Package-Example.html#create-a-rate)

#### Graceful Shutdown Handling
- ✅ All Python nodes now handle `KeyboardInterrupt` gracefully
- ✅ Added `rclpy.ok()` checks before calling `rclpy.shutdown()` to prevent double shutdown errors
- ✅ Added `rclpy.ok()` checks before logging in KeyboardInterrupt handlers to prevent "context is invalid" errors
- Pattern used:
  ```python
  try:
      rclpy.spin(node)
  except KeyboardInterrupt:
      if rclpy.ok():
          node.get_logger().info("Shutting down node...")
  finally:
      node.destroy_node()
      if rclpy.ok():
          rclpy.shutdown()
  ```

#### Custom Interface Package Architecture
- ✅ **Best Practice**: Custom interfaces (messages, services, actions) are defined in C++ packages using `ament_cmake` and `rosidl_generate_interfaces`
- ✅ Python packages depend on C++ interface packages, not duplicate them
- ✅ Example: `service_server_and_client_py` imports from `custom_msg_and_srv.srv`, not `custom_msg_and_srv_py.srv`
- ✅ Removed duplicate `custom_action_py` package - Python action packages now depend on `custom_action` (C++)
- ✅ Updated all imports and package.xml dependencies accordingly

#### Action Server API Changes
- ❌ **Deprecated in Jazzy**: `goal_handle.succeed(result)` and `goal_handle.canceled(result)` with result parameter
- ✅ **New Jazzy API**: `goal_handle.succeed()` and `goal_handle.canceled()` called without arguments
- ✅ **New Jazzy API**: Execute callback must return the result object instead of setting `goal_handle.result`
- ✅ Pattern:
  ```python
  def execute_callback(self, goal_handle):
      result = Concatenate.Result()
      result.final_concatenation = concatenation
      goal_handle.succeed()  # No result parameter
      return result  # Return result from callback
  ```
- ✅ For cancellation: `goal_handle.canceled()` then `return result`

#### Action Client API Changes
- ❌ **Deprecated in Jazzy**: `rclpy.action.GoalStatus` (does not exist)
- ✅ **New Jazzy API**: Import `GoalStatus` from `action_msgs.msg`
  ```python
  from action_msgs.msg import GoalStatus
  if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
      # Handle success
  ```
- ✅ **Feedback Callback Change**: Feedback callback receives a `FeedbackMessage` object
- ✅ Access feedback via `feedback.feedback.field_name` (not `feedback.field_name`)
  ```python
  def feedback_callback(self, feedback):
      self.get_logger().info(f'Feedback: {feedback.feedback.partial_concatenation}')
  ```

#### Message Synchronization API
- ❌ **Deprecated in Jazzy**: `ApproximateTimeSynchronizer(subscribers, queue_size)` without slop
- ✅ **New Jazzy API**: `ApproximateTimeSynchronizer(subscribers, queue_size, slop)`
  ```python
  self.sync = ApproximateTimeSynchronizer(
      [sub1, sub2],
      10,  # queue_size
      0.1  # slop parameter (required in Jazzy)
  )
  ```

#### TF2 Transformations
- ❌ **Removed in Jazzy**: `tf_transformations` package (not available)
- ✅ **Solution**: Manual implementation of `quaternion_from_euler` using `math` module
- ✅ Pattern:
  ```python
  import math
  
  def quaternion_from_euler(roll, pitch, yaw):
      # Manual implementation using standard formulas
      cy = math.cos(yaw * 0.5)
      sy = math.sin(yaw * 0.5)
      # ... (full implementation)
      return (qx, qy, qz, qw)
  ```

### Build Order
When building packages that depend on custom interfaces:
1. Build the C++ interface package first: `colcon build --packages-select custom_msg_and_srv`
2. Then build dependent Python packages: `colcon build --packages-select service_server_and_client_py`
3. Or build all together: `colcon build --symlink-install`

## Notes
- The repository maintains both C++ and Python implementations side-by-side
- Users can learn both `rclcpp` and `rclpy` APIs
- All examples are tested and verified for ROS 2 Jazzy
- Custom interfaces follow ROS 2 best practice: defined in C++ packages, used by both C++ and Python packages

