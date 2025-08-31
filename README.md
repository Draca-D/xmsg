# XMSG - Cross-Message Library

A C++ library that provides a unified interface for working with different message types (ROS2, Protobuf, and JSON) through reflection and runtime type introspection. XMSG allows you to handle heterogeneous message collections and perform type conversions between different message formats at runtime.

## Features

- **Unified Message Interface**: Work with ROS2, Protobuf, and JSON messages through a common `GenericMessage` interface
- **Runtime Type Introspection**: Access message members by name without compile-time knowledge of the structure
- **Type-Safe Conversions**: Convert between different message formats while maintaining type safety
- **Heterogeneous Collections**: Store different message types in the same container (e.g., `std::vector<GenericMessage*>`)
- **Flexible Member Access**: Support for strict and lax member name matching
- **Automatic Type Coercion**: Intelligent conversion between compatible types (int ↔ double, etc.)

## Supported Message Types

1. **ROS2 Messages**: Any ROS2 message type with introspection support
2. **Google Protobuf**: Messages with reflection capabilities
3. **JSON Messages**: Dynamic JSON objects using nlohmann::json

## Installation

### Dependencies

- ROS2 (Galactic or later)
- Google Protobuf
- nlohmann_json
- rclcpp
- std_msgs

### Building

```bash
# In your ROS2 workspace
colcon build --packages-select xmsg
source install/setup.bash
```

## Core Concepts

### GenericMessage Base Class

All message wrappers inherit from `XMSG::GenericMessage`, providing:

```cpp
// Access members by name with type checking
const RawMemberWrapper get_member(const std::string& member_name, const Type expected_type) const;

// Set members by name
void set_member(const std::string& member_name, const RawMemberWrapper& data);

// Copy data between different message types
void copy_from(const GenericMessage* const message);
```

### RawMemberWrapper

A type-safe wrapper around raw data that supports:
- Automatic type coercion where sensible
- Template-based type extraction: `member.as<int>()`
- Runtime type information

### Type System

XMSG uses a flag-based type system supporting:
```cpp
enum Type {
    NONE = 0,
    ARRAY = 1,
    CHAR = 2,
    UINT = 4,
    INT = 8,
    DOUBLE = 16,
    STRING = 32,
    BOOL = 64,
    GENERIC = 128,
    NOTSET = 256
};
```

## Usage Examples

### Basic ROS2 Message Handling

```cpp
#include <xmsg/rosgenericmessage.h>

// Create a generic wrapper for a ROS2 message type
auto ros_msg = std::make_unique<XMSG::ROSGenericMessage>("geometry_msgs/msg/Twist");

// Set linear velocity
XMSG::RawMemberWrapper linear_x_wrapper;
double velocity = 1.5;
linear_x_wrapper = /* create wrapper with velocity */;
ros_msg->set_member("linear.x", linear_x_wrapper);

// Get angular velocity
auto angular_z = ros_msg->get_member("angular.z", XMSG::Type::DOUBLE);
double z_value = angular_z.as<double>();
```

### JSON Message Manipulation

```cpp
#include <xmsg/jsongenericmessage.h>

// Create from JSON string
std::string json_str = R"({"name": "robot1", "position": {"x": 10.5, "y": 20.3}})";
auto json_msg = std::make_unique<XMSG::JSONGenericMessage>(json_str);

// Access nested members
auto x_pos = json_msg->get_member("position.x", XMSG::Type::DOUBLE);
double x = x_pos.as<double>();

// Modify values
XMSG::RawMemberWrapper new_name;
std::string robot_name = "robot2";
// ... setup wrapper
json_msg->set_member("name", new_name);

// Export back to JSON
std::string updated_json = json_msg->dump(2); // Pretty print with 2-space indent
```

### Protobuf Message Handling

```cpp
#include <xmsg/protogenericmessage.h>

// Create from protobuf type name
auto proto_msg = std::make_unique<XMSG::ProtoGenericMessage>("my_package.MyMessage");

// Set a string field
XMSG::RawMemberWrapper str_wrapper;
std::string message = "Hello XMSG";
// ... setup wrapper
proto_msg->set_member("message_field", str_wrapper);

// Debug output
std::cout << proto_msg->get_debug_string() << std::endl;
```

### Cross-Format Message Conversion

```cpp
// Convert ROS2 message to JSON
auto ros_msg = std::make_unique<XMSG::ROSGenericMessage>("sensor_msgs/msg/Temperature");
auto json_msg = std::make_unique<XMSG::JSONGenericMessage>();

// Copy structure and data from ROS to JSON
json_msg->copy_from(ros_msg.get());

std::string json_output = json_msg->dump();
std::cout << "Converted to JSON: " << json_output << std::endl;

// Convert JSON to Protobuf
auto proto_msg = std::make_unique<XMSG::ProtoGenericMessage>("temperature_msg.Temperature");
proto_msg->copy_from(json_msg.get());
```

### Heterogeneous Message Collections

```cpp
// Store different message types in the same container
std::vector<std::unique_ptr<XMSG::GenericMessage>> messages;

messages.emplace_back(std::make_unique<XMSG::ROSGenericMessage>("std_msgs/msg/String"));
messages.emplace_back(std::make_unique<XMSG::JSONGenericMessage>(R"({"count": 42})"));
messages.emplace_back(std::make_unique<XMSG::ProtoGenericMessage>("my_proto.Status"));

// Process all messages uniformly
for (auto& msg : messages) {
    std::cout << "Message type: " << msg->get_type() << std::endl;
    std::cout << "Identifier: " << msg->get_identifier() << std::endl;
    
    // Log all members
    msg->log_members();
    
    // Access common fields if they exist
    if (msg->has_member("timestamp")) {
        auto ts = msg->get_member("timestamp", XMSG::Type::UINT);
        uint64_t timestamp = ts.as<uint64_t>();
        std::cout << "Timestamp: " << timestamp << std::endl;
    }
}
```

### Advanced: Message Configuration

```cpp
#include <xmsg/Configuration/genericmessageconfigurator.h>

class CustomConfigurator : public GenericMessageConfigurator {
public:
    std::string get_mapped_name(const std::string& identifier,
                               const std::string& type,
                               const std::string& name) override {
        // Map field names between different message formats
        if (name == "pos_x") return "position.x";
        if (name == "pos_y") return "position.y";
        return name; // Default: no mapping
    }
    
    std::string get_transformation(const std::string& identifier,
                                 const std::string& type,
                                 const std::string& name) override {
        // Apply mathematical transformations
        if (name == "temperature" && identifier == "ROSMessage") {
            return "+273.15"; // Convert Celsius to Kelvin
        }
        return ""; // No transformation
    }
    
    std::string check_if_child(const std::string& identifier,
                              const std::string& type,
                              const std::string& name) override {
        // Handle nested message extraction
        return "";
    }
};

// Apply custom configuration
auto config = std::make_shared<CustomConfigurator>();
XMSG::GenericMessage::set_configuration_engine(config);

// Enable automatic simple translations
XMSG::GenericMessage::auto_perform_simple_translation();
```

### Working with Arrays

```cpp
// Handle array fields
auto msg = std::make_unique<XMSG::ROSGenericMessage>("sensor_msgs/msg/PointCloud2");

// Get points array
auto points_array = msg->get_member("data", XMSG::Type::ARRAY | XMSG::Type::UINT);

// Extract as vector
auto points = points_array.as<std::vector<uint8_t>>();
std::cout << "Point cloud has " << points.size() << " bytes" << std::endl;

// Modify array
std::vector<uint8_t> new_data = {1, 2, 3, 4, 5};
XMSG::RawMemberWrapper new_array_wrapper;
// ... setup wrapper with new_data
msg->set_member("data", new_array_wrapper);
```

### Error Handling

```cpp
try {
    auto msg = std::make_unique<XMSG::ROSGenericMessage>("invalid_msg_type");
} catch (const XMSG::BrokenMessage& e) {
    std::cerr << "Failed to create message: " << e.what() << std::endl;
}

// Check if member exists before accessing
if (msg->has_member("optional_field")) {
    auto field = msg->get_member("optional_field", XMSG::Type::STRING);
    std::string value = field.as<std::string>();
}
```

## Advanced Features

### Strict vs. Lax Member Matching

```cpp
// Strict matching: exact field names required
msg->set_strict(true);
auto field1 = msg->get_member("fieldName", XMSG::Type::INT); // Must match exactly

// Lax matching: ignores case and underscores
msg->set_strict(false);
auto field2 = msg->get_member("field_name", XMSG::Type::INT); // Matches "fieldName", "FieldName", etc.
```

### Automatic Type Extraction

```cpp
// Enable automatic child extraction for simple generic types
XMSG::GenericMessage::set_auto_extract_child(true);
XMSG::GenericMessage::set_auto_set_child(true);

// Now single-member generic messages are automatically unwrapped
```

## ROS2 Galactic Limitations

Due to limitations in ROS2 Galactic, arrays of complex message types require manual registration:

```cpp
// For each complex message type used in arrays, register a handler
XMSG::ROSGenericMessage::register_nested_handler(
    "geometry_msgs/msg/Point",
    [](void* ros_member_loc, void* data) {
        XMSG::ROSGenericMessage::handle_repeated_nested_msg<geometry_msgs::msg::Point>(
            ros_member_loc, data);
    }
);
```

This limitation will be removed when upgrading to ROS2 Humble or later.

## CMake Integration

```cmake
find_package(xmsg REQUIRED)

add_executable(my_app src/main.cpp)
target_link_libraries(my_app XMSG::GenericMessage)
```

## Performance Considerations

- Message introspection has runtime overhead compared to direct member access
- Use `get_member_reference()` for high-frequency access to avoid copies
- Prefer batch operations when converting between message types
- Consider caching `GenericMessage` instances for repeated use

## Thread Safety

XMSG is not thread-safe by default. When using in multi-threaded applications:
- Protect shared `GenericMessage` instances with mutexes
- Create separate instances per thread when possible
- Configuration changes affect all instances globally

## Contributing

See the project repository for contribution guidelines and issue reporting.

## License

This project is licensed under the MIT License - see the LICENSE.txt file for details.