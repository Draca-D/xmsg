## Description
This libary wraps complex types that support code reflection. It allows handling of types as generic types, meaning member values can be extracted by name as opposed to traditional C struct accessors. Generic types can be be copied from one another allowing for at runtime translation between unknown types while maintaining type safety. Currently supported types:

- Google Protobuf
- JSON messages
- ROS2 messages
