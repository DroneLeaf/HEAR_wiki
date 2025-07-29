# Overview
This document describes general rules for data storage in DynamoDB

## Tables
Table names used in configurations must have the `config-` prefix.

## Common fields
Every table has the following fields by definition:
1. `id` (UUIDv4) -> This is a mandatory unique field that is used as the primary key.
2. `organization_id` (UUIDv4) -> This is optional field that declares if the entry belongs to the organization with given organization_id
3. `robot_type_id` (UUIDv4) -> This is optional field that declares if the entry belongs to the robot type with given robot_type_id
4. `robot_instance_id` (UUIDv4) -> This is optional field that declares if the entry belongs to the robot instance with given robot_instance_id
5. `Address` (String): This is optional field. Allows specialization of entries, e.g. "PID/x", "PID/x/outdoors" in PIDs tables.

**Note:** entries with missing `id` field

## Recommended access pattern
**Note** the recommended access pattern is strictly followed by LeafFC.


There are four cardinality levels for entries in `config-` tables.
1. Instance specific: The `robot_instance_id` must be specified.
2. Type specific: The `robot_type_id` must be specified.
3. Org specific: The `organization_id` must be specified.
4. General: `robot_instance_id`, `robot_type_id`, `organization_id` must all be empty.

## Link with WebApp front-end

**Notes** with regards to assignment of `Common fields`:
1. `organization_id`: for personal and organization accounts alike.


### Specification format of table entries
config-mech-properties:robot_type_id[`Mandatory`],organization_id[`Mandatory`]
config-mech-properties/dist_between_mtr_drone_center[`Double`]