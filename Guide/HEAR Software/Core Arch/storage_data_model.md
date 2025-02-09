# Overview
This document describes general rules for data storage in DynamoDB

## Common fields
Every table has the following fields:
1. "id" (UUIDv4)
2. "OrgID" (UUIDv4)
3. "RobotTypeID" (UUIDv4)
4. "RobotInstanceID" (UUIDv4)
5. "Address" (String): Allows specialization of entries, e.g. "PID/x", "PID/x/outdoors" in PIDs tables

## Tables
Table names used in configurations must have the `config-` prefix.