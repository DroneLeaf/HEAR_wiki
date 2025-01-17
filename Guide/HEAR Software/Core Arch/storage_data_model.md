# Overview
This document describes general rules for data storage in the DynamoDB

## Common fields
Every table has the following fields:
1. OrgID (UUIDv4)
2. RobotTypeID (UUIDv4)
3. RobotInstanceID (UUIDv4)
4. Address (String): Allows specialization of entries