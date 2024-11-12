# Software Development and Releasing Workflow

## Introduction

This document describes the software development and releasing workflow for the project.

Below is a diagram of the software development and releasing workflow.

![Development Workflow](media/SoftwareWorkflow.drawio.svg)

## Development Workflow

The development workflow consists of iterative steps that are repeated until the software is ready for a new release.

Below is a diagram that explains the process in detail.

![Development Workflow](media/DevelopmentWorkflow.drawio.svg)

## Releasing Workflow

The releasing workflow consists of steps that are followed to release a new version of the software.

Below is a diagram that explains the process in detail.

![Releasing Workflow](media/ReleasingWorkflow.drawio.svg)

## Versioning

The software versioning scheme follows the Semantic Versioning 2.0.0 specification.

The version number is in the format of `MAJOR.PATCH.INTERNAL`.

- `MAJOR`: Major version number.
- `PATCH`: Patch version number. Incremented for bug fixes or new features.
- `INTERNAL`: Internal version number. Incremented for internal releases.

For exemple, 
`1.0.0` is the first major release of the software.
`1.1.0` is the first patch release of the software.
`1.1.1` is the first internal (custom) release of the software that has been built on top of the `1.1.0` release.

