# Software Development and Releasing Workflow

## Introduction

This document describes the process for development and releasing of the software-stack.

It assumes that the developer machine is setup with all needed packages.

## Development Workflow

Check the `SoftwareContributionWorkflow.drawio`

## Releasing Workflow

Releasing is automated in `dev` and `main` branches PRs. Additionally, it can be manually initiated in the `dev` branch in case hot-fixes were applied.

When initiating a PR to `dev` the image and patch version numbers of ORIN/RPI targets has to be supplied. They will be used later by `HEAR_FC` for compatibility checks.

When releasing with PR on `dev` the maintainer would be asked for `MAJOR` and `PATCH` version numbers AND the image and patch version numbers of ORIN/RPI targets.

As part of the build automation on `dev` and `main`, the version numbers of all compiled artifacts would match the tag version.

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

